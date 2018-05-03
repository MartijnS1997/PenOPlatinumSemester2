package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import TestbedAutopilotInterface.Overseer.AutopilotDelivery;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

/**
 * Created by Martijn on 18/02/2018.
 * A class of takeoff controllers, responsible for controlling the takeoff of the drone
 */
public class AutopilotTakeoffController extends Controller {

    public AutopilotTakeoffController(AutoPilot autopilot) {
        super(autopilot);
    }

    /**
     * Getter for the control actions used to steer the drone.
     * first the controller tries to get to the right altitude (the assigned cruising altitude)
     * secondly it tries to stabilize on that altitude
     * @param currentInputs the outputs currently received by the autopilot from the testbed
     * @param previousInputs the inputs previously received by the autopilot from the testbed
     * @return the control outputs needed for a takeoff
     * note that the drone waits for a valid package to deliver if there is no such package the drone will not take off
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //generate the control outputs
        ControlOutputs outputs = new ControlOutputs(getStandardOutputs());
        //check if it has a package to deliver
        if(!hasPackageToDeliver()){
            //if not, we do not take off yet
            return outputs;
        }
        //then get the pitch controls
        this.pitchControl(outputs, currentInputs, previousInputs);
        //check for AOA
        this.angleOfAttackControl(getAOAMargin(), outputs, currentInputs, previousInputs);
        //get thrust controls
        this.thrustControl(outputs, currentInputs, previousInputs);
        //return the current outputs
        return outputs;
    }

    @Override
    public void reset() {
        this.getPitchController().reset();
        this.getPitchStabilizerPID().reset();
        this.getThrustController().reset();
    }

    /**
     * Checks if the drone has any packages to deliver, if not the drone remains idle and does not take off
     * @return true if the drone has a package to deliver
     */
    private boolean hasPackageToDeliver(){
        //get the autopilot
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        AutopilotDelivery delivery = communicator.getCurrentRequest();
        return delivery != null;
    }

    /**
     * Generates the control actions concerning the pitch of the drone during takeoff
     * @param outputs the outputs to write the result to (this parameter will be modified)
     * @param currentInputs the inputs most recently received from the testbed, used to infer the parameters from
     * @param previousInputs the previous inputs received from the testbed, also used to infer the parameters from
     */
    private void pitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the set point for the pitch
        float referencePitch = this.getReferencePitch();
        //get the current pitch from the inputs
        float currentPitch = Controller.extractPitch(currentInputs);
        //calculate te error
        float errorPitch = referencePitch - currentPitch;
        //get the outputs from the pid controller
        PIDController pitchPid = this.getPitchController();
        float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
        //if the reference pitch is larger than the current pitch we have a positive error pitch fed into
        //the PID, the pid will return a negative output --> negative PID output = steer upwards
        //if reference pitch < current pitch, we'll get a negative PID input and a positive output
        //we'll need to steer upwards
        float pidOutputs = pitchPid.getPIDOutput(errorPitch,deltaTime);

        //now calculate the control actions based on the error on the pitch
        //take the linear approach, we want the same inclination of the horizontal stabilizer as the error
        //if possible (first iteration) if we need to go up, negative inclination, if we need to go down
        //positive inclination
        float horizontalInclination = capInclination(pidOutputs, STANDARD_HORIZONTAL, MAX_HORIZONTAL);
        //put it into the outputs
        outputs.setHorStabInclination(horizontalInclination);
    }

    /**
     * Wrapper for the method in the main controller class to regulate the thrust if the velocity
     * to be maintained is stationary (and thus can be controlled by a cruise control)
     * @param outputs the outputs to write the thrust to and read the inclinations from
     * @param currentInputs the most recent inputs from the testbed
     * @param previousInputs the previous inputs from the testbed
     * note: must be invoked only after all the control angles are known because the output inclinations
     *       are used to calculate the thrust needed to acquire the reference velocity
     */
    private void thrustControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the pid responsible for the trust
        PIDController thrustControl = this.getThrustController();
        //get the reference velocity used by the controller
        float referenceVelocity = this.getReferenceVelocity();
        //check if the reference velocity is already configured
        if(referenceVelocity < 0){
            //calculate the reference velocity
            referenceVelocity = this.getAutopilot().getPhysXOptimisations().calcStableZeroPitchVelocity(STANDARD_MAIN);
            //save the reference velocity
            this.setReferenceVelocity(referenceVelocity);
        }
        //then get the outputs from the standard cruise control implemented in the main controller class
        this.flightCruiseControl(outputs,currentInputs, previousInputs, thrustControl, referenceVelocity);
    }

    private void stabilizePitchControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the pitch difference
        float pitchDiffAngle = this.getRefPitchDiff(currentInputs);
        Vector orientation = Controller.extractOrientation(currentInputs);
        float pitchAngle = -this.getRefPitchDiff(currentInputs);
        //the pitch should be 0;
        //System.out.println(pitch);
        PIDController pitchPid = this.getPitchStabilizerPID();
        float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitchAngle,  deltaTime);
        //System.out.println("Pitch result PID" + PIDControlActions);
        //adjust the horizontal stabilizer
        float horizontalInclination = STANDARD_HORIZONTAL - PIDControlActions;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), MAX_HORIZONTAL);
        outputs.setHorStabInclination(horizontalInclination);
    }

    /**
     * Calculates the pitch difference, the angle between the reference vector (the vector between
     * the pitch reference point and the position of the drone) transformed to the drone axis system and
     * projected onto the yz plane in the drone axis system, and the heading vector (0,0,-1) in the drone axis system.
     * The direction is determined by the cross product of the heading vector and the projected and transformed reference
     * vector.
     * @return returns a pos angle if the drone needs to go up, and a negative angle if the drone needs to go down
     */
    private float getRefPitchDiff(AutopilotInputs_v2 currentInputs){
        //get the difference vector
        //get the position of the drone
        Vector dronePos = Controller.extractPosition(currentInputs);
        //get the position of the way point
        Vector pitchRefPoint = this.getPitchReference(currentInputs);
        //get the ref vector
        Vector ref = pitchRefPoint.vectorDifference(dronePos);
        //transform it to the drone axis system
        //first get the current orientation
        Vector orientation = Controller.extractOrientation(currentInputs);
        Vector refDrone = PhysXEngine.worldOnDrone(ref, orientation);
        //then project it onto the yz plane: normal vector (1,0,0)
        Vector normalYZ = new Vector(1,0,0);
        Vector projRefDrone = refDrone.orthogonalProjection(normalYZ);
        //calculate the angle between the heading vector (0,0,-1) and the reference
        Vector headingVect = new Vector(0,0,-1);
        float angle = abs(projRefDrone.getAngleBetween(headingVect));

        //then get the vector product for the direction(the x-component)
        float direction = headingVect.crossProduct(projRefDrone).getxValue();

        float res = angle*signum(direction);
        //check for NaN
        if(Float.isNaN(res)){
            return 0; // NaN comes from the angle
        }
        //else return the result
        return res;

    }

    /**
     * Get the reference point for the pitch used by the takeoff controller
     * @param currentInputs the inputs most recently received from the testbed used to infer the parameters from
     * @return a vector containing a reference point for the drone in the world axis system
     */
    private Vector getPitchReference(AutopilotInputs_v2 currentInputs){
        //get the position & orientation of the drone
        Vector position = extractPosition(currentInputs);
        Vector orientation = extractOrientation(currentInputs);
        float cruisingAltitude = this.getCruisingAltitude();
        //get the relative distance for the reference point (located on the negative z-axis in the
        //heading axis system
        float pitchReferenceDistance = this.getPitchRefDistance();
        Vector refPointHeadingAxis = new Vector(0,0,-pitchReferenceDistance);
        //transform the reference point to the world axis system (relative to drone position)
        Vector refPointWorldAxisRel = PhysXEngine.headingOnWorld(refPointHeadingAxis, orientation);
        //add the x and z position of the drone to the heading vector and there after add
        //the cruising altitude to get the real set point
        Vector relativePos = new Vector(position.getxValue(), cruisingAltitude, position.getzValue());
        Vector refPointWorldAxis = refPointWorldAxisRel.vectorSum(relativePos);

        return refPointWorldAxis;
    }

    /**
     * Checks if the take off controller has reached its current objective
     * @param currentInputs the current inputs (this is the base of the check)
     * @param previousInputs the previous inputs of the autopilot
     * @return true if the current height (y-pos) is larger than or equal to the cruising altitude
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        float cruisingAlt = this.getCruisingAltitude();
        float currentAlt = extractAltitude(currentInputs);
        return currentAlt >= cruisingAlt;
    }


    /**
     * Getter for the cruising altitude of the drone, this is the altitude the drone has to reach
     * during takeoff, the cruising altitude is assigned by the autopilot overseer and may be changed
     * during the simulation
     * @return a float containing the altitude to reach (in meters)
     */
    private float getCruisingAltitude() {
        return cruisingAltitude;
    }

    /**
     * Setter for the cruising altitude, the altitude to reach with the drone during takeoff
     * @param cruisingAltitude the altitude to reach (>0)
     */
    public void setCruisingAltitude(float cruisingAltitude) {
        if(!isValidCruisingAltitude(cruisingAltitude))
            throw new IllegalArgumentException("Invalid cruising altitude, assigned altitude was <= 0");
        this.cruisingAltitude = cruisingAltitude;
    }

    /**
     * Checks if the provided cruising altitude is valid
     * @param cruisingAltitude the cruising altitude to check
     * @return true if and only if cruising altitude > 0;
     */
    private static boolean isValidCruisingAltitude(float cruisingAltitude){
        return cruisingAltitude > 0;
    }

    /**
     * Getter for the reference pitch, the pitch to reach as fast as possible during takeoff to reach the cruising
     * altitude (is used as the set-point for the PID controller responsible for the pitch)
     * @return a float containing the reference pitch range[-PI/2, PI/2]
     */
    private float getReferencePitch() {
        return referencePitch;
    }

    /**
     * Setter for the reference pitch used by the pitch control as the set-point
     * @param referencePitch a float within range[-PI/2, PI/2]
     */
    public void setReferencePitch(float referencePitch) {
        if(!isValidReferencePitch(referencePitch)){
            throw new IllegalArgumentException("the reference pitch is not in range [-PI/2, PI/2]");
        }
        this.referencePitch = referencePitch;
    }

    /**
     * Checks if the reference pitch provided is valid
     * @param referencePitch the pitch to check
     * @return true if and only if reference pitch in range[-PI/2, PI/2]
     */
    private boolean isValidReferencePitch(float referencePitch){
        return referencePitch >= -PI/2 && referencePitch <= PI/2;
    }

    /**
     * Getter for the reference velocity, the velocity that must be reached and maintained by the controller
     * during takeoff
     * @return the velocity (in m/s) to be maintained by the drone
     */
    private float getReferenceVelocity() {
        return referenceVelocity;
    }

    /**
     * Setter for the reference velocity of the controller (for more info see getter)
     * @param referenceVelocity the reference velocity of the drone (must be > 0)
     */
    private void setReferenceVelocity(float referenceVelocity) {
        System.out.println("reference velocity: " + referenceVelocity);
        if(!isValidReferenceVelocity(referenceVelocity)){
            throw new IllegalArgumentException("velocity is not strictly positive");
        }
        this.referenceVelocity = referenceVelocity;
    }

    /**
     * Checks if the provided reference velocity is valid
     * @param referenceVelocity the velocity to check
     * @return true if and only if reference velocity > 0;
     */
    private boolean isValidReferenceVelocity(float referenceVelocity){
        return referenceVelocity > 0;
    }

    /**
     * Getter for the pitch pid controller, the controller used to generate the error measurement
     * for controlling the pitch of the drone, use the reference pitch as a set point for this controller
     * @return the PID controller to be used for the pitch controls
     */
    private PIDController getPitchController() {
        return pitchController;
    }

    /**
     * Getter for the thrust PID controller, the controller to generate the error measurement in velocity
     * used by the cruise control implemented in the main controller class
     * @return a PID controller tuned for the cruise control implemented in de controller class
     */
    private PIDController getThrustController() {
        return thrustController;
    }

    /**
     * Getter for the standard outputs of the controller, used to construct a new control outputs objects
     * @return a standard controls outputs used to construct a new ControlOutputs object, the standard
     *         controls contain default values for the outputs of the controller (such that they don't need to be
     *         bothered with when generating the important control values)
     */
    private StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

    /**
     * Getter for the distance the reference point is located from the drone in the heading axis
     * @return a float containing the distance from the origin of the drone to the reference point (virtual ref point)
     */
    public float getPitchRefDistance() {
        return pitchRefDistance;
    }

    /**
     * Getter for the error margin on the aoa control calculations
     * @return the margin (in radians)
     */
    public float getAOAMargin() {
        return AOAMargin;
    }

    /**
     * Setter for the error margin on the angle of attack calculations
     * @param AoaMargin the desired error margin
     */
    public void setAOAMargin(float AoaMargin) {
        this.AOAMargin = AoaMargin;
    }

    /**
     * The PID used to stabilize the pitch after a successful takeoff, the drone stabilizes itself
     * so the next controller may have it easier
     * @return the pid tuned to stabilize the pitch of the drone
     */
    public PIDController getPitchStabilizerPID() {
        return pitchStabilizerPID;
    }

    /**
     * The cruising altitude of the drone used as a milestone to check if we've reached the correct height
     * during the takeoff
     */
    private float cruisingAltitude;

    /**
     * The reference pitch, used during takeoff as a reference to reach for a stable takeoff
     * this pitch should be reached as soon as possible by the autopilot and is used as the error measurement
     * for the PID controller
     */
    private float referencePitch = (float) (15f*PI/180);

    /**
     * The reference velocity, used during the takeoff to cap the maxiumum velocity for the next part of the flight
     * (must be fed into the cruise controller as a reference)
     * upon init the value is set to -1f to indicate that the velocity is not yet configured
     * will be set upon first actual call of the controller
     */
    private float referenceVelocity = -1f;

    /**
     * The margin on the angle of attack control used to account for the imperfect approx of the velocity
     * of the drone
     */
    private float AOAMargin = (float) (2*PI/180);

    /**
     * The distance from the drone to the reference pitch point. This point is used by the takeoff controller
     * to stabilize the flight after initialization
     */
    private float pitchRefDistance = 100f;

    /*
    Constants used to steer the drone
     */

    /**
     * Main wing configuration
     * --> standard main, the usual inclination of the main wings
     * --> max inclination delta, the maximum deviation from the standard inclination of the main wings
     */
    private static float STANDARD_MAIN = (float) (5*PI/180);
    private static float MAX_MAIN_DELTA = (float) (2*PI/180);

    /**
     * Horizontal stabilizer configuration
     * --> standard horizontal, the usual inclination of the stabilizer
     * --> max horizontal, the maximum horizontal stabilizer inclination
     */
    private static float STANDARD_HORIZONTAL = 0f;
    private static float MAX_HORIZONTAL = (float) (8f*PI/180f);

    /**
     * Vertical stabilizer configuration
     * --> standard vertical, the usual vertical inclination
     * --> max vertical, the maximum vertical stabilizer inclination
     */
    private static float STANDARD_VERTICAL = 0f;
    private static float MAX_VERTICAL = 0f;

    /**
     * Configuration for the pitch pid controller, responsible for steering the drone upwards during takeoff
     */
    private final static float PITCH_GAIN = 1.0f;
    private final static float PITCH_INTEGRAL = 0.0f;
    private final static float PITCH_DERIVATIVE = 0.0f;
    private PIDController pitchController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);

    /**
     * Configuration for the thrust pid, used by the cruise control provided by the main controller class
     * (we only need to provide the PID we're using for the thrust)
     */
    private final static float THRUST_GAIN = 1.0f;
    private final static float THRUST_INTEGRAL = 0.0f;
    private final static float THRUST_DERIVATIVE = 0.0f;
    private PIDController thrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);


    /**
     * The tunings for the PID responsible for stabilizing the pitch of the takeoff controller
     * is invoked before the next controller is invoked, once the pitch is stabilized the next controller is invoked
     */
    private final static float STABLE_PITCH_GAIN = 1.0f;
    private final static float STABLE_PITCH_DERIVATIVE = 0.2f;
    private final static float STABLE_PITCH_INTEGRAL = 0.5f;

    private PIDController pitchStabilizerPID = new PIDController(STABLE_PITCH_GAIN, STABLE_PITCH_INTEGRAL,STABLE_PITCH_DERIVATIVE);


    //standard outputs

    /**
     * The standard outputs for this controller, is used to initialize the control outputs of the drone
     */
    private StandardOutputs standardOutputs = new StandardOutputs() {
        @Override
        public float getStandardRightMainInclination() {
            return STANDARD_MAIN;
        }

        @Override
        public float getStandardLeftMainInclination() {
            return STANDARD_MAIN;
        }

        @Override
        public float getStandardHorizontalStabilizerInclination() {
            return STANDARD_HORIZONTAL;
        }

        @Override
        public float getStandardVerticalStabilizerInclination() {
            return STANDARD_VERTICAL;
        }

        @Override
        public float getStandardThrust() {
            return 0;
        }
    };
}

//
//    public AutopilotTakeoffController(AutoPilot autopilot){
//        //implement constructor
//        super(autopilot);
////        this.getVelocityPID().setSetPoint(this.referenceVelocity);
////        this.getOrientationPID().setSetPoint(this.referenceOrientation);
////        this.getAltitudePID().setSetPoint(this.referenceAltitude);
//    }
//
//
//    @Override
//    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
//        Vector position = Controller.extractPosition(inputs);
//        Vector target = getTarget();
//        //check if we've reached the cruising altitude
//        return abs(position.getyValue() - target.getyValue()) < TARGET_DISTANCE_THRESHOLD;
////            System.out.println("altitude reached");
////            System.out.println("Z - position: " + inputs.getZ());
////            System.out.println("Approx velocity: " + this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs()));
////            //load the image in and check if there are any cubes in sight
////            //if not use this controller further
////            AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
////            APCamera.loadNewImage(inputs.getImage());
////            return APCamera.getAllCubeCenters().size() > 0;
//    }
//
//    /**
//     * Controls the thrust of the takeoff controller
//     * @param outputs the outputs for the drone
//     */
//    private void setThrust(ControlOutputs outputs){
//        //get the maximal thrust
//        float maxThrust = this.getConfig().getMaxThrust();
//        float pitch = this.getCurrentInputs().getPitch();
//        float thrustCoeff = 50;
//        float outputThrust = (float) (STANDARD_THRUST + pitch*thrustCoeff*180/PI);
//        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));
//
//    }
//
//
//    /**
//     * Determines the control outputs for the main wings in takeoff
//     * @param outputs the outputs for the drone (are overwritten by this method)
//     */
//    private void setMainWing(ControlOutputs outputs){
//
//        outputs.setLeftWingInclination(MAIN_STABLE);
//        outputs.setRightWingInclination(MAIN_STABLE);
//    }
//
//    /**
//     * Generates the control actions to stabilize the drone and pitch towards the first target of the drone
//     * @param outputs the outputs to write the control actions to
//     */
//    private void stabilizerActions(ControlOutputs outputs){
//        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
//        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
//        Vector orientation = Controller.extractOrientation(currentInputs);
//        float pitchAngle = -this.pitchToTarget(currentInputs);
//        //the pitch should be 0;
//        //System.out.println(pitch);
//        PIDController pitchPid = this.getPitchPID();
//        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
//        float PIDControlActions =  pitchPid.getPIDOutput(pitchAngle,  deltaTime);
//        //System.out.println("Pitch result PID" + PIDControlActions);
//        //adjust the horizontal stabilizer
//        float horizontalInclination = this.getStabilizerStableInclination() - PIDControlActions;
//        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HOR_STABILIZER_MAX);
//        outputs.setHorStabInclination(horizontalInclination);
//    }
//
//    /**
//     * Calculates the angle between the heading vector of the drone (0,0,-1) and the reference vector
//     * (the vector between the current position of the drone and the target)
//     * while indicating the direction to steer in the signum of the angle (positive means upward, negative downward)
//     * @param inputs the inputs to extract the current drone state from
//     * @return a positive angle if upward steering is required, a negative one if downward is necessary
//     */
//    private float pitchToTarget(AutopilotInputs_v2 inputs){
//        //get the target
//        Vector target = this.getTarget();
//        //get the current position
//        Vector dronePos = Controller.extractPosition(inputs);
//        //calculate the diff vector to the target
//        Vector diffVectorWorld = target.vectorDifference(dronePos);
//        //get the orientation of the drone
//        Vector orientation = Controller.extractOrientation(inputs);
//        //transform the world diff vector to the drone axis system
//        Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, orientation);
//        //get the heading vector in the drone axis system
//        Vector heading = new Vector(0,0,-1);
//        //project the difference vector onto the yz-plane of the drone axis system
//        Vector yzNormal = new Vector(1,0,0);
//        Vector projDiffVector = diffVectorDrone.orthogonalProjection(yzNormal);
//        //calculate the angle between the heading and the ref vector
//        float angle = abs(projDiffVector.getAngleBetween(heading));
//        //get the direction by normalizing the cross product a positive cross product means flying up, neg down
//        float direction = signum(heading.crossProduct(projDiffVector).getxValue()); // get the x-value for the direction
//
//        float result = angle*direction;
//        //check for NaN
//        if(Float.isNaN(result)){
//            //default output
//            return 0;
//        }
//        //otherwise return as normal
//        return result;
//
//
//    }
//
//
//
//    /**
//     * Checks if the desired altitude has been reached
//     */
//    private void checkDesiredAltitude(){
//        if(this.getCurrentInputs().getY() >= this.getTarget().getyValue()){
//            this.setReachedDesiredAltitude();
//        }
//    }
//
//
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
////        System.out.println("Altitude: " + (inputs.getY() - this.getConfig().getWheelY() - this.getConfig().getTyreRadius()));
////        System.out.println("z-pos: " + inputs.getZ());
//        this.updateInputs(inputs);
//        ControlOutputs outputs = new ControlOutputs();
//        this.checkDesiredAltitude();
//        //first the simple flight pattern, getting up in the air
//        if(!isAtDesiredAltitude()){
//            simpleTakeoffControls(outputs);
//            pitchControl(outputs);
//        }else{
//            //if we have once reached the desired altitude, go for steady state
//            this.setThrust(outputs);
//            //this.setHorizontalStabilizer(outputs);
//            stabilizerActions(outputs);
//            this.setMainWing(outputs);
//        }
//        //outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
//        rollControl(outputs, inputs);
//        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());
//
//        //System.out.println("outputs takeoffController: " + outputs);
//
//        return outputs;
//    }
//
//    /**
//     * Sets the controls for the first part of the takeoff where the drone needs to get actually of the ground
//     * @param outputs the outputs of the controller for the drone
//     */
//    private void simpleTakeoffControls(ControlOutputs outputs) {
//        float maxThrust = this.getConfig().getMaxThrust();
//        float outputThrust = maxThrust;
//        outputs.setThrust(outputThrust);
//        outputs.setRightWingInclination(MAIN_TAKEOFF);
//        outputs.setLeftWingInclination(MAIN_TAKEOFF);
//        outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
//    }
//
//    private void pitchControl(ControlOutputs outputs){
//        float pitch = Controller.extractOrientation(this.getCurrentInputs()).getyValue();
//        if(abs(pitch) >= PITCH_THRESHOLD){
//            outputs.setHorStabInclination(0f);
//        }
//    }
//
//    @Override
//    protected float getMainStableInclination() {
//        return MAIN_STABLE;
//    }
//
//    @Override
//    protected float getStabilizerStableInclination() {
//        return STABILIZER_STABLE;
//    }
//
//    @Override
//    protected float getRollThreshold() {
//        return ROLL_THRESHOLD;
//    }
//
//    @Override
//    protected float getInclinationAOAErrorMargin() {
//        return INCLINATION_AOA_ERROR_MARGIN;
//    }
//
//    @Override
//    protected float getStandardThrust() {
//        return STANDARD_THRUST;
//    }
//
//    /**
//     * Checks if the desired altitude has been reached
//     * @return true if and only if the desired altitude has been reached
//     */
//    private boolean isAtDesiredAltitude() {
//        return hasReachedDesiredAltitude;
//    }
//
//    /**
//     * Setter for the desired altitude flag
//     */
//    private void setReachedDesiredAltitude() {
//        this.hasReachedDesiredAltitude = true;
//    }
//
//
//    /**
//     * Getter for the reference altitude of the drone
//     * @return the reference altitude as a float
//     */
//    public float getReferenceAltitude() {
//        return referenceAltitude;
//    }
//
//    /**
//     * Setter for the reference altitude, the altitude used for reference by the altitude PID
//     * @param PIDReferenceAltitude the reference altitude
//     */
//    public void setReferenceAltitude(float PIDReferenceAltitude) {
//        //don't forget to set the reference altitude
//        this.referenceAltitude = PIDReferenceAltitude;
//        this.desiredAltitude = PIDReferenceAltitude*0.95f; //set the desired altitude a little lower such that we start
//        //with some pitch upwards
//    }
//
//    /**
//     * Getter for the controller responsible for the pitch of the drone
//     * @return the PID controller
//     */
//    private PIDController getPitchPID() {
//        return pitchPID;
//    }
//
//    private Vector getTarget() {
//        return target;
//    }
//
//    public void setTarget(Vector target) {
//        this.target = target;
//    }
//
//    /**
//     * The stable inclination of the main wings
//     */
//    private final static float MAIN_STABLE = (float) (5*PI/180);
//
//
//    /**
//     * The stable inclination of the stabilizer wings
//     */
//    private final static float STABILIZER_STABLE = 0;
//
//    /**
//     * The takeoff inclination of the main wings
//     */
//    private final static float MAIN_TAKEOFF = (float) (7*PI/180);
//    /**
//     * The takeoff inclination of the stabilizer
//     */
//    private final static float HORIZONTAL_STABILIZER_TAKEOFF = (float) (5*PI/180);
//
////    /**
////     * The maximum inclination of the main wings
////     */
////    private final static float MAIN_MAX = (float) (10*PI/180);
//
//    /**
//     * The maximum inclination of the stabilizer wings
//     */
//    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);
//
//    /**
//     * The threshold for the roll for roll control
//     */
//    private final static float ROLL_THRESHOLD = (float)(3*PI/180);
//
//
//    private final static float PITCH_THRESHOLD = (float)(5*PI/180);
//    /**
//     * The safety margin taken for the AOA
//     */
//    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);
//
//    /**
//     * Standard thrust: the reset thrust for control actions
//     */
//    private final static float STANDARD_THRUST = 550;
//
//    /**
//     * The desired altitude to reach with the drone before starting the altitude control
//     */
//    private float desiredAltitude;
//
//    /**
//     * Flag to check if the desired altitude has been reached
//     */
//    private boolean hasReachedDesiredAltitude = false;
//
//    /**
//     * Reference for the entry of the height (needed for the PID)
//     */
//    private float referenceAltitude;
//
//    /**
//     * A reference for the position to fly to
//     */
//    private Vector target;
////
////    /**
////     * A pid controller for the velocity of the drone
////     */
////    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
////    /**
////     * A pid controller for the orientation of the drone
////     */
////    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);
////
////    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);
//
//    private final static float PITCH_GAIN = 1.0f;
//    private final static float PITCH_DERIVATIVE = 0.2f;
//    private final static float PITCH_INTEGRAL = 0.5f;
//
//    private PIDController pitchPID = new PIDController(PITCH_GAIN,PITCH_INTEGRAL,PITCH_DERIVATIVE);
//
//    /**
//     * the distance to the first target before the controller passes control to the next controller
//     * note: the drone also needs to see a cube to pass the control;
//     */
//    private final static float TARGET_DISTANCE_THRESHOLD = 80f;