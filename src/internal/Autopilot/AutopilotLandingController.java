
package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import TestbedAutopilotInterface.Overseer.MapAirport;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;


import static internal.Autopilot.AutopilotLandingController.LandingPhases.BRAKE;
import static internal.Autopilot.AutopilotLandingController.LandingPhases.RAPID_DESCEND;
import static internal.Autopilot.AutopilotLandingController.LandingPhases.SOFT_DESCEND;
import static java.lang.Math.*;



/**
 * Created by Martijn on 18/02/2018, extended by Jonathan on 12/3/2018
 * A class of landing controllers, responsible for controlling the landing of the drone
 */

public class AutopilotLandingController extends Controller {

    public AutopilotLandingController(AutoPilot autopilot) {
        // implement constructor
        super(autopilot);
        setDescendController(new DescendController(autopilot));
        setSoftDescendController(new SoftDescendController(autopilot));
        setBrakeController(new BrakeController(autopilot));

    }

    @Override
    public void reset() {
        this.getPitchPIDController().reset();
        this.setTargetAirport(null);
        this.setState(RAPID_DESCEND);
    }

    /**
     * Returns true if the plane came to a standstill on the ground
     * @param currentInputs the current inputs (this is the base of the check)
     * @param previousInputs
     * @return true if the approximate velocity is below velocity threshold
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        float absVel = getVelocityApprox(currentInputs, previousInputs).getSize();
        return absVel < 1.0;
    }


    /**
     * Generates the control actions for the autopilot
     *
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs, AutopilotInputs_v2 prevInputs) {
        LandingPhases landingPhase = toNextState(inputs, prevInputs);
        LandingPhaseController controller = getStateController(landingPhase);
        AutopilotOutputs outputs = controller.getControlActions(inputs, prevInputs);
        return outputs;
    }



    /**
     * Getter for the state that will be active during the next iteration (not the logically next state)
     * @param currentInputs the inputs used to determine the next state
     * @return the next active state of the autopilot
     */
    private LandingPhases getNextActiveState(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        //get the current state
        LandingPhases state = getState();
        //determine which controller is next
        switch(state){
            case RAPID_DESCEND:
                //check if we've already passed a single iteration (needed to configure the autopilot)
                return descendController.hasReachedGoal(currentInputs,prevInputs) ? SOFT_DESCEND : RAPID_DESCEND;
            case SOFT_DESCEND:
                //get the takeoff controller
                //check if it has reached its Goal, if so return the next state, if not continue the takeoff
                return softDescendController.hasReachedGoal(currentInputs, prevInputs ) ? BRAKE : SOFT_DESCEND;
            case BRAKE:
                //get the takeoff stabilization controller
                return brakeController.hasReachedGoal(currentInputs, prevInputs) ? BRAKE : LandingPhases.DONE;

            default:
                //Default action, may change later (determine based on the inputs which state should be appropriate
                return BRAKE;
        }
    }

    /**
     * Advances the finite state machine to the next state (may be the same one as before)
     * while setting the right parameters for each controller
     * @param inputs the inputs to base the configuration on
     * @return the state that needs to be active to generate the next outputs
     * note: may change the configuration of the controllers during the method call (eg targets may be changed)
     * note: usecase --> should be called every iteration to get the next state
     */
    private LandingPhases toNextState(AutopilotInputs_v2 inputs, AutopilotInputs_v2 prevInputs){
        //get the next state needed by the controller
        LandingPhases nextState = this.getNextActiveState(inputs,prevInputs);
        //get the previous state
        LandingPhases prevState = this.getState();

        //check if they are the same, if not, configure the controller & save the next state
        if(!nextState.equals(prevState)){
            this.setState(nextState);
        }

        //now return the next state
        return nextState;
    }

    private LandingPhases landingPhase = RAPID_DESCEND;

    private LandingPhases getState() {
        return landingPhase;
    }
    private void setState(LandingPhases landingPhase){
        this.landingPhase = landingPhase;
    }
    /**
     * Get the controller responsible for the provided state
     * @param state the state to get the controller for
     * @return the controller assiciated with the state
     */
    private LandingPhaseController getStateController(LandingPhases state){
        switch(state){
            case RAPID_DESCEND:
                return getDescendController();
            case SOFT_DESCEND:
                return getSoftDescendController();
            case BRAKE:
                return getBrakeController();
            default:
                return getBrakeController();
            }
    }

    /**
     * Stabilizes the pitch of the drone
     * @param outputs the outputs to write to
     * @param currentInputs the current inputs to extract the flight data from
     * @param prevInputs the previous inputs to extract the flight data from
     */
    private void pitchStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
        //stabilize the pitch
        //extract the current orientation
        Vector orientation = Controller.extractOrientation(currentInputs);
        float pitch = orientation.getyValue();
        PIDController pitchPid =this.getPitchPIDController();
        float deltaTime = Controller.getDeltaTime(currentInputs, prevInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitch, deltaTime);
        //adjust the horizontal stabilizer
        float horizontalInclination = AutopilotLandingController.getStabilizerStable() - PIDControlActions;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HOR_STABILIZER_MAX);
        outputs.setHorStabInclination(horizontalInclination);
    }

    /**
     * Stabilizer for the roll of the drone
     * @param outputs the outputs to write to
     * @param currentInputs the current inputs to extract the flight data from
     * @param prevInputs the previous inputs to extract the flight data from
     */
    private void rollStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        Vector orientation = Controller.extractOrientation(currentInputs);
        float roll = orientation.getzValue();
        PIDController rollPid = this.getRollPIDController();
        float deltaTime = Controller.getDeltaTime(currentInputs, prevInputs);
        float PIDControlActions = rollPid.getPIDOutput(roll, deltaTime);
        float rightMainWing = AutopilotLandingController.getMainStable() + PIDControlActions;
        float leftMainWing = AutopilotLandingController.getMainStable() - PIDControlActions;

        outputs.setRightWingInclination(capMainWingInclination(rightMainWing));
        outputs.setLeftWingInclination(capMainWingInclination(leftMainWing));

    }

    /**
     * Enforces a maximum onto the main wing inclination based on the MAIN_CAP_DELTA_INCLINATION variable
     * @param inclination the inclination to adjust
     * @return if the inclination is in the range getMainStable +- MAIN_CAP_DELTA_INCLINATION
     *         the same value as the inclination is returned, if it exceeds the border value, the inclination
     *         is set to the border value
     */
    private float capMainWingInclination(float inclination){
        //first determine the lower cap:
        float lowerCap = getMainStable() - MAIN_CAP_DELTA_INCLINATION;
        float upperCap = getMainStable() + MAIN_CAP_DELTA_INCLINATION;
        if(inclination < lowerCap){
            return lowerCap;
        }
        if(inclination > upperCap){
            return upperCap;
        }
        return inclination;
    }

    /**
     * Getter for the target airport this is the airport where the drone has to land
     * @return the airport
     */
    private MapAirport getTargetAirport() {
        return targetAirport;
    }

    /**
     * Setter for the target airport, this is the airport where the drone should land
     * this airport is used for generating the reference points
     * @param targetAirport the target airport for the controller
     */
    public void setTargetAirport(MapAirport targetAirport) {
        this.targetAirport = targetAirport;
    }

    /**
     * The airport where the drone has to land
     */
    private MapAirport targetAirport;


    /**
     * Calculates the error in the heading of the drone against the given reference point
     * the error contains the magnitude of the error as well as the direction of the error
     * positive angle means the drone needs to be steered to the left
     * negative angle means the drone needs to be steered to the right
     * @param currentInputs the inputs most recently received from the testbed
     * @param referencePoint the reference point used by the controller for calculating the heading error for
     * @return the error angle between the current heading vector and the vector pointing from the drone to the
     *         reference point ( =difference vector). The angle is positive if the difference vector is to the
     *         left of the heading vector and negative if to the right.
     */
    private static float getHeadingError(AutopilotInputs_v2 currentInputs, Vector referencePoint){
        //grab the parameters & other stuff to calculate the heading error for the controller
        Vector headingDroneHa = new Vector(0,0,-1 ); //the heading vector in the heading axis
        Vector xzNormal = new Vector(0,1,0);
        Vector dronePosition = extractGroundPosition(currentInputs);
        Vector droneOrientation = extractOrientation(currentInputs);

        //transform the heading vector onto the world
        Vector headingWorld = PhysXEngine.headingOnWorld(headingDroneHa, droneOrientation);

        //get the difference vector between the drone and the set point (points from drone to set point)
        Vector diffVector = referencePoint.vectorDifference(dronePosition);
        //project both of them on the xz plane
        Vector headingWorldXZ = headingWorld.orthogonalProjection(xzNormal);
        Vector diffVectorXZ = diffVector.orthogonalProjection(xzNormal);

        //calculate the angle between the heading of the world and the difference vector
        float headingAngle = headingWorldXZ.getAngleBetween(diffVectorXZ);

        //now we also need to get the direction of the angle (vector product between heading and the difference)
        //headingWorld.crossProduct(diffVector) is positive if diff is to the left of the heading
        //and negative if the difference vector is to the right of the heading vector (the sign of the resulting y component)
        float direction = signum(headingWorldXZ.crossProduct(diffVectorXZ).getyValue());

        //the resulting error
        float headingError = direction*abs(headingAngle);

        //errorLog(headingError);
        //for some reason the result can be NaN, better to avert controller spouting errors
        //just return some default value that will cause no harm(?)
        return Float.isNaN(headingError) ? 0 : headingError;
    }

    /**
     * Writes rhe main wing inclination to the outputs specified in the arguments, the provided wing inclinations
     * are capped with the getMainDeltaIncl as specified in the capInclination method in the super class
     * @param outputs the outputs to write the main wing inclinations to
     * @param mainLeftWingInclination the left main wing inclination to set (and to cap)
     * @param mainRightWingInclination the right main wing inclination to set (and to cap)
     */
    private void setMainWingInclinations(ControlOutputs outputs, float mainLeftWingInclination, float mainRightWingInclination) {
        mainLeftWingInclination = capInclination(mainLeftWingInclination, getMainStable(), getMainDeltaIncl());
        mainRightWingInclination = capInclination(mainRightWingInclination, getMainStable(), getMainDeltaIncl());

        outputs.setLeftWingInclination(mainLeftWingInclination);
        outputs.setRightWingInclination(mainRightWingInclination);
    }


    /**
     * Constants
     */
    private final static float STOP_VELOCITY = 5.0f;

    private final static float MAIN_STABLE = (float) (7*PI/180);
    private final static float MAIN_DELTA_INCL = (float) (2*PI/180);
    private final static float STABILIZER_STABLE = 0;
    private final static float HOR_STABILIZER_MAX = (float)(7*PI/180);
    private final static float MAIN_CAP_DELTA_INCLINATION = (float) (3*PI/180);




    //the roll PID controller and its constants, used in all phases of the flight
    private static float ROLL_GAIN = 1;
    private static float ROLL_INTEGRAL = 0.2f;
    private static float ROLL_DERIVATIVE = 0;
    private PIDController rollPIDController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    //soft descend constants
    private final static float SOFT_DESCEND_PHASE_REF_PITCH = (float) (-5*PI/180);
    private final static float SOFT_DESCEND_THRUST = 0f;
    private final static float SOFT_DESCEND_MIN_START_HEIGHT = 20f;
    private final static float SOFT_DESCEND_STOP_HEIGHT = 2f;



    public PIDController getRollPIDController(){
        return rollPIDController;
    }



    //TODO implement these methods accordingly

    protected static float getMainStable() {
        return MAIN_STABLE;
    }

    protected static float getStabilizerStable() {
        return STABILIZER_STABLE;
    }

    protected static float getMainDeltaIncl(){
        return MAIN_DELTA_INCL;
    }

    protected static float getHorizontalDeltaIncl(){
        return HOR_STABILIZER_MAX;
    }

    protected static float getAoaResultMargin(){ return AOA_RESULT_MARGIN;}
    /**
     * Getter for the standard outputs of the controller
     * @return the standard outputs
     */
    public StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }



    StandardOutputs standardOutputs = new StandardOutputs() {
        @Override
        public float getStandardRightMainInclination() {
            return MAIN_STABLE;
        }

        @Override
        public float getStandardLeftMainInclination() {
            return MAIN_STABLE;
        }

        @Override
        public float getStandardHorizontalStabilizerInclination() {
            return STABILIZER_STABLE;
        }

        @Override
        public float getStandardVerticalStabilizerInclination() {
            return 0;
        }

        @Override
        public float getStandardThrust() {
            return 0;
        }
    };

    /**
     * Safeguard for rounding and approximation errors for the AOA control
     */
    private final static float AOA_RESULT_MARGIN = (float) (2*PI/180);

    /**
     * Enumerations for the landing phases
     */
    enum LandingPhases {
        RAPID_DESCEND, SOFT_DESCEND, BRAKE, DONE}


    private void setDescendController(DescendController descendController){
        this.descendController = descendController;
    }
    private void setSoftDescendController(SoftDescendController softDescendController){
        this.softDescendController = softDescendController;
    }

    private void setBrakeController(BrakeController brakeController){
        this.brakeController = brakeController;
    }

    private DescendController getDescendController(){
        return descendController;
    }

    private SoftDescendController getSoftDescendController(){
        return softDescendController;
    }

    private BrakeController getBrakeController(){
        return brakeController;
    }


    private DescendController descendController;
    private SoftDescendController softDescendController;
    private BrakeController brakeController;


    private interface LandingPhaseController {

         boolean hasReachedGoal(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs);

         AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs);

        /**
         * Clears all the state from the controller
         */
        void reset();

    }
    private class DescendController implements LandingPhaseController {

        public DescendController(AutoPilot autopilot) /*extends Controller*/ {
            //super(autopilot);
        }

        @Override
        public void reset() {
            this.getHeadingPID().reset();
            this.getPitchPID().reset();
            this.getRollPID().reset();
        }

        public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            ControlOutputs outputs = new ControlOutputs(getStandardOutputs());
            getHeadingControls(outputs,currentInputs,previousInputs);
            getRollControls(outputs,currentInputs,previousInputs);
            getPitchControls(outputs,currentInputs,previousInputs);
            angleOfAttackControl((float) (2*PI/180),outputs,currentInputs,previousInputs);
            return outputs;
        }


        public boolean hasReachedGoal(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            float altitude = Controller.extractAltitude(currentInputs);
            return(altitude < SOFT_DESCEND_MIN_START_HEIGHT);
        }

        /**
         * Getter for the controls for the roll of the drone, this controller steers against the heading controller
         * to keep the over-steering in check, should be invoked after the call to the heading contoller
         * @param outputs the outputs to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void getRollControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the current roll (the error) & the other variables needed to calculate the control actions
            float roll = extractRoll(currentInputs);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            if(abs(roll) < 25*PI/180){
                return;
            }

            //get the PID controller used to steer for the roll
            PIDController rollPid = this.getRollPID();
            //get the outputs based on the error (the roll itself is the error on the wanted situation)
            float pidResult = rollPid.getPIDOutput(roll, deltaTime);
            //if we need to roll to the left the roll is negative & the error generated will be positive
            //thus we need to lift the right main wing a larger inclination than the left one if we want to go to the left
            //the converse for a negative error we need to steer to the right (left wing larger than right)
            float leftWingInclination = getMainStable() - pidResult;
            float rightWingInclination = getMainStable() + pidResult;

            //cap the results
            leftWingInclination = capInclination(outputs.getLeftWingInclination() + leftWingInclination, getMainStable(), getMainDeltaIncl());
            rightWingInclination = capInclination(outputs.getRightWingInclination() + rightWingInclination, getMainStable(), getMainDeltaIncl());

            //save the results
            outputs.setLeftWingInclination(leftWingInclination);
            outputs.setRightWingInclination(rightWingInclination);
        }

        /**
         * Getter for the controls for the heading during the rapid descend
         * @param outputs the outputs to write the results to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void getHeadingControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the target for the heading error
            MapAirport targetAirport = AutopilotLandingController.this.getTargetAirport();
            Vector referencePoint = targetAirport.getLocation();
            //get the error on the heading and the delta time
            float headingError = AutopilotLandingController.getHeadingError(currentInputs, referencePoint);
            //headingError = abs(headingError) < 10E-3 ? 0 : headingError; //to counter oscillating behavior
            float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
            //get the heading PID
            PIDController bankControl = this.getHeadingPID();
            float pidOutput = bankControl.getPIDOutput(headingError, deltaTime);
            //and inclining the main wings
            float mainLeftWingInclination = getMainStable() + pidOutput;
            float mainRightWingInclination = getMainStable() - pidOutput;
            //errorLog((float) (headingError*180/PI));

            //get the cap on the inclinations (prevent over steering)
            setMainWingInclinations(outputs, mainLeftWingInclination, mainRightWingInclination);

        }


        /**
         * Getter for the control actions for the pitch of the drone during the descend phase
         * the controls that are generated are written to the outputs provided
         * @param outputs the outputs to write the results to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void getPitchControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){

            Vector referencePoint = getPitchReference(currentInputs);
            float pitchError = this.getPitchError(currentInputs, referencePoint);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            //feed the results into the pid controller
            PIDController pitchPid = this.getPitchPID();
            //errorLog(pitchError);
            float pidResult = pitchPid.getPIDOutput(pitchError, deltaTime);
            //if the pid result is positive: the set point is larger than the input -> lower the pitch (pos inclination)
            //if the pid result is negative: the set point is smaller than the input -> increase the pitch (neg inclination)
            float horizontalInclination = getStabilizerStable() + pidResult;
            horizontalInclination = capInclination(horizontalInclination, getStabilizerStable(), getHorizontalDeltaIncl());
            outputs.setHorStabInclination(horizontalInclination);
        }

        /**
         * Calculates the reference point for the pitch controller
         * this is the end point of the runway closest to the drone
         * @param currentInputs the inputs most recently received from the testbed
         * @return A vector containing the reference point
         */
        private Vector getPitchReference(AutopilotInputs_v2 currentInputs){
            MapAirport airport = AutopilotLandingController.this.getTargetAirport();
            Vector dronePos = Controller.extractPosition(currentInputs);
            //get the entry points for the landing strips
            Vector runwayZeroEntry = airport.getRunwayZeroEnd();
            Vector runwayOneEntry =  airport.getRunwayOneEnd();

            return dronePos.distanceBetween(runwayZeroEntry) < dronePos.distanceBetween(runwayOneEntry)
                    ? runwayZeroEntry : runwayOneEntry;
        }

        /**
         * Calculates the error on the pitch of the drone against the reference point of the flight
         * the pitch error is the angle between the heading vector (zero roll) in the pitch axis system
         * and the vector pointing from the drone to the reference point ( =difference vector) projected onto
         * the yz-plane of the pitch axis system
         * @param currentInputs the inputs most recently received from the testbed
         * @param referencePoint the reference point to calculate the pitch error for
         * @return the angle as described above with the sign indicating the direction of the error
         *         if the pitch error is positive, extra pitch is required (steering upwards)
         *         if the pitch error is negative, lesser pitch is required (steering downwards)
         */
        private float getPitchError(AutopilotInputs_v2 currentInputs, Vector referencePoint){
            //grab the parameters needed fo the calculation
            Vector droneHeadingPa = new Vector(0,0,-1); //the heading vector of the drone in pitch axis
            Vector yzNormalPa = new Vector(1,0,0); // normal of the yz-plane in pitch axis
            Vector dronePosition = extractPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);

            //calculate the difference vector and transform it to the pitch axis system
            //difference vector points from the drone position to the reference point
            Vector diffVector =  referencePoint.vectorDifference(dronePosition);
            Vector diffPa = PhysXEngine.worldOnPitch(diffVector, droneOrientation);

            //project the diff vector in pitch axis system onto the yz plane of the pitch axis
            Vector projDiffPa = diffPa.orthogonalProjection(yzNormalPa);

            //calculate the angle between the heading vector and the projected diff vector
            float pitchErrorAngle = projDiffPa.getAngleBetween(droneHeadingPa);

            //also calculate the direction of the error, take the vector product of the heading and the diff vector
            //if the x-component is positive the drone has to pitch upwards, if it is negative the drone has
            //to pitch downwards to reach the desired altitude/pitch
            float direction = signum(droneHeadingPa.crossProduct(projDiffPa).getxValue());

            //multiply to get the error
            float pitchError = direction*abs(pitchErrorAngle);

            //return the result while checking for NaN, if so return harmless (?) value
            return Float.isNaN(pitchError) ? 0 : pitchError;

        }


        //TODO document
        private PIDController getRollPID() {
            return rollPID;
        }

        private PIDController getHeadingPID() {
            return headingPID;
        }

        private PIDController getPitchPID() {
            return pitchPID;
        }


        /**
         * Constants for the roll controller and the pid controller itself
         */
        private final static float ROLL_GAIN = 1.0f;
        private final static float ROLL_INTEGRAL = 1.0f;
        private final static float ROLL_DERIVATIVE =1.0f;
        private final PIDController rollPID = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

        /**
         * Constants for the heading controller and the pid controller itself
         */
        private final static float HEADING_GAIN = 0.8f;
        private final static float HEADING_INTEGRAL = 0.0f;
        private final static float HEADING_DERIVATIVE = 1.0f ;
        private final PIDController headingPID = new PIDController(HEADING_GAIN, HEADING_INTEGRAL, HEADING_DERIVATIVE);

        /**
         * Constants for the pitch controller and the pid controller itself
         */
        private final static float PITCH_GAIN = 1f;
        private final static float PITCH_INTEGRAL  = 4.0f;
        private final static float PITCH_DERIVATIVE = 0.1f;
        private final PIDController pitchPID = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);


    }

    private class SoftDescendController implements LandingPhaseController {

        public SoftDescendController(AutoPilot autopilot) {
            //super(autopilot);
        }


        @Override
        public boolean hasReachedGoal(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
            float altitude = Controller.extractAltitude(currentInputs);
            return(altitude < SOFT_DESCEND_STOP_HEIGHT);
        }

        @Override
        public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
            ControlOutputs outputs = new ControlOutputs(getStandardOutputs());
            getDescendControls(outputs, currentInputs, prevInputs);
            toTargetControls(outputs, currentInputs, prevInputs);
            angleOfAttackControl(getAoaResultMargin(), outputs, prevInputs, currentInputs);

            return outputs;
        }


        @Override
        public void reset() {
            this.getHeadingPID().reset();
            this.getRollPID().reset();
        }

        /**
         * Getter for the pitch controls of the drone, these should steer the drone during the soft descending phase of
         * the landing
         * @param outputs the outputs to write the result to
         * @param currentInputs the inputs most recently received from the testbed
         * @param prevInputs the inputs previously received from the testbed
         */
        private void getDescendControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
            AutopilotConfig config = getConfig();
            PIDController pitchPID = getPitchPIDController();
            pitchPID.setSetPoint(SOFT_DESCEND_PHASE_REF_PITCH);
            pitchStabilizer(outputs, currentInputs, prevInputs);
            rollStabilizer(outputs, currentInputs, prevInputs);
            outputs.setThrust(SOFT_DESCEND_THRUST);
            outputs.setRightBrakeForce(config.getRMax());
            outputs.setLeftBrakeForce(config.getRMax());
            outputs.setFrontBrakeForce(config.getRMax());
        }

        /**
         * Control actions needed to keep the drone on track for the landing
         * @param outputs the outputs to write the result to
         * @param currentInputs current inputs
         * @param previousInputs previous inputs
         */
        private void toTargetControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
           this.getHeadingControls(outputs, currentInputs, previousInputs);
           this.getRollControls(outputs, currentInputs, previousInputs);
        }

        /**
         * Getter for the heading controls of the soft landing phase, these are meant to keep the drone on track during
         * the landing
         * @param outputs the outputs to write the control outputs to
         * @param currentInputs current inputs
         * @param previousInputs previous inputs
         */
        private void getHeadingControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the target for the heading error
            MapAirport targetAirport = AutopilotLandingController.this.getTargetAirport();
            Vector referencePoint = targetAirport.getLocation();
            //get the error on the heading and the delta time
            float headingError = AutopilotLandingController.getHeadingError(currentInputs, referencePoint);
            //headingError = abs(headingError) < 10E-3 ? 0 : headingError; //to counter oscillating behavior
            float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
            //get the heading PID
            PIDController bankControl = this.getHeadingPID();
            float pidOutput = bankControl.getPIDOutput(headingError, deltaTime);
            //and inclining the main wings
            float mainLeftWingInclination = getMainStable() + pidOutput;
            float mainRightWingInclination = getMainStable() - pidOutput;
            //errorLog((float) (headingError*180/PI));

            //get the cap on the inclinations (prevent over steering)
            setMainWingInclinations(outputs, mainLeftWingInclination, mainRightWingInclination);
        }

        /**
         * Getter for the roll controls used to keep the oscillations of the heading controls in check
         * this controller should be invoked after the heading controls
         * @param outputs the outputs to write the result to
         * @param currentInputs the current inputs of the drone
         * @param previousInputs the inputs previously received
         */
        private void getRollControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the current roll (the error) & the other variables needed to calculate the control actions
            float roll = extractRoll(currentInputs);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            if(abs(roll) < 1f*PI/180){
                return;
            }

            //get the PID controller used to steer for the roll
            PIDController rollPid = this.getRollPID();
            //get the outputs based on the error (the roll itself is the error on the wanted situation)
            float pidResult = rollPid.getPIDOutput(roll, deltaTime);
            //if we need to roll to the left the roll is negative & the error generated will be positive
            //thus we need to lift the right main wing a larger inclination than the left one if we want to go to the left
            //the converse for a negative error we need to steer to the right (left wing larger than right)
            float leftWingInclination = getMainStable() - pidResult;
            float rightWingInclination = getMainStable() + pidResult;

            //cap the results
            leftWingInclination = capInclination(outputs.getLeftWingInclination() + leftWingInclination, getMainStable(), getMainDeltaIncl());
            rightWingInclination = capInclination(outputs.getRightWingInclination() + rightWingInclination, getMainStable(), getMainDeltaIncl());

            //save the results
            outputs.setLeftWingInclination(leftWingInclination);
            outputs.setRightWingInclination(rightWingInclination);
        }

        private PIDController getHeadingPID() {
            return headingPID;
        }

        private PIDController getRollPID() {
            return rollPID;
        }

        private final static float HEADING_GAIN = 0.8f;
        private final static float HEADING_INTEGRAL = 0.0f;
        private final static float HEADING_DERIVATIVE = 1.0f ;
        private final PIDController headingPID = new PIDController(HEADING_GAIN, HEADING_INTEGRAL, HEADING_DERIVATIVE);

        /**
         * Constants for the roll controller and the pid controller itself
         */
        private final static float ROLL_GAIN = 1.0f;
        private final static float ROLL_INTEGRAL = 1.0f;
        private final static float ROLL_DERIVATIVE =1.0f;
        private final PIDController rollPID = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    }

    private class BrakeController implements LandingPhaseController {

        public BrakeController(AutoPilot autopilot) {
            //super(autopilot);
        }

        @Override
        public void reset() {
            //nothing to reset
        }

        @Override
        public boolean hasReachedGoal(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
            Vector velocity = Controller.getVelocityApprox(currentInputs, prevInputs);
            return (velocity.getSize() < STOP_VELOCITY) ;//TODO which Velocity
        }

        @Override
        public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
            ControlOutputs outputs = new ControlOutputs(getStandardOutputs());
            AutopilotConfig config = getConfig();
            rollStabilizer(outputs, currentInputs, prevInputs);
            PIDController pitchPID = getPitchPIDController();
            pitchPID.setSetPoint(SOFT_DESCEND_PHASE_REF_PITCH/2);
            outputs.setThrust(SOFT_DESCEND_THRUST);
            outputs.setRightBrakeForce(config.getRMax());
            outputs.setLeftBrakeForce(config.getRMax());
            outputs.setFrontBrakeForce(config.getRMax());
            return outputs;
        }
    }


    private PIDController getPitchPIDController() {
        return pitchPIDController;
    }

    //the pitch controller used in all stages of the flight but with different set points
    private static float PITCH_GAIN = 1;
    private static float PITCH_INTEGRAL = 0.2f;
    private static float PITCH_DERIVATIVE = 0.0f;
    private PIDController pitchPIDController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);
}



