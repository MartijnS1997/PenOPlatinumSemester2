package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;
import static java.lang.Math.PI;

/**
 * Created by Martijn on 17/04/2018.
 * This controller is used to stabilize the drone during the transition between different flight phases
 */
public class AutopilotStabilization extends Controller {

    public AutopilotStabilization(AutoPilot autopilot) {
        super(autopilot);
    }

    /**
     * Getter for the controls used to stabilize the drone in between flight states
     * @param currentInputs the outputs currently received by the autopilot from the testbed
     * @param previousInputs the inputs previously received by the autopilot from the testbed
     * @return the control actions needed to stabilize the drone after takeoff
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //create the control outputs
        //Controller.trajectoryLog(currentInputs);
        ControlOutputs outputs = new ControlOutputs(this.getStandardOutputs());
        stabilizePitchControls(outputs, currentInputs, previousInputs);
        angleOfAttackControl(getAoaErrorMargin(), outputs, currentInputs, previousInputs);
        thrustControl(outputs, currentInputs, previousInputs);
        return outputs;
    }

    /**
     * Checks if the objective of the current controller has been reached by achieving the right
     * error margins
     * @param currentInputs the current inputs (this is the base of the check)
     * @param previousInputs the previous inputs (also base to check on)
     * @return true if the pitch and the altitude of the drone are within acceptable margins
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //get the current pitch and the reference altitude
        float pitch = extractPitch(currentInputs);
        float altitude = extractAltitude(currentInputs);
        float cruisingAltitude = this.getCruisingAltitude();
        float referencePitch = 0f;
        return abs(pitch) < referencePitch + abs(this.getPitchErrorMargin())
                && abs(cruisingAltitude - altitude) < abs(this.getAltitudeErrorMargin());
    }

    @Override
    public void reset() {
        this.getPitchStabilizerPID().reset();
        this.getThrustController().reset();
    }

    /**
     * Getter for the controls to control the horizontal stabilizer of the drone
     * @param outputs the outputs to write the result to
     * @param currentInputs the most recent inputs received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void stabilizePitchControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the pitch difference
        float pitchAngle = -this.getRefPitchDiff(currentInputs);
        //the pitch should be 0;
        //System.out.println(pitch);
        PIDController pitchPid = this.getPitchStabilizerPID();
        float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitchAngle,  deltaTime);
        //System.out.println("Pitch result PID" + PIDControlActions);
        //adjust the horizontal stabilizer
        float horizontalInclination = getHorizontalStable() - PIDControlActions;
        horizontalInclination = capInclination(horizontalInclination, getHorizontalStable(), getHorizontalDeltaIncl());
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
        //check if the reference velocity was already assigned or not
        if(referenceVelocity < 0){
            //calculate the reference velocity
            referenceVelocity = this.getAutopilot().getPhysXOptimisations()
                    .calcStableZeroPitchVelocity(getMainStable());
            //and save the calculated value
            this.setReferenceVelocity(referenceVelocity);
        }
        //then get the outputs from the standard cruise control implemented in the main controller class
        this.flightCruiseControl(outputs,currentInputs, previousInputs, thrustControl, referenceVelocity);
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
        Vector pitchRefPoint = this.getPitchReferencePoint(currentInputs);
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
    private Vector getPitchReferencePoint(AutopilotInputs_v2 currentInputs){
        //get the position & orientation of the drone
        Vector position = extractPosition(currentInputs);
        Vector orientation = extractOrientation(currentInputs);
        float cruisingAltitude = this.getCruisingAltitude();
        //get the relative distance for the reference point (located on the negative z-axis in the
        //heading axis system
        float pitchReferenceDistance = this.getDistanceToPitchReference();
        //the location of the reference point in the heading axis system (y-values remain unchanged)
        Vector refPointHeadingAxis = new Vector(0,cruisingAltitude,-pitchReferenceDistance);
        //transform the reference point to the world axis system (relative to drone position)
        Vector refPointWorldAxisRel = PhysXEngine.headingOnWorld(refPointHeadingAxis, orientation);
        //add the x and z position of the drone to the heading vector
        Vector relativePos = new Vector(position.getxValue(), 0 , position.getzValue());
        Vector refPointWorldAxis = refPointWorldAxisRel.vectorSum(relativePos);

        return refPointWorldAxis;
    }

    /**
     * Getter for the cruising altitude assigned to the drone by the overseer
     * @return the cruising altitude of the drone in meters (measured along the y-axis)
     */
    private float getCruisingAltitude() {
        return cruisingAltitude;
    }

    /**
     * Setter for the cruising altitude of the drone (must be larger than zero)
     * @param cruisingAltitude the cruising altitude of the drone
     */
    public void setCruisingAltitude(float cruisingAltitude) {
        if(!isValidCruisingAltitude(cruisingAltitude)){
            throw new IllegalArgumentException("provided cruising altitude is not strictly positive");
        }
        this.cruisingAltitude = cruisingAltitude;
    }

    /**
     * Checks if the given cruising altitude is valid
     * @param cruisingAltitude the altitude to check
     * @return true if and only if the cruisingAltitude > 0
     */
    private static boolean isValidCruisingAltitude(float cruisingAltitude){
        return cruisingAltitude > 0;
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
     * Getter for the distance to the dummy target for the stabilizer controller
     * @return the z-axis (in heading axis system) from the drone to the dummy target
     */
    private float getDistanceToPitchReference() {
        return distanceToPitchReference;
    }

    /**
     * Getter for the error margin on the altitude, used to check if the control may be passed to the next controller
     * @return the error margin on the altitude before going to the next state (in meters)
     */
    private float getAltitudeErrorMargin() {
        return altitudeErrorMargin;
    }

    /**
     * Setter for the error margin on the altitude, used to check if the control may be passed to the next controller
     * @param altitudeErrorMargin the error margin to set (in meters)
     */
    public void setAltitudeErrorMargin(float altitudeErrorMargin) {
        this.altitudeErrorMargin = altitudeErrorMargin;
    }

    /**
     * Getter for the error margin on the pitch, used to check if the control may be passed to the next controller
     * @return the error margin on the pitch in radians
     */
    private float getPitchErrorMargin() {
        return pitchErrorMargin;
    }

    /**
     * Setter for the error margin on the pitch, used to check if the control may be passed to the next controller
     * @param pitchErrorMargin the error margin to set for the controller (in radians)
     */
    public void setPitchErrorMargin(float pitchErrorMargin) {
        this.pitchErrorMargin = pitchErrorMargin;
    }

    /**
     * Getter for the stable inclination of the main wings for the stabilizer controller
     * @return an angle in radians
     */
    private static float getMainStable() {
        return MAIN_STABLE;
    }

    /**
     * Getter for the inclination delta, the maximum deviation of the main wing inclination
     * @return the maximum deviation in main wing inclination in radians
     */
    private static float getMainDeltaIncl() {
        return MAIN_DELTA_INCL;
    }

    /**
     * Getter for the horizontal stabilizer stable inclination, the inclination of the horizontal stabilizer
     * that is used as a reference when determining the total inclination
     * @return the stable inclination in radians
     */
    private static float getHorizontalStable() {
        return HORIZONTAL_STABLE;
    }

    /**
     * Getter for the maximum deviation in horizontal inclination (analogous to the main delta inclination)
     * @return the maximum deviation in radians
     */
    private static float getHorizontalDeltaIncl() {
        return HORIZONTAL_DELTA_INCL;
    }

    /**
     * Getter for the pitch PID the PID used to stabilize the pitch
     * @return a pid controller tuned for the pitch
     */
    private PIDController getPitchStabilizerPID() {
        return pitchStabilizerPID;
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
     * Getter for the standard outputs of the stabilizer controller, used by the stabilizer controller to generate
     * a new control outputs object
     * @return a standard outputs object containing the outputs standard for this specific controller (default values)
     */
    private StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

    /**
     * Getter for the error margin on the AOA calculations used a safeguard against the errors in the
     * velocity approximation
     * @return the error margin on the AOA calculations in radians
     */
    private float getAoaErrorMargin() {
        return aoaErrorMargin;
    }

    /**
     * Setter for the aoa error margin, used to account for the errors in the velocity approximation
     * @param aoaErrorMargin the error margin to be used during the calculations
     */
    public void setAoaErrorMargin(float aoaErrorMargin) {
        this.aoaErrorMargin = aoaErrorMargin;
    }

    /**
     * The cruising altitude currently assigned to the drone by the autopilot overseer, this is the set-point for which
     * the pitch will be adjusted
     */
    private float cruisingAltitude;

    /**
     * The z-distance (in the heading axis system) to reference point used to control the pitch, this point is located
     * in front of the drone. If the heading axis system (since we only will control the pitch not the heading)
     * --> used as a dummy for a target to stabilize the flight
     */
    private float distanceToPitchReference = 100f;

    /**
     * Getter for the error margin on the altitude before the next controller may start
     */
    private float altitudeErrorMargin = 0.5f;

    /**
     * Getter for the error margin on the pitch before the next controller may start
     */
    private float pitchErrorMargin = (float) (1*PI/180f);

    /**
     * The error margin employed by the AOA control calculations to account for the
     * imperfect information about the velocity of the drone
     */
    private float aoaErrorMargin = (float) (2*PI/180);

    /**
     * The reference velocity used to steer the cruise control with
     * is set to -1 upon initialization if the controller is called for the first time
     * the correct value is calculated
     */
    private float referenceVelocity = -1f;


    /*
    Configuration of the steering angles of the controller
     */

    //the main wings
    /**
     * Main wing configuration, we cap the main wing inclinations to prevent excessive control actions
     * --> Main stable: the stable inclination of the main wings, used as a reference
     * --> Main delta incl: de maximum inclination deviation from the stable value, the range of possible main wing
     *                      inclinations= range[MAIN_STABLE - MAIN_DELTA_INCL, MAIN_STABLE + MAIN_DELTA_INCL]
     */
    private final static float MAIN_STABLE = (float) (5*PI/180);
    private final static float MAIN_DELTA_INCL = (float) (2*PI/180);

    /**
     * Configuration for the horizontal stabilizer, we cap the horizontal stabilizer inclinations to prevent
     * excessive control actions
     * --> horizontal stable: the stable inclination of the horizontal stabilizer, used for reference
     * --> horizontal delta: the maximum stabilizer inclination deviation, the range of possible inclinations
     *                       [HORIZONTAL_STABLE -HOR_MAX, HORIZONTAL_STABLE + HOR_MAX]
     */
    private final static float HORIZONTAL_STABLE = 0f;
    private final static float HORIZONTAL_DELTA_INCL = (float) (8*PI/180);

    /**
     * Configuration of the vertical stabilizer, these wings should stay fixed during the flight
     */
    private final static float VERTICAL_STABLE = 0f;
    private final static float VERTICAL_MAX = 0f;

    /**
     * The tunings for the PID responsible for stabilizing the pitch
     */
    private final static float STABLE_PITCH_GAIN = 1.4f;
    private final static float STABLE_PITCH_INTEGRAL = 0.0f;
    private final static float STABLE_PITCH_DERIVATIVE = 0.7f;


    private PIDController pitchStabilizerPID = new PIDController(STABLE_PITCH_GAIN, STABLE_PITCH_INTEGRAL,STABLE_PITCH_DERIVATIVE);

    /**
     * Configuration for the thrust pid, used by the cruise control provided by the main controller class
     * (we only need to provide the PID we're using for the thrust)
     */
    private final static float THRUST_GAIN = 1.0f;
    private final static float THRUST_INTEGRAL = 0.0f;
    private final static float THRUST_DERIVATIVE = 0.0f;
    private PIDController thrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

    /**
     * The standard outputs for this controller, is used to initialize the control outputs of the drone
     */
    private StandardOutputs standardOutputs = new StandardOutputs() {
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
            return HORIZONTAL_STABLE;
        }

        @Override
        public float getStandardVerticalStabilizerInclination() {
            return VERTICAL_STABLE;
        }

        @Override
        public float getStandardThrust() {
            return 0;
        }
    };

}
