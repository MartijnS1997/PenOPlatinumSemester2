
package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;


import static java.lang.Math.*;


/**
 * Created by Martijn on 18/02/2018, extended by Jonathan on 12/3/2018
 * A class of landing controllers, responsible for controlling the landing of the drone
 */

public class AutopilotLandingController extends Controller {

    public AutopilotLandingController(AutoPilot autopilot) {
        // implement constructor
        super(autopilot);
        this.getVelocityPID().setSetPoint(this.referenceVelocity);
        this.getOrientationPID().setSetPoint(this.referenceOrientation);
        this.getAltitudePID().setSetPoint(this.referenceAltitude);

    }

    /**
     * Returns true if the plane came to a standstill on the ground
     * @param inputs the current inputs (this is the base of the check)
     * @return true if the approximate velocity is below velocity threshold
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        Vector velocityApprox = this.getVelocityApprox(this.getCurrentInputs(), inputs);
        return velocityApprox.getSize() <= MAXIMUM_LANDING_VELOCITY;
    }

    /**
     * Generates the control actions for the autopilot
     *
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        this.setCurrentInputs(inputs);

        if(this.isFirstControlCall()){
            this.setStartElapsedTime(this.getCurrentInputs());
            this.setFirstControlCall();
        }

        // generate path
/*        AutopilotInputs currentInputs = getCurrentInputs();
        AutopilotInputs previousInputs = getPreviousInputs();*/
        ControlOutputs outputs = new ControlOutputs();

        //TODO check if stable flight
        if(!this.isStartedDescend()){
            //check if the flight is stable
            if(!mayInitializeLanding(inputs)){

                this.stabilizeFlight(outputs);
                return outputs;
            }
            System.out.println("Drone is stabilized");
            this.setStartedDescend();
        }else {

/*        Vector position = new Vector(currentInputs.getX(), currentInputs.getY(), currentInputs.getZ());
        Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
        Vector destination = this.getAutopilot().getStartPosition();*/

            //start going down
            loseAltitude(outputs);

            pitchControl(outputs);

            //under INIT_LANDING_HEIGHT start stabilizing the plane
            if (Controller.extractPosition(inputs).getyValue() <= INIT_LANDING_HEIGHT) {
                //activate the breaks on the wheels when plane hits the ground
                if (Controller.extractPosition(inputs).getyValue() <= PLANE_HEIGHT_FROM_GROUND) {
                    outputs.setRightBrakeForce(100);
                    outputs.setLeftBrakeForce(100);
                    outputs.setFrontBrakeForce(100);
                    System.out.println(Controller.extractPosition(inputs).getzValue());
                }
                this.stayDown(outputs);
                this.setThrust(outputs);
                this.setHorizontalStabilizer(outputs);

            }
        }

        AutopilotInputs_v2 previousInputs = getPreviousInputs();
        angleOfAttackControl(outputs, previousInputs, inputs);
     // System.out.println(Controller.extractPosition(inputs).getyValue());


        return outputs;
    }

    /**
     * Checks if the drone may initialize landing
     * @param inputs the current inputs of the autopilot
     * @return true if and only if the drone is stabilized and has stabilized for at least minimal stabilizing time
     */
    private boolean mayInitializeLanding(AutopilotInputs_v2 inputs) {
        return (this.getCurrentInputs().getElapsedTime() - this.getStartElapsedTime()) > MINIMAL_STABILIZING_TIME && this.isStabilized(inputs);
    }

    private void loseAltitude(ControlOutputs outputs) {
        float outputThrust  = this.getConfig().getMaxThrust();
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(WING_INCL);
        outputs.setLeftWingInclination(WING_INCL);
        outputs.setHorStabInclination(HORIZONTAL_STABILIZER);
    }

    /**
     *  Makes sure the Drone stays on the ground and doesn't take off again after landing
     */
    private void stayDown(ControlOutputs outputs) {
        float outputThrust  = this.getConfig().getMaxThrust();
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(-WING_INCL);
        outputs.setLeftWingInclination(-WING_INCL);
        outputs.setHorStabInclination(HORIZONTAL_STABILIZER/2);
    }

    /**
     * Controls the pitch, makes sure it doesn't get out of its range (PITCH_THRESHOLD)
     */
    private void pitchControl(ControlOutputs outputs) {
        float pitch = Controller.extractOrientation(this.getCurrentInputs()).getyValue();
        if (abs(pitch) >= PITCH_THRESHOLD) {
            outputs.setHorStabInclination(0f);
        }
    }

    /**
     *  Set the thrust to 0
     */
    private void setThrust(ControlOutputs outputs) {

        outputs.setThrust(0f);
    }

    /**
     * Stabilizes the flight before commencing the landing sequence
     * @param outputs the output object to write the control actions to
     */
    private void stabilizeFlight(ControlOutputs outputs){
        //get the current and previous inputs
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();

        //stabilize the pitch and the roll
        //super.rollControl(outputs, currentInputs);
        pitchStabilizer(outputs, currentInputs, prevInputs);
        rollStabilizer(outputs, currentInputs, prevInputs);
        outputs.setThrust(STABILIZING_THURST);
    }

    private void pitchStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
        //stabilize the pitch
        //extract the current orientation
        Vector orientation = Controller.extractOrientation(currentInputs);
        float pitch = orientation.getyValue();
        PIDController pitchPid =this.getPitchPIDController();
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitch, deltaTime);
        //System.out.println("Pitch result PID" + PIDControlActions);
        //adjust the horizontal stabilizer
        float horizontalInclination = this.getStabilizerStableInclination() - PIDControlActions;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HOR_STABILIZER_MAX);
        outputs.setHorStabInclination(horizontalInclination);
    }

    private void rollStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        Vector orientation = Controller.extractOrientation(currentInputs);
        float roll = orientation.getzValue();
        PIDController rollPid = this.getRollPIDController();
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float PIDControlActions = rollPid.getPIDOutput(roll, deltaTime);
        float rightMainWing = this.getMainStableInclination() + PIDControlActions;
        float leftMainWing = this.getMainStableInclination() - PIDControlActions;

        outputs.setRightWingInclination(capMainWingInclination(rightMainWing));
        outputs.setLeftWingInclination(capMainWingInclination(leftMainWing));

    }

    /**
     * Enforces a maximum onto the main wing inclination based on the MAIN_CAP_DELTA_INCLINATION variable
     * @param inclination the inclination to adjust
     * @return if the inclination is in the range getMainStableInclination +- MAIN_CAP_DELTA_INCLINATION
     *         the same value as the inclination is returned, if it exceeds the border value, the inclination
     *         is set to the border value
     */
    private float capMainWingInclination(float inclination){
        //first determine the lower cap:
        float lowerCap = this.getMainStableInclination() - MAIN_CAP_DELTA_INCLINATION;
        float upperCap = this.getMainStableInclination() + MAIN_CAP_DELTA_INCLINATION;
        if(inclination < lowerCap){
            return lowerCap;
        }
        if(inclination > upperCap){
            return upperCap;
        }
        return inclination;
    }

    /**
     * Checks if the roll and the pitch are stabilized, they are if they are within the appropriate margins
     * (see the constants below)
     * @param inputs the inputs to read the current state from
     * @return true if and only if the roll and the pitch are within the allowed margin
     */
    private boolean isStabilized(AutopilotInputs_v2 inputs){
        //extract the orientation
        Vector orientation = Controller.extractOrientation(inputs);
        //the roll
        float roll = orientation.getzValue();
        //the pitch
        float pitch = orientation.getyValue();
        //check if the roll is within limits
        boolean rollStabilized = abs(roll) < ROLL_STABILIZING_MARGIN;
        boolean pitchStabilized = abs(pitch) < PITCH_STABILIZING_MARGIN;

        return rollStabilized && pitchStabilized;

    }




    private void setHorizontalStabilizer(ControlOutputs outputs){
        //we want to go for zero (stable inclination of the horizontal stabilizer is zero), so the corrective action needs also to be zero
        Vector orientation = Controller.extractOrientation(this.getCurrentInputs());
        Vector orientationPID = this.getOrientationPID().getPIDOutput(orientation, this.getCurrentInputs().getElapsedTime());
        //extract the pitch (positive is upward looking, negative is downward looking)
        float pitch = orientationPID.getyValue();
        float pitchConstant = 2;
        //calculate the desired action of the stabilizer(negative is upward movement, positive downward)
        float desiredAngle = (float) (-pitch/PI*pitchConstant);//the pitch/PI is to get a % of the pitch that we're off
        float outputInclination = min(abs(HOR_STABILIZER_MAX), abs(desiredAngle));
        outputs.setHorStabInclination(outputInclination*signum(desiredAngle));
    }


    /**
     * Constants
     */
    private final static float STOP_VELOCITY = 0.0f;
    private final static float STANDARD_THRUST = 128.41895f * 3.5f;
    private final static float WING_INCL = (float) (PI / 180);
    private final static float HORIZONTAL_STABILIZER = (float) (PI / (180));
    private final static float PITCH_THRESHOLD = (float) (5 * PI / 180);
    private static final float INIT_LANDING_HEIGHT = 8f;
    private final static float MAIN_STABLE = (float) (5*PI/180);
    private final static float STABILIZER_STABLE = 0;
    private final static float ROLL_THRESHOLD = (float)(3*PI/180);
    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);
    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);
    private final static float PLANE_HEIGHT_FROM_GROUND = 1.2f;
    private final static float MAXIMUM_LANDING_VELOCITY = 1f;

    //stabilizing constants
    private final static float STABILIZING_THURST = 550f;
    private final static float ROLL_STABILIZING_MARGIN = (float) (1*PI/180);
    private final static float PITCH_STABILIZING_MARGIN = (float) (1*PI/180);
    private final static float MAIN_CAP_DELTA_INCLINATION = (float) (3*PI/180);
    private final static float MINIMAL_STABILIZING_TIME = 2f;


    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
    private Vector referenceVelocity = new Vector(0,0,-STOP_VELOCITY);
    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);
    private Vector referenceOrientation = new Vector();
    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);
    private float referenceAltitude = 10f;
    private boolean startedDescend = false;

    private static float PITCH_GAIN = 1;
    private static float PITCH_INTEGRAL = 0.2f;
    private static float PITCH_DERIVATIVE = 0;
    private PIDController pitchPIDController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);

    private static float ROLL_GAIN = 1;
    private static float ROLL_INTEGRAL = 0.2f;
    private static float ROLL_DERIVATIVE = 0;
    private PIDController rollPIDController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    private boolean firstControlCall = true;

    private float startElapsedTime;

    /**
     * true if the controller is called for control actions for the first time
     * @return true if and only if the controller is queried for the first time
     */
    public boolean isFirstControlCall() {
        return firstControlCall;
    }

    /**
     * Sets the flag for the first call (one way use)
     */
    public void setFirstControlCall() {
        this.firstControlCall = false;
    }

    /**
     * Getter for the elapsed time at the moment of the first control actions query
     * @return the elapsed time at the first control query
     */
    private float getStartElapsedTime() {
        return startElapsedTime;
    }

    /**
     * Setter for the elapsed time at the moment of the first control actions query
     * @param inputs the inputs at the moment of the first invocation
     */
    private void setStartElapsedTime(AutopilotInputs_v2 inputs) {
        if(!this.isFirstControlCall())
            throw new IllegalArgumentException("not first call");
        this.startElapsedTime = inputs.getElapsedTime();
    }

    private VectorPID getVelocityPID() {
        return this.velocityPID;
    }

    private VectorPID getOrientationPID() {
        return orientationPID;
    }

    private PIDController getAltitudePID() {
        return altitudePID;
    }

    private PIDController getPitchPIDController() {
        return pitchPIDController;
    }

    public PIDController getRollPIDController(){
        return rollPIDController;
    }

    /**
     * Getter for the flag to indicate if the descend is initiated
     * @return the value of the flag
     */
    private boolean isStartedDescend() {
        return startedDescend;
    }

    /**
     * Toggles the start descend flag, once we start to descend, there is no way back
     */
    private void setStartedDescend() {
        this.startedDescend = true;
    }

    //TODO implement these methods accordingly
    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE;
    }

    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return INCLINATION_AOA_ERROR_MARGIN;
    }

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }
}


