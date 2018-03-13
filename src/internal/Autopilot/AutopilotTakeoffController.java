package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;

import static java.lang.Math.*;

/**
 * Created by Martijn on 18/02/2018.
 * A class of takeoff controllers, responsible for controlling the takeoff of the drone
 */
public class AutopilotTakeoffController extends Controller{

    public AutopilotTakeoffController(AutoPilot autopilot){
        //implement constructor
        super(autopilot);
        this.getVelocityPID().setSetPoint(this.referenceVelocity);
        this.getOrientationPID().setSetPoint(this.referenceOrientation);
        this.getAltitudePID().setSetPoint(this.referenceAltitude);
    }

    private void setThrust(ControlOutputs outputs){
        //get the maximal thrust
        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
        //elapsed time:
        float elapsedTime = this.getCurrentInputs().getElapsedTime();
        //first get an approx of the current velocity
        Vector velocity = this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs());
        //then get a response from the PID
        Vector velocityPID = this.getVelocityPID().getPIDOutput(velocity, elapsedTime);
        //use the PID output for the corrective action (gives the error on the velocity)
        //we we take the desired output thrust (for stable config) if the velocity is too high, pull back, to low go faster
        //System.out.println("velocityPID: " + velocityPID);
        float outputThrust = (2-(DESIRED_VELOCITY + velocityPID.getzValue())/DESIRED_VELOCITY)*STANDARD_THRUST;
        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));

    }

    private void setHorizontalStabilizer(ControlOutputs outputs){
        //we want to go for zero (stable inclination of the horizontal stabilizer is zero), so the corrective action needs also to be zero
        Vector orientation = Controller.extractOrientation(this.getCurrentInputs());
        Vector orientationPID = this.getOrientationPID().getPIDOutput(orientation, this.getCurrentInputs().getElapsedTime());
        //extract the pitch (positive is upward looking, negative is downward looking)
        float pitch = orientationPID.getyValue();
        float pitchConstant = 1;
        //calculate the desired action of the stabilizer(negative is upward movement, positive downward)
        float desiredAngle = (float) (-pitch/PI*pitchConstant);//the pitch/PI is to get a % of the pitch that we're off
        float outputInclination = min(abs(HOR_STABILIZER_MAX), abs(desiredAngle));
        outputs.setHorStabInclination(outputInclination*signum(desiredAngle));
    }

    private void setMainWing(ControlOutputs outputs){
        //first get the PID value
        Vector position = Controller.extractPosition(this.getCurrentInputs());
        float altitudeOutput = this.getAltitudePID().getPIDOutput(position.getyValue(), getCurrentInputs().getElapsedTime());
        //if we are under the ideal velocity, we need to maintain a higher inclination
        float zVelocity = -this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs()).getzValue();
        //then get the inclination based on the PID
        float outputInclination = (((getReferenceAltitude() + altitudeOutput)/getReferenceAltitude()  + 1/3f*DESIRED_VELOCITY/zVelocity) *MAIN_STABLE);
        float selectedInclination = max(min(outputInclination, MAIN_MAX), 0f);
        outputs.setLeftWingInclination(selectedInclination);
        outputs.setRightWingInclination(selectedInclination);
    }

    private void checkDesiredAltitude(){
        if(this.getCurrentInputs().getY() >= DESIRED_ALTITUDE){
            this.setReachedDesiredAltitude();
        }
    }

    private boolean hasReachedDesiredVelocity(){
        return this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs()).getSize() >= DESIRED_VELOCITY;
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        this.setCurrentInputs(inputs);
        ControlOutputs outputs = new ControlOutputs();
        this.checkDesiredAltitude();
        //first the simple flight pattern, getting up in the air
        if(!isDesiredAltitude()){
            simpleTakeoffControls(outputs);
            pitchControl(outputs);
        }else{
            //if we have once reached the desired altitude, go for steady state
            this.setThrust(outputs);

            this.setHorizontalStabilizer(outputs);
            this.setMainWing(outputs);
        }
        //outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
        rollControl(outputs, inputs);
        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());
//        System.out.println("Altitude: " + inputs.getY());
//        System.out.println("Orientation: " + extractOrientation(inputs));
//        System.out.println("Outputs: " + outputs);
//        System.out.println("\n");
        return outputs;
    }

    /**
     * Sets the controls for the first part of the takeoff where the drone needs to get actually of the ground
     * @param outputs
     */
    private void simpleTakeoffControls(ControlOutputs outputs) {
        float maxThrust = this.getAutopilot().getMaxThrust();
        float outputThrust = maxThrust;
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(MAIN_TAKEOFF);
        outputs.setLeftWingInclination(MAIN_TAKEOFF);
        outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
    }

    private void pitchControl(ControlOutputs outputs){
        float pitch = Controller.extractOrientation(this.getCurrentInputs()).getyValue();
        if(abs(pitch) >= PITCH_THRESHOLD){
            outputs.setHorStabInclination(0f);
        }
    }

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

    private boolean isDesiredAltitude() {
        return hasReachedDesiredAltitude;
    }

    private void setReachedDesiredAltitude() {
        this.hasReachedDesiredAltitude = true;
    }

    public VectorPID getVelocityPID() {
        return velocityPID;
    }

    public VectorPID getOrientationPID() {
        return orientationPID;
    }

    public PIDController getAltitudePID() {
        return altitudePID;
    }

    public float getReferenceAltitude() {
        return referenceAltitude;
    }

    /**
     * The stable inclination of the main wings
     */
    private final static float MAIN_STABLE = (float) (5*PI/180);


    /**
     * The stable inclination of the stabilizer wings
     */
    private final static float STABILIZER_STABLE = 0;

    /**
     * The takeoff inclination of the main wings
     */
    private final static float MAIN_TAKEOFF = (float) (7*PI/180);
    /**
     * The takeoff inclination of the stabilizer
     */
    private final static float HORIZONTAL_STABILIZER_TAKEOFF = (float) (5*PI/180);

    /**
     * The maximum inclination of the main wings
     */
    private final static float MAIN_MAX = (float) (10*PI/180);

    /**
     * The maximum inclination of the stabilizer wings
     */
    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);

    /**
     * The threshold for the roll for roll control
     */
    private final static float ROLL_THRESHOLD = (float)(3*PI/180);


    private final static float PITCH_THRESHOLD = (float)(5*PI/180);
    /**
     * The safety margin taken for the AOA
     */
    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);

    /**
     * Standard thrust: the reset thrust for control actions
     */
    private final static float STANDARD_THRUST = 128.41895f*3.5f;

    /**
     * The desired altitude to reach with the drone
     */
    private final static float DESIRED_ALTITUDE = 8f;

    /**
     * The desired velocity to reach with the drone
     */
    private final static float DESIRED_VELOCITY = 50.0f;

    /**
     * Flag to check if the desired altitude has been reached
     */
    private boolean hasReachedDesiredAltitude = false;

    /**
     * Reference entry for the velocity (needed for the PID)
     */
    private Vector referenceVelocity = new Vector(0,0,-DESIRED_VELOCITY);

    /**
     * The reference orientation for the drone
     */
    private Vector referenceOrientation = new Vector();
    /**
     * Reference for the entry of the height (needed for the PID)
     */
    private float referenceAltitude = 10f;

    /**
     * A pid controller for the velocity of the drone
     */
    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
    /**
     * A pid controller for the orientation of the drone
     */
    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);

    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);




    //    /**
//     * Generates the control actions for the autopilot
//     * @param inputs the inputs of the autopilot
//     * @return the control actions
//     * @author Anthony Rathe
//     */
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs inputs){
//
//    	setCurrentInputs(inputs);
//
//    	ControlOutputs controlOutputs = new ControlOutputs();
//
//    	AutopilotInputs currentInputs = getCurrentInputs();
//    	AutopilotInputs previousInputs = getPreviousInputs();
//
//    	float currentHeight = currentInputs.getY();
//
//    	Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
//
//    	if (currentHeight < STOP_TAKEOFF_HEIGHT) {
//    		// Still on the ground
//
//    		// Set max thrust
//    		controlOutputs.setThrust(this.getAutopilot().getConfig().getMaxThrust());
//
//    		if (velocityApprox.getyValue() >= LIFTOFF_THRESHOLD ) {
//    			// Drone is lifting off
//
//    			if (currentInputs.getPitch() <= TAKEOFF_PITCH) {
//    				// Start ascending
//    				controlOutputs.setHorStabInclination(STANDARD_INCLINATION);
//    			}else if(currentInputs.getPitch() >= MAX_PITCH){
//    				// Start descending
//    				controlOutputs.setHorStabInclination(-STANDARD_INCLINATION);
//    			}else {
//    				// Stop ascending/descending
//    				controlOutputs.setHorStabInclination(0f);
//    			}
//
//    		}
//
//    	}else {
//    		// In mid-air
//    		// Turn to flight mode
//    		this.getAutopilot().setAPMode(2);
//
//    	}
//        return controlOutputs;
//    }
//
//    @Override
//    protected float getMainStableInclination() {
//        return 0;
//    }
//
//    @Override
//    protected float getStabilizerStableInclination() {
//        return 0;
//    }
//
//    @Override
//    protected float getRollThreshold() {
//        return 0;
//    }
//
//    @Override
//    protected float getInclinationAOAErrorMargin() {
//        return 0;
//    }
//
//    @Override
//    protected float getStandardThrust() {
//        return 0;
//    }
//
//    private static final float LIFTOFF_THRESHOLD = 1f;
//    private static final float STOP_TAKEOFF_HEIGHT = 10f;
//    private static final float TAKEOFF_PITCH = (float)Math.PI/18f;
//    private static final float MAX_PITCH = (float)Math.PI/4f;
//    private static final float STANDARD_INCLINATION = (float) PI/12;
}
