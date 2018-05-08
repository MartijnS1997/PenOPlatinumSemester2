package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 8/05/2018.
 * A class of controllers used to avoid collisions
 */
public class CasTestController extends Controller {
    public CasTestController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //generate the control outputs
        ControlOutputs outputs = new ControlOutputs(getStandardOutputs());

        //get the pitch controls
        this.pitchControl(outputs, currentInputs, previousInputs);
        //get thrust controls
        outputs.setThrust(1000f);
        //return the current outputs
        System.out.println(outputs);
        return outputs;
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        return false;
    }

    @Override
    public void reset() {

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
        float referenceVelocity = 50;
        //then get the outputs from the standard cruise control implemented in the main controller class
        this.flightCruiseControl(outputs,currentInputs, previousInputs, thrustControl, referenceVelocity);
    }

    private float getReferencePitch() {
        return referencePitch;
    }

    private PIDController getPitchController(){
        return this.pitchController;
    }

    private PIDController getThrustController(){
        return this.thrustController;
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
     * The reference pitch used for the tests
     */
    private float referencePitch = (float) (-15*PI/180f);

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
