package internal;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;

import static java.lang.Math.*;

/**
 * Created by Martijn on 30/10/2017.
 * Appended and edited by Anthony Rath� on 6/11/2017
 * A class of Autopilot Controllers
 */
public abstract class AutoPilotFlightController extends Controller{


	/**
     * Constructor for the autopilotController
     * @param autopilot
     */
    public AutoPilotFlightController(AutoPilot autopilot){
        super(autopilot);
        this.setPreviousInputs(dummyData);
        this.currentInputs = dummyData;
    }



    private void logControlActions(ControlOutputs outputs, String controlString){

        controlString += "Left wing inclination: "  + outputs.getLeftWingInclination()*RAD2DEGREE + "\n";
        controlString += "Right wing inclination: " + outputs.getRightWingInclination()*RAD2DEGREE + "\n";
        controlString += "Horizontal stabilizer inclination: " + outputs.getHorStabInclination()*RAD2DEGREE + "\n";
        controlString += "Vertical wing inclination" + outputs.getVerStabInclination()*RAD2DEGREE + "\n";

        // write the controls to the recorder
        FlightRecorder recorder = this.getFlightRecorder();
        recorder.appendControlLog(controlString);
    }

    /**
     * determines the largest value of both entries, if both are negative an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the largest entry of both
     * @throws IllegalArgumentException thrown if both entries are negative
     */
    private float getMaxPos(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 >= entry2 && entry1 >= 0){
            return entry1;
        }else if(entry2 >= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /**
     * determines the smallest value of both entries, if both are positive an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the smallest entry of both
     * @throws IllegalArgumentException thrown if both entries are positive
     */
    private float getMinNeg(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 <= entry2 && entry1 <= 0){
            return entry1;
        }else if(entry2 <= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /*
    Getters and setters
     */



    /**
     * Getter for the current inputs (the autopilot inputs object)
     * @return the current outputs
     */
    public AutopilotInputs getCurrentInputs() {
        return currentInputs;
    }

    /**
     * get the flight recorder of the drone
     * @return
     */
    public FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    /**
     * Setter for the flight recorder
     * @param flightRecorder
     */
    public void setFlightRecorder(FlightRecorder flightRecorder) {
        this.flightRecorder = flightRecorder;
    }

    /**
     * Setter for the current inputs of the drone, the old currentInputs are automatically
     * set previous inputs
     * @param currentInputs the current input for the autopilot
     */
    public void setCurrentInputs(AutopilotInputs currentInputs) {
        //first write to the previous outputs
        this.setPreviousInputs(this.getCurrentInputs());

        //then write to the new ones.
        this.currentInputs = currentInputs;
    }

    /**
     * Returns the previous Autopilot Inputs
     */
    public AutopilotInputs getPreviousInputs() {
        return previousInputs;
    }

    /**
     * Setter for the autopilot inputs
     * @param previousInputs the pervious inputs
     */
    private void setPreviousInputs(AutopilotInputs previousInputs){
        this.previousInputs = previousInputs;
    }

    /*
    Getters and setters
     */

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }

    /**
     * Get the PID used for the corrections on the x-coordinate inputs
     * @return the pid controller
     */
    public PIDController getxPID() {
        return xPID;
    }

    /**
     * Get the PID used for the corrections on the y-coordinate inputs
     * @return
     */
    public PIDController getyPID() {
        return yPID;
    }

    private AutopilotInputs currentInputs;
    private AutopilotInputs previousInputs;
    private FlightRecorder flightRecorder;
    private PIDController xPID = new PIDController(1.f, 0.2f, 0.2f);
    private PIDController yPID = new PIDController(1.f, 0.2f, 0.2f);


    private static final float STANDARD_THRUST = 32.859283f *2;
    private static final float RAD2DEGREE = (float) (180f/ PI);


    /**
     * dummy data used for the initialization of the drone
     */
    private  static AutopilotInputs dummyData = new AutopilotInputs() {
        @Override
        public byte[] getImage() {
            return new byte[0];
        }

        @Override
        public float getX() {
            return 0;
        }

        @Override
        public float getY() {
            return 0;
        }

        @Override
        public float getZ() {
            return 0;
        }

        @Override
        public float getHeading() {
            return 0;
        }

        @Override
        public float getPitch() {
            return 0;
        }

        @Override
        public float getRoll() {
            return 0;
        }

        @Override
        public float getElapsedTime() {
            return 0;
        }
    };



    /**
     * A class of PID controllers
     */
    class PIDController {
        /**
         * Constructor for a PID controller object
         * @param gainConstant the constant for the gain of de PID controller (also denoted as Kp)
         * @param integralConstant the constant for the integral of the PID controller (also denoted as Ki)
         * @param derivativeConstant the constant for the derivative of the PID controller (also denoted as Kd)
         */
        PIDController(float gainConstant, float integralConstant, float derivativeConstant){
            // set the constants
            this.gainConstant = gainConstant;
            this.integralConstant = integralConstant;
            this.derivativeConstant = derivativeConstant;
        }

        /**
         * Constructs a PID controller with the gain, integral and derivative parameters set to 1.0
         */
        private PIDController(){
            this(1.0f, 1.0f, 1.0f);
        }

        /**
         * Calculates the output for the current inputs of the PID controller
         * @param input the input signal of the controller (from the feedback loop)
         * @param elapsedTime the elapsed time during the simulation
         * @return the output of the PID controller for the given inputs
         * note: algorithm comes from https://en.wikipedia.org/wiki/PID_controller
         */
        float getPIDOutput(float input, float elapsedTime){

            // P part is proportional (set to 1)
            // I part reduces overall error
            // D part reduces the oscillation of the path
            // variables needed for calculation
            float setPoint = this.getSetPoint();
            float prevError = this.getPreviousError();
            float integral = this.getIntegral();
            float Kp = this.getGainConstant();
            float Ki = this.getIntegralConstant();
            float Kd = this.getDerivativeConstant();
            float deltaTime = elapsedTime - this.getPreviousTime();

            //determine the PID control factors
            float error = setPoint - input;
            float derivative = (error - prevError)/deltaTime;
            integral = integral + error*deltaTime;

            // calculate the output
            float output = Kp * error + Ki*integral + Kd*derivative;

            // save the state
            this.setIntegral(integral);
            this.setPreviousError(error);
            this.setPreviousTime(elapsedTime);

            return output;
        }

        /*
        Getters and setters
         */

        /**
         * get the integral part (saved over the course of the algorithm)
         * @return the integral part of the PID
         */
        private float getIntegral() {
            return integral;
        }

        /**
         * set the integral part of the PID (saved over the course of the algorithm)
         * @param integral
         */
        private void setIntegral(float integral) {
            this.integral = integral;
        }

        /**
         * get the error of the previous iteration
         * @return the previous error
         */
        private float getPreviousError() {
            return previousError;
        }

        /**
         * Set the previous error (used when the new values for the error are loaded)
         * @param previousError
         */
        private void setPreviousError(float previousError) {
            this.previousError = previousError;
        }

        /**
         * The set point is the desired value used for reference, in our case it is 0.0
         * @return
         */
        private float getSetPoint() {
            return setPoint;
        }

        /**
         * Set the set point of the PID
         * @param setPoint
         */
        protected void setSetPoint(float setPoint) {
            this.setPoint = setPoint;
        }

        /**
         * Get the previous time of the simulation (used for the diff part of the PID)
         * @return
         */
        public float getPreviousTime() {
            return previousTime;
        }

        /**
         * Set the previous time of the simulation (used when setting the new values of the drone)
         * @param previousTime
         */
        public void setPreviousTime(float previousTime) {
            this.previousTime = previousTime;
        }

        /**
         * Constant used for the gain (proportional) part of the PID
         * @return
         */
        private float getGainConstant() {
            return gainConstant;
        }

        /**
         * Constant used for the integral part of the PID
         * @return
         */
        private float getIntegralConstant() {
            return integralConstant;
        }

        /**
         * Constant used for the derivative part of the PID
         * @return
         */
        public float getDerivativeConstant() {
            return derivativeConstant;
        }

        private float integral = 0.0f;
        private float previousError = 0.0f;
        private float setPoint = 0.0f;
        private float previousTime = 0.0f;
        private float gainConstant;
        private float integralConstant;
        private float derivativeConstant;
    }

}
