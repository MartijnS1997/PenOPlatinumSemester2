package internal;

import Autopilot.AutopilotConfig;
import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;

/**
 * Created by Martijn on 18/02/2018.
 * A class of controllers
 */
//Todo migrate the AOA physics to the controller or the PhysXEngine (will also be needed in the landing and takeoff)
public abstract class Controller {

    public Controller(AutoPilot autopilot){
        this.autopilot = autopilot;
    }

    /**
     * Generates the control actions for the autopilot
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    public abstract AutopilotOutputs getControlActions(AutopilotInputs inputs);

    /*
    Getters and setters
     */

    protected abstract float getMainStableInclination();

    protected abstract float getStabilizerStableInclination();

    protected abstract float getRollThreshold();

    protected abstract float getInclinationAOAMargin();

    protected abstract float getStandardThrust();

    /**
     * Getter for the autopilot of the drone
     * @return the autopilot
     */
    protected AutoPilot getAutopilot() {
        return autopilot;
    }


    /**
     * Getter for the configuration of the drone
     * @return the configuration
     */
    private AutopilotConfig getConfig() {
        return config;
    }

    /**
     * Setter for the configuration
     */
    protected void setConfig(AutopilotConfig config){
        this.config = config;
    }


    /**
     * Object that stores the autopilot of the drone
     */
    private AutoPilot autopilot;

    /**
     * Object that stores the configuration of the drone
     */
    private AutopilotConfig config;


    /**
     * An implementation of AutopilotOutputs used in the controller for cascading control (passes trough the basic
     * controller, roll control and AOA controll)
     */
    class ControlOutputs implements AutopilotOutputs{

        ControlOutputs(){
            //do nothing, everything stays initialized on zero
        }

        /**
         * Set to default values
         * used to reset the outputs if the controller is not fully initialized
         */
        protected void reset(){

            this.setRightWingInclination(getMainStableInclination());
            this.setLeftWingInclination(getMainStableInclination());
            this.setHorStabInclination(getStabilizerStableInclination());
            this.setVerStabInclination(getStabilizerStableInclination());

        }

        @Override
        public float getThrust() {
            return this.thrust;
        }

        @Override
        public float getLeftWingInclination() {
            return this.leftWingInclination;
        }

        @Override
        public float getRightWingInclination() {
            return this.rightWingInclination;
        }

        @Override
        public float getHorStabInclination() {
            return this.horStabInclination;
        }

        @Override
        public float getVerStabInclination() {
            return this.verStabInclination;
        }

        @Override
        public float getFrontBrakeForce() {
            return 0;
        }

        @Override
        public float getLeftBrakeForce() {
            return 0;
        }

        @Override
        public float getRightBrakeForce() {
            return 0;
        }

        /**
         * Setter for the Thrust
         * @param thrust the desired thrust
         */
        public void setThrust(float thrust) {
            this.thrust = thrust;
        }

        /**
         * Setter for the left wing inclination
         * @param leftWingInclination
         */
        public void setLeftWingInclination(float leftWingInclination) {
            this.leftWingInclination = leftWingInclination;
        }

        /**
         * Setter for the right wing inclination
         * @param rightWingInclination the desired right wing inclination
         */
        public void setRightWingInclination(float rightWingInclination) {
            this.rightWingInclination = rightWingInclination;
        }

        /**
         * Setter for the horizontal stabilizer inclination
         * @param horStabInclination the desired horizontal stabilizer inclination
         */
        public void setHorStabInclination(float horStabInclination) {
            this.horStabInclination = horStabInclination;
        }

        /**
         * Setter for the vertical stabilizer inclination
         * @param verStabInclination the desired vertical stabilizer inclination
         */
        public void setVerStabInclination(float verStabInclination) {
            this.verStabInclination = verStabInclination;
        }

        //initialize the writes to the stable state of the drone
        private float thrust = getStandardThrust();
        private float leftWingInclination = getMainStableInclination();
        private float rightWingInclination = getMainStableInclination();
        private float horStabInclination = getStabilizerStableInclination();
        private float verStabInclination = getStabilizerStableInclination();

        @Override
        public String toString() {
            return "ControlOutputs{" +
                    "thrust=" + thrust +
                    ", leftWingInclination=" + leftWingInclination +
                    ", rightWingInclination=" + rightWingInclination +
                    ", horStabInclination=" + horStabInclination +
                    ", verStabInclination=" + verStabInclination +
                    '}';
        }
    }
}
