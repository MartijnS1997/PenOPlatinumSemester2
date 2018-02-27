package internal.Autopilot;

import Autopilot.AutopilotConfig;
import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;
import internal.Physics.PhysXEngine;
import internal.Helper.Vector;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

/**
 * Created by Martijn on 18/02/2018.
 * A class of controllers
 */
//Todo migrate the AOA physics to the controller or the PhysXEngine (will also be needed in the landing and takeoff)
public abstract class Controller {

    public Controller(AutoPilot autopilot){
        this.autopilot = autopilot;
        this.setPreviousInputs(dummyData);
        this.currentInputs = dummyData;
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

    /**
     * Getter for the main wing stable inclination
     * @return the main wing stable inclination
     */
    protected abstract float getMainStableInclination();

    /**
     * Getter for the stabilizer wings stable inclination
     * @return the stabilizer stable inclination
     */
    protected abstract float getStabilizerStableInclination();

    /**
     * Getter for the threshold for when roll control is activated
     * @return the roll threshold
     */
    protected abstract float getRollThreshold();

    /**
     * Getter for the extra error margin builtin to account for AOA calculation errors
     * @return the error margin for the AOA
     */
    protected abstract float getInclinationAOAErrorMargin();

    /**
     * Getter for the standard thrust for controlling the drone
     * @return the standard thrust
     */
    protected abstract float getStandardThrust();

    /*
  * Supplementary control methods
  */
    protected void rollControl(ControlOutputs outputs, AutopilotInputs currentInput){
        float roll = currentInput.getRoll();

        if(roll >= this.getRollThreshold()){
            outputs.setRightWingInclination(-this.getMainStableInclination());
        }
        else if(roll <= - this.getRollThreshold()){
            outputs.setLeftWingInclination(-this.getMainStableInclination());
        }else{
            // change nothing
        }
    }

    /**
     * Checks if the current control outputs are realisable under the angle of attack constraint provided
     * by the autopilot configuration. If not the controls are adjusted to fit the constraints
     * @param controlOutputs the control outputs to be checked
     */
    protected void angleOfAttackControl(ControlOutputs controlOutputs,AutopilotInputs prevInputs, AutopilotInputs currentInputs){

        //first check if the current and the previous steps are initialized, if not so delete all control actions
        //and set to standard value
        if(currentInputs == null || prevInputs == null){
            controlOutputs.reset();
            return;
        }
        //first prepare all the variables
        PhysXEngine.PhysXOptimisations optimisations = this.getAutopilot().getPhysXOptimisations();
        Vector orientation = extractOrientation(currentInputs);
        Vector velocity = this.getVelocityApprox(prevInputs, currentInputs);
        Vector rotation = this.getRotationApprox(prevInputs, currentInputs);
        float angleOfAttack = this.getAutopilot().getConfig().getMaxAOA();

        //change until the controls fit
        AOAControlMainLeft(controlOutputs, optimisations,angleOfAttack, orientation, rotation, velocity);
        AOAControlMainRight(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlHorStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlVerStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainLeft(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack,  Vector orientation, Vector rotation, Vector velocity){
        float inclinationBorder1 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getLeftWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setLeftWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainRight(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){
        float inclinationBorder1 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getRightWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setRightWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlHorStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){

        float inclinationBorder1 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getHorStabInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setHorStabInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlVerStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){

        float inclinationBorder1 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getVerStabInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setVerStabInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }


    /*
    Helper methods
     */
    //TODO account for the fact that the distance between the borders could be smaller than the error margin
    private float setBetween(float value, float border1, float border2, float errorMargin){
        //first check if the value isn't already between the borders:
        float[] borders = sortValue(border1, border2);
        float lowerBorder = borders[0];
        float upperBorder = borders[1];
        //check if it is already between the borders
        if(value >= lowerBorder && value <= upperBorder)
            return value;

        //if not so, set it between with a given error margin
        //check if the value is closest to the lower border
        if(abs(lowerBorder - value) <= abs(upperBorder - value)){
            return lowerBorder - signum(lowerBorder)*errorMargin;
        }else{
            return upperBorder - signum(upperBorder)*errorMargin;
        }

    }

    /**
     * Sorts the two values
     * @param value1 the first value to be sorted
     * @param value2 the second value to be sorted
     * @return an array of size 2 with the smallest value first and the largest second.
     */
    private float[] sortValue(float value1, float value2){

        float[] sortedArray = new float[2];
        if(value1 <= value2) {
            sortedArray[0] = value1;
            sortedArray[1] = value2;
        }else{
            sortedArray[0] = value2;
            sortedArray[1] = value1;
        }

        return sortedArray;
    }

    protected float getTotalMass(){
        AutoPilot autopilot = this.getAutopilot();
        float mainWings = autopilot.getMainWingMass()*2;
        float stabilizers = autopilot.getStabilizerMass()*2;
        float engine = autopilot.getEngineMass();

        return mainWings + stabilizers + engine;
    }

    /**
     * Calculate an approximation of the velocity
     * @param prevInputs the autopilot inputs at moment k-1 needed for the derivative
     * @param currentInputs the autopilot inputs at moment k needed for the derivative
     * @return the approximation of the velocity
     * elaboration: see textbook numerical math for derivative methods, the
     * derivative of f(k+1) - f(k-1) / (2*timeStep) has O(h²) correctness
     */
    protected Vector getVelocityApprox(AutopilotInputs prevInputs, AutopilotInputs currentInputs){
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevPos = extractPosition(prevInputs);
        Vector currentPos = extractPosition(currentInputs);

        Vector posDiff = currentPos.vectorDifference(prevPos);
        float timeDiff = currentTime - prevTime;

        return posDiff.scalarMult(1/timeDiff);
    }

    /**
     * Approximates the current rotation of the drone based on the old rotations
     * @param prevInputs  the previous autopilot inputs (moment k-1) needed for the derivative
     * @param currentInputs  the current autopilot inputs (moment k) needed for the derivative
     * @return an approx for the rotation (first calculate the rotation in heading pitch and roll components
     *         and transform them to the actual rotational components)
     */
    private Vector getRotationApprox(AutopilotInputs prevInputs, AutopilotInputs currentInputs){

        //get the passed time interval
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        //extract the orientations from the inputs
        Vector prevOrient = extractOrientation(prevInputs);
        Vector currentOrient = extractOrientation(currentInputs);

        Vector orientDiff = currentOrient.vectorDifference(prevOrient);
        float timeDiff = currentTime - prevTime;

        // the given rotation vector is given in heading pitch and roll components
        Vector rotationHPR = orientDiff.scalarMult(1/timeDiff);
        // convert back to the world axis rotation vector
        return PhysXEngine.HPRtoRotation(rotationHPR, currentOrient);
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the orientation of the drone in vector format
     */
    private static Vector extractOrientation(AutopilotInputs inputs){
        return new Vector(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the position of the drone in vector format
     */
    private static Vector extractPosition(AutopilotInputs inputs){
        return new Vector(inputs.getX(), inputs.getY(), inputs.getZ());
    }


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
     * Getter for the current inputs (the autopilot inputs object)
     * @return the current outputs
     */
    public AutopilotInputs getCurrentInputs() {
        return currentInputs;
    }
    
    /**
     * Setter for the current inputs of the drone, the old currentInputs are automatically
     * set previous inputs
     * @param currentInputs the current input for the autopilot
     */
    protected void setCurrentInputs(AutopilotInputs currentInputs) {
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
    protected void setPreviousInputs(AutopilotInputs previousInputs){
        this.previousInputs = previousInputs;
    }

    private AutopilotInputs currentInputs;
    private AutopilotInputs previousInputs;

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
}