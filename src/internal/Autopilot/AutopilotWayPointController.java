package internal.Autopilot;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;
import internal.Exceptions.SimulationEndedException;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

/**
 * Created by Martijn on 13/03/2018.
 */
public class AutopilotWayPointController extends Controller {

    public AutopilotWayPointController(AutoPilot autopilot){
        // implement constructor
        super(autopilot);
        //add some way points to test against
        List<WayPoint> wayPoints = this.getPath();
//        wayPoints.add(new WayPoint(new Vector(5,20f, -60f), 20));
//        wayPoints.add(new WayPoint(new Vector(10,20f, -120f), 20));
//        wayPoints.add(new WayPoint(new Vector(15,20f, -180f), 20));
        int nbwaypoints = 50;
        for(int i = 1; i < nbwaypoints; i++){
//            float turningRadius = 1500;
//            float x = (float) (-turningRadius*cos(PI/nbwaypoints * i) + turningRadius);
//            float z = (float) (-sin(PI/nbwaypoints*i)*turningRadius);

            wayPoints.add(new WayPoint(new Vector(i*5, 20, -i*60), 40));
        }

    }

    /**
     * Sets the control actions for the thrust of the autopilot
     * @param outputs the outputs-object generated by the autopilot
     */
    private void thrustControl(Controller.ControlOutputs outputs){


        //first get the maximum thrust
        float maxThrust =  this.getAutopilot().getConfig().getMaxThrust();
        //also get the current inputs
        AutopilotInputs inputs = this.getCurrentInputs();
        //extract the orientation
        Vector orientation = Controller.extractOrientation(inputs);
        //extract the position
        Vector position = Controller.extractPosition(inputs);
        //get the current way point
        WayPoint wayPoint = this.getCurrentWayPoint();

        //get the pitch difference
        float refPitch = wayPoint.referencePitch(inputs);

        //get the distance to the target
        float refDist = wayPoint.distanceToWayPoint(position);
        float minRefDist = 20;
        //System.out.println("reference pitch" + refPitch);
        //the thrust should be stable if target is faraway, and higher if close
        float outputThrust = (float) (BASE_THRUST + THRUST_COEFF*refPitch*refPitch*RAD2DEGREE + THRUST_COEFF/20*pow(orientation.getzValue()*RAD2DEGREE,1));

        outputs.setThrust(max(min(outputThrust, maxThrust), 0));
        //System.out.println("thrust " + outputs.getThrust());
    }

    private float sigmoid(float x) {
        return (float) (1/(1+Math.exp(-x)));
    }


    /**
     * Control actions for the pitch of the drone (influences the horizontal stabilizer position)
     * @param outputs the outputs object generated by the autopilot and sent to the drone
     */
    private void pitchControl(Controller.ControlOutputs outputs){
        //first get the current way point
        WayPoint wayPoint = this.getCurrentWayPoint();

        //we need the current inputs
        AutopilotInputs inputs = this.getCurrentInputs();
        //calculate the elapsed time:
        //then calculate the ideal pitch
        //get the reference pitch:
        float referencePitch = wayPoint.referencePitch(inputs);
        //System.out.println("reference pitch: " + referencePitch);
        //then extract the current pitch
        float currentPitch = Controller.extractOrientation(inputs).getyValue();
        //System.out.println("current pitch: " + currentPitch);
        //then get the PID output for the given set point (the set point is calculated for every iteration)
        Controller.PIDController pitchController = this.getPitchPID();
        float pidResult = pitchController.getPIDOutputSetPoint(currentPitch, inputs.getElapsedTime(), referencePitch);
        //System.out.println("PID horizontal output: " + pidResult);
        //if the pid result is positive: the set point is larger than the input -> lower the pitch (pos inclination)
        //if the pid result is negative: the set point is smaller than the input -> increase the pitch (neg inclination)
        float horizontalInclination = this.getStabilizerStableInclination() - pidResult; //todo change the constants for better result
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HORIZONTAL_CAP_INCLINATION);
        outputs.setHorStabInclination(horizontalInclination);
    }

    private void bankControl(Controller.ControlOutputs outputs){
        //get the current way point
        WayPoint wayPoint = this.getCurrentWayPoint();
        //then get the current inputs
        AutopilotInputs inputs = this.getCurrentInputs();
        //extract the current orientation:
        Vector orientation = Controller.extractOrientation(inputs);
        //extract the position
        Vector position = Controller.extractPosition(inputs);

        //extract the current heading
        float heading = orientation.getxValue();
        //extract the elapsed time
        float elapsedTime = inputs.getElapsedTime();
        //get the reference heading
        float refHeading = wayPoint.referenceHeading(inputs);
        System.out.println("reference heading: " + refHeading*RAD2DEGREE);
        System.out.println("Orientation: " + orientation.scalarMult(RAD2DEGREE));
        //get the heading PID
        Controller.PIDController bankControl = this.getBankPID();
        //get the output from the PID
        float pidInput = -signum(refHeading)*abs(refHeading-heading);
        System.out.println(pidInput);
        //float pidOutput = bankControl.getPIDOutput(pidInput, elapsedTime);
        float pidOutput = bankControl.getPIDOutput(-signum(refHeading)*this.getHeadingDifference(), elapsedTime);

        System.out.println("currentHeading Error: " + abs(refHeading-heading)*RAD2DEGREE);
        //bankingError+= abs(refHeading-heading);
        //System.out.println("accumulated Banking Error: " + bankingError);
        System.out.println("Current banking PID output: " + pidOutput);
        //pidOutput*=sqrt(abs(wayPoint.distanceToWayPoint(position)));
        System.out.println("Current banking PID output: " + pidOutput);

        System.out.println("The heading difference calc with vectors: " + this.getHeadingDifference());
        //now adjust the heading of the drone by banking
        //banking is done by setting the horizontal stabilizer upward (may be done manually - or relied on by the pid controller)
        //and inclining the main wings
        float mainLeftWingInclination = this.getMainStableInclination() - pidOutput;
        float mainRightWingInclination = this.getMainStableInclination() + pidOutput;

        //put some extra umpf into the horizontal stabilizer
        outputs.setHorStabInclination(outputs.getHorStabInclination() + BANK_HOR_STAB_EXTRA);

        outputs.setLeftWingInclination(capMainWingInclination(mainLeftWingInclination));
        outputs.setRightWingInclination(capMainWingInclination(mainRightWingInclination));

        //it may be a good idea to add a bit vertical stabilizer into the mix
//        if(pidOutput>0.2){
//            outputs.setVerStabInclination(-(float) (3*PI/180));
//        }if(pidOutput<0.2){
//            outputs.setVerStabInclination(+(float)(3*PI/180));
//        }
    }

    private double bankingError  = 0;

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
     * Generates the control actions for the autopilot
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs inputs){
        System.out.println("Current position: " + Controller.extractPosition(inputs));
        this.setCurrentInputs(inputs);
        //create a new output object
        Controller.ControlOutputs outputs = new Controller.ControlOutputs();
        this.wayPointControl(inputs);
        System.out.println("Current way point: " + this.getCurrentWayPoint());
        //then issue control actions
        //get the control actions for the thrust
        thrustControl(outputs);
        //get the control actions for the pitch
        pitchControl(outputs);
        //get the control actions for the roll
        bankControl(outputs);
        //roll control
        rollControl(outputs, inputs);

        System.out.println("control outputs: " + outputs);

        //check if everything is within allowed parameters
        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());System.out.println(outputs);
        System.out.println();

        return outputs;
    }

    /**
     * Checks if the current way point has been reached, if so increments the way point index
     * @param inputs the inputs of the autopilot, generated by the drone
     */
    private void wayPointControl(AutopilotInputs inputs){
        WayPoint currentWayPoint = this.getCurrentWayPoint();
        //check if we've reached the currently active way point
        boolean currReached = currentWayPoint.isReached(inputs);
        if(currReached){
            //if so, activate the next way point
            this.toNextWayPoint();
            currentWayPoint = this.getCurrentWayPoint();
            //check if we've reached the final way point
            if(currentWayPoint == null){
                //if so, throw simulation ended
                throw new SimulationEndedException();
            }
        }
    }
    @Override
    protected void rollControl(Controller.ControlOutputs outputs, AutopilotInputs currentInput){
        float roll = currentInput.getRoll();

        if(roll >= this.getRollThreshold()&&isSteeringLeft(outputs)){
            outputs.setRightWingInclination(this.getMainStableInclination());
            outputs.setLeftWingInclination(this.getMainStableInclination());
        }
        else if(roll <= - this.getRollThreshold()&&isSteeringRight(outputs)){
            outputs.setLeftWingInclination(this.getMainStableInclination());
            outputs.setRightWingInclination(this.getMainStableInclination());
        }else{
            // change nothing
        }
    }

    private boolean isSteeringRight(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() < this.getMainStableInclination();
    }

    private boolean isSteeringLeft(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() > this.getMainStableInclination();
    }


    private float getHeadingDifference(){
        AutopilotInputs inputs = this.getCurrentInputs();
        //get the orientation of the drone
        Vector orientation = Controller.extractOrientation(inputs);
        //then calculate the heading vector in the world axis system (the negative z-unit vector in the drone axis system)
        Vector headingVectorDrone = new Vector(0,0,-1);
        Vector headingVectorWorld = PhysXEngine.droneOnWorld(headingVectorDrone, orientation);

        //get the current way point
        WayPoint currentWayPoint = this.getCurrentWayPoint();
        //then get the reference heading vector
        Vector refHeadingVector = currentWayPoint.referenceHeadingVector(inputs);

        //then calculate the angle between the two vectors
        return refHeadingVector.getAngleBetween(headingVectorWorld);

    }

    //TODO implement these methods accordingly
    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE_INCLINATION;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return HORIZONTAL_STABLE_INCLINATION;
    }

    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return AOA_CALC_ERROR_MARGIN;
    }

    @Override
    protected float getStandardThrust() {
        return 0;
    }

    /**
     * getter for the current way point
     * @return the current way point
     */
    private WayPoint getCurrentWayPoint(){
        int index = this.getCurrentWayPointIndex();
        //get the path
        List<WayPoint> path = this.getPath();

        //check if the path size is equal to the index
        if(path.size() == index){
            //if so return null
            return null;
        }
        // otherwise return the element at the current index
        return path.get(index);
    }

    /**
     * increments the way point index
     * Todo add to check if we are close enough to advance to the next node
     */
    private void toNextWayPoint(){
        this.currentWayPointIndex ++;
    }

    private int getCurrentWayPointIndex() {
        return currentWayPointIndex;
    }

    public List<WayPoint> getPath() {
        return path;
    }

    /**
     * Getter for the previous thrust
     * @return the previous thrust ordered by the autopilot
     */
    public float getPrevThrust() {
        return prevThrust;
    }

    /**
     * Setter for the previous thrust
     * @param prevThrust the previous thrust
     */
    public void setPrevThrust(float prevThrust) {
        this.prevThrust = prevThrust;
    }

    /**
     * Getter for the PID controller that controls the pitch of the drone
     * @return the controller that controls the pitch of the drone
     */
    private Controller.PIDController getPitchPID() {
        return pitchPID;
    }

//    /**
//     * Getter for the PID controller that controls the thrust of the drone
//     * @return the controller that controls the thrust of the drone
//     */
//    public PIDController getThrustPID() {
//        return thrustPID;
//    }

    /**
     * Getter for the bank PID controller (controls the banking inclination for the drone
     * @return the banking controller
     */
    private Controller.PIDController getBankPID() {
        return bankPID;
    }

    private final static float PITCH_GAIN = .9f;
    private final static float PITCH_DERIVATIVE = 0.f;
    private final static float PITCH_INTEGRAL = 0.4f;

    private Controller.PIDController pitchPID = new Controller.PIDController(PITCH_GAIN,  PITCH_INTEGRAL, PITCH_DERIVATIVE);

    private final static float BANK_GAIN = 0.5f;
    private final static float BANK_DERIVATIVE = 1.5f ;
    private final static float BANK_INTEGRAL = .2f ;
    private final static float BANK_HOR_STAB_EXTRA = (float) (-3*PI/180);

    private Controller.PIDController bankPID = new Controller.PIDController(BANK_GAIN, BANK_INTEGRAL, BANK_DERIVATIVE);

    private final static float BASE_THRUST = 550f;
    private final static float THRUST_COEFF = 50;

//    private final static float THRUST_GAIN = 0.1f;
//    private final static float THRUST_DERIVATIVE = -0.1f;
//    private final static float THRUST_INTEGRAL = 0.0f;
//    private PIDController thrustPID = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

    /**
     * List that stores the current path
     */
    private List<WayPoint> path = new ArrayList<>();

    private int currentWayPointIndex = 0;

    private final static float MAIN_STABLE_INCLINATION = (float) (5*PI/180);
    private final static float HORIZONTAL_STABLE_INCLINATION = 0f;
    private final static float ROLL_THRESHOLD = (float) (10*PI/180);
    private final static float AOA_CALC_ERROR_MARGIN = (float) (2*PI/180);
    private final static float HORIZONTAL_CAP_INCLINATION = (float) (10*PI/180);
    private final static float MAIN_CAP_DELTA_INCLINATION = (float) (3*PI/180);
    private final static float RAD2DEGREE = (float) (180/PI);

    /**
     * Variable that stores the previous value for the thrust
     */
    private float prevThrust = 0f;

    /**
     * A class of way points used in navigation and PID control
     */
    private class WayPoint{
        public WayPoint(Vector position, float acceptanceRadius){
            this.position = position;
            this.acceptanceRadius = acceptanceRadius;
        }

        /**
         * Calculates the pitch used as the reference for the autopilot
         * @param inputs the inputs of the autopilot generated by the testbed (containing the necessary data)
         * @return the pitch to the way point
         */
        public float referencePitch(AutopilotInputs inputs){
            //first get the position of the drone
            Vector dronePos = Controller.extractPosition(inputs);
            //then get the distance between the drone and way point (used as the hypotenuse)
            float hypotenuse = this.distanceToWayPoint(dronePos);
            //then get the height difference (used for the opposite side)
            float oppositeSide = this.getPosition().getyValue() - dronePos.getyValue();
            //then get the reference pitch by arcSin(opposite/hypotenuse)
            float pitch = (float) asin(oppositeSide/hypotenuse);

            return pitch;
        }

        /**
         * Calculates the heading used as the reference for the autopilot
         * @param inputs the inputs of the autopilot genereated by the testbed (containing the necessary data)
         * @return the heading to the way point
         */
        public float referenceHeading(AutopilotInputs inputs){
            //first get the position of the drone
            Vector dronePos = Controller.extractPosition(inputs);
            //then get the wayPoint position
            Vector wayPointPos = this.getPosition();
            //then take the x-difference for the tan
            float deltaX = wayPointPos.getxValue() - dronePos.getxValue();
            float deltaZ = wayPointPos.getzValue() - dronePos.getzValue();
            //System.out.println("delta X: " + deltaX + "; drone pos: " + dronePos.getxValue() + "; way point pos: " + wayPointPos);
            //System.out.println("delta Z: " + -deltaZ);
            //now calculate the heading:
            float heading = -(float) atan2(deltaX, -deltaZ);
            return heading;
        }

        /**
         * Calculates the vector that points from the drone to the selected way point
         * @param inputs the autopilot inputs containing the position of the drone
         * @return a Vector pointing from the drone to the way point
         */
        public Vector referenceHeadingVector(AutopilotInputs inputs){
            //first get the position of the drone
            Vector dronePosition = Controller.extractPosition(inputs);
            //then get the way point position
            Vector wayPointPos = this.getPosition();

            //calculate the vector that points from the drone to the way point
            Vector droneToWayPoint = wayPointPos.vectorDifference(dronePosition);

            return droneToWayPoint;
        }

        /**
         * Calculates the distance between the way point and the drone
         * @param dronePosition the position of the drone
         * @return the distance between the drone and the way point
         */
        private float distanceToWayPoint(Vector dronePosition){
            return dronePosition.distanceBetween(this.getPosition());
        }

        /**
         * Returns true if the way point has been reached for the given current radius
         * @param inputs the current inputs provided by the testbed
         * @return true if and only if the distance between the drone and the way point is smaller than or equal
         * to the acceptance radius
         */
        public boolean isReached(AutopilotInputs inputs){
            return getPosition().distanceBetween(Controller.extractPosition(inputs)) <= getAcceptanceRadius();
        }

        private Vector getPosition() {
            return position;
        }

        private float getAcceptanceRadius() {
            return acceptanceRadius;
        }

        public void setPosition(Vector position) {
            this.position = position;
        }

        /**
         *
         * @param inputs
         */
        public void setVisited(AutopilotInputs inputs){
            if(isReached(inputs)){
                this.isVisited = true;
            }
        }

        private boolean isVisited = false;
        private Vector position;
        private float acceptanceRadius;

        @Override
        public String toString() {
            return "WayPoint{" +
                    //"isVisited=" + isVisited +
                    ", pos: " + position +
                    ", acceptanceRadius: " + acceptanceRadius +
                    '}';
        }
    }

}


//        //first get the maximal thrust:
//        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
//        //get the approx of the velocity
//        //first get the latest inputs
//        AutopilotInputs currentInputs = this.getCurrentInputs();
//        AutopilotInputs prevInputs = this.getPreviousInputs();
//
//
//        //get the velocity approx (TODO change to PID if concept works);
//        //prepare the variables for calculation
//        float prevThrust = this.getPrevThrust();
//        //System.out.println("PrevThrust: " + prevThrust);
//        Vector orientation = Controller.extractOrientation(currentInputs);
//        Vector rotationApprox = this.getRotationApprox(prevInputs, currentInputs);
//        Vector position = Controller.extractPosition(currentInputs);
//        Vector velocityApprox = this.getVelocityApprox(prevInputs, currentInputs);
//        float deltaTime = this.getDeltaTime(prevInputs, currentInputs);
////        System.out.println("deltaTime: " + deltaTime);
////        System.out.println("rotation approx: " + rotationApprox);
////        System.out.println("velocity approx: " + velocityApprox);
//        //calculate the resulting force on the drone
//        PhysXEngine engine = this.getAutopilot().getPhysXEngine();
//        Vector netWorldForce = engine.getTotalExternalForcesWorld(prevThrust,orientation, rotationApprox, position, velocityApprox, deltaTime);
//        //our goal is a net zero in the y-direction (setpoint = 0)
//        //extract the y-force
//        float netYForce = netWorldForce.getyValue();
//
//
//        //feed it into the controller
//        PIDController thrustPID = this.getThrustPID();
//        float pidResult = thrustPID.getPIDOutput(netYForce, currentInputs.getElapsedTime());
//        //System.out.println("Thrust PID result: " + pidResult);
//        //if the result is negative, too less lift was present to sustain stable flight, add thrust
//        //if to much lift, subtract
//
//        float newThrust = prevThrust + pidResult;//signum(netYForce)*50f; //negative because a neg error means too little lift
//        System.out.println("Thrust PID: " + pidResult);
//        //check if max is not exceeded or not below zero:
//        newThrust = max(min(maxThrust, newThrust),0);
//
//        //save & set the thrust
//        this.setPrevThrust(newThrust);
//        System.out.println("Thrust: " + newThrust);
//        outputs.setThrust(1000f);
//        // we are (finally) done;
