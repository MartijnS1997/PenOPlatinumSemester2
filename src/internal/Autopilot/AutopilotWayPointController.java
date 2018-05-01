package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

//todo do more efficient way point control
/**
 * Created by Martijn on 13/03/2018.
 * A class of way point controllers, these controllers fly based on auto generated
 * way points to return to the flight base
 */
@Deprecated
public class AutopilotWayPointController {

    /**
     * Constructor for a way point controller
     * @param autopilot the autopilot the controller is associated with
     */
    public AutopilotWayPointController(AutoPilot autopilot){
        // implement constructor
        //super(autopilot);
//        this.pathGenerator = new dummyWayPointGenerator(1100, new Vector(0, 30, 0), 1200, 0.1f);
    }

//        //add some way points to test against
////        List<WayPoint> wayPoints = this.getWayPointPath();
////        wayPoints.add(new WayPoint(new Vector(5,20f, -60f), 20));
////        wayPoints.add(new WayPoint(new Vector(10,20f, -120f), 20));
////        wayPoints.add(new WayPoint(new Vector(15,20f, -180f), 20));
////        int nbwaypoints = 50;
////        for(int i = 1; i < nbwaypoints; i++){
//////            float turningRadius = 1500;
//////            float x = (float) (-turningRadius*cos(PI/nbwaypoints * i) + turningRadius);
//////            float z = (float) (-sin(PI/nbwaypoints*i)*turningRadius);
////
////            wayPoints.add(new WayPoint(new Vector(0, 30, -i*60), 30, 500));
////        }
////
////        this.setWayPointPath(wayPoints);
//
//
//
////    public void setLandingPath(Vector destination){
////        //System.out.println("Setting destination: ################################################################################");
////        //get the current inputs
////        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
////        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
////
////        //get the position and the velocity
////        Vector currentPos = Controller.extractPosition(currentInputs);
////        Vector currentVelocity =this.getVelocityApprox(prevInputs, currentInputs);
////
////        //check if the current velocity is zero (means we have init on dummy)
////        currentVelocity = currentVelocity.getSize() == 0 ?
////                PhysXEngine.droneOnWorld(new Vector(0,0, -DUMMY_APPROX_VEL), Controller.extractOrientation(currentInputs)) //if so, set to balance velocity
////                : currentVelocity; //if not, keep the approx
////        //generate the path
////        PathGenerator pathGen = new PathGenerator();
////        pathGen.generateLandingPath(currentPos, currentVelocity, destination);
////        List<Vector> path = pathGen.getPath();
////        //map the path elems to way points
////        List<WayPoint> wayPointPath = path.stream().map(pathElem -> new WayPoint(pathElem)).collect(Collectors.toList());
////        //configure the resulting way points
////        //get the average distance between the points to set an ignore radius
//////        float avgDistance = (float) (IntStream.range(0, path.size()-1).asDoubleStream()
//////                .map(i -> path.get((int) i).distanceBetween(path.get((int) (i+1))))
//////                .sum())/path.size();
//////        float acceptRatio = 0.15f; //15% of average distance for acceptance
//////        float ignoreRatio = 3.f; //300% of average distance for ignore
//////        //now convert to way points
//////        List<WayPoint> wayPointPath = path.stream()
//////                .map(p-> new WayPoint(p, avgDistance*acceptRatio, avgDistance*ignoreRatio))
//////                .collect(Collectors.toList());
////        //assign the path
////        this.setWayPointPath(wayPointPath);
////
////    }
////
//
//    /**
//     * The controller has reached its objective if all the way points are passed
//     * @param currentInputs the current inputs (this is the base of the check)
//     * @param previousInputs
//     * @return
//     */
//    @Override
//    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
//        //get the way point flag:
//         return this.hasReachedFinalWayPoint();
//    }
//
//    /**
//     * Sets the control actions for the thrust of the autopilot
//     * @param outputs the outputs-object generated by the autopilot
//     */
//    private void thrustControl(Controller.ControlOutputs outputs){
//
//
//        //first get the maximum thrust
//        float maxThrust =  this.getConfig().getMaxThrust();
//        //also get the current inputs
//        AutopilotInputs_v2 inputs = null; //this.getCurrentInputs();
//        //extract the orientation
//        Vector orientation = Controller.extractOrientation(inputs);
//        //extract the position
//        Vector position = Controller.extractPosition(inputs);
//        //get the current way point
//        WayPoint wayPoint = this.getCurrentWayPoint();
//
//        //System.out.println("WayPoint: " + wayPoint);
//        //get the pitch difference
//        float refPitch = this.getPitchDifference();
//
//        //get the distance to the target
//        float refDist = wayPoint.distanceToWayPoint(position);
//        float minRefDist = 20;
//        //System.out.println("reference pitch" + refPitch);
//        //the thrust should be stable if target is faraway, and higher if close
//        float outputThrust = (float) (BASE_THRUST + 10*THRUST_COEFF*refPitch*RAD2DEGREE );//+ THRUST_COEFF/20*pow(orientation.getzValue()*RAD2DEGREE,1));
//
//        outputs.setThrust(max(min(outputThrust, maxThrust), 0));
//        //System.out.println("thrust " + outputs.getThrust());
//    }
//
//
//
//    /**
//     * Control actions for the pitch of the drone (influences the horizontal stabilizer position)
//     * @param outputs the outputs object generated by the autopilot and sent to the drone
//     */
//    private void pitchControl(Controller.ControlOutputs outputs){
//        //first get the current way point
//        WayPoint wayPoint = this.getCurrentWayPoint();
//
//        //we need the current inputs
//        AutopilotInputs_v2 currentInputs = null;// this.getCurrentInputs();
//        AutopilotInputs_v2 prevInputs = null;//this.getPreviousInputs();
//
//        //calculate the pitch difference
//        float pidInput = this.getPitchDifference();
//        //get the time difference
//        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
//        //get the controller used to control the pitch
//        Controller.PIDController pitchController = this.getPitchPID();
//        //get the PID output of the controller
//        float pidResult = pitchController.getPIDOutput(pidInput, deltaTime) /*pitchController.getPIDOutputSetPoint(currentPitch, inputs.getElapsedTime(), referencePitch)*/;
////        System.out.println("PID input: " + pidInput);
////        System.out.println("PID output: " + pidResult);
//        //if the pid result is positive: the set point is larger than the input -> lower the pitch (pos inclination)
//        //if the pid result is negative: the set point is smaller than the input -> increase the pitch (neg inclination)
//        float horizontalInclination = /*this.getStabilizerStable() */+ pidResult; //todo change the constants for better result
//        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HORIZONTAL_CAP_INCLINATION);
//        outputs.setHorStabInclination(horizontalInclination);
//    }
//
//    /**
//     * Generates control actions for the bank of the drone
//     * @param outputs the outputs of the controller containing the control commands for the drone
//     *                they may be modified by this method
//     */
//    private void bankControl(Controller.ControlOutputs outputs){
//        //get the current way point
//        WayPoint wayPoint = this.getCurrentWayPoint();
//        //then get the current inputs
//        AutopilotInputs_v2 currentInputs = null;//this.getCurrentInputs();
//        AutopilotInputs_v2 prevInputs = null;//this.getPreviousInputs();
//        //extract the current orientation:
//        Vector orientation = Controller.extractOrientation(currentInputs);
//        //extract the position
//        Vector position = Controller.extractPosition(currentInputs);
//        System.out.println("Current position: " + position);
//        //extract the elapsed time
//        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
//        //get the heading PID
//        Controller.PIDController bankControl = this.getBankPID();
//        //get the output from the PID
//        float pidInput = this.getHeadingDifference();
//        System.out.println("Heading difference: (l = +, r = -) " + pidInput*RAD2DEGREE);
//        //float pidOutput = bankControl.getPIDOutput(pidInput, elapsedTime);
//        float pidOutput = bankControl.getPIDOutput(pidInput, deltaTime);
//
//        //System.out.println( "input sign: " + pidInput);
//        //and inclining the main wings
//        float mainLeftWingInclination = /*this.getMainStable()*/ + pidOutput;
//        float mainRightWingInclination =/* this.getMainStable()*/ - pidOutput;
//
//        //put some extra umpf into the horizontal stabilizer
//        float surplusHor = outputs.getHorStabInclination()  + BANK_HOR_STAB_EXTRA;
//        outputs.setHorStabInclination(signum(surplusHor)*min(HORIZONTAL_CAP_INCLINATION, abs(surplusHor)));
//
//        outputs.setLeftWingInclination(capMainWingInclination(mainLeftWingInclination));
//        outputs.setRightWingInclination(capMainWingInclination(mainRightWingInclination));
//
//    }
//
//
//    /**
//     * Enforces a maximum onto the main wing inclination based on the MAIN_CAP_DELTA_INCLINATION variable
//     * @param inclination the inclination to adjust
//     * @return if the inclination is in the range getMainStable +- MAIN_CAP_DELTA_INCLINATION
//     *         the same value as the inclination is returned, if it exceeds the border value, the inclination
//     *         is set to the border value
//     */
//    //note: replace with main class call cap inclination
//    private float capMainWingInclination(float inclination){
////        //first determine the lower cap:
////        float lowerCap = this.getMainStable() - MAIN_CAP_DELTA_INCLINATION;
////        float upperCap = this.getMainStable() + MAIN_CAP_DELTA_INCLINATION;
////        if(inclination < lowerCap){
////            return lowerCap;
////        }
////        if(inclination > upperCap){
////            return upperCap;
////        }
////        return inclination;
//        return 0;
//    }
//
//    /**
//     * Generates the control actions for the autopilot
//     * @param currentInputs the inputs of the autopilot
//     * @return the control actions
//     */
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
//        //System.out.println("Current position: " + Controller.extractPosition(inputs));
//        //this.updateInputs(inputs);
//        //create a new output object
//        Controller.ControlOutputs outputs = new Controller.ControlOutputs();
//        this.wayPointControl();
//        //System.out.println("Current way point: " + this.getCurrentWayPoint());
//        //then issue control actions
//        //get the control actions for the thrust
//        thrustControl(outputs);
//        //get the control actions for the pitch
//        pitchControl(outputs);
//        //get the control actions for the roll
//        bankControl(outputs);
//        //roll control
//        //rollControl(outputs, currentInputs);
//
//        //check if everything is within allowed parameters
//        //angleOfAttackControl(outputs, currentInputs, previousInputs);
//        System.out.println(outputs);
//        System.out.println();
//        return outputs;
//    }
//
//    /**
//     * Checks if the current way point has been reached, if so increments the way point index
//     */
//    private void wayPointControl(){
//
//        PathGenerator pathGenerator = this.getPathGenerator();
//        WayPoint currentWayPoint = this.getCurrentWayPoint();
//        AutopilotInputs_v2 currentInputs = null; //this.getCurrentInputs();
//        AutopilotInputs_v2 prevInputs = null; //this.getPreviousInputs();
//
//        System.out.println(this.getCurrentWayPoint());
//        if(currentWayPoint == null){
//            //TODO DELETE THE DUMMY AFTER THE TESTS
//
//            initWayPoint(pathGenerator, currentInputs, prevInputs);
//            //we're finished for now
//            System.out.println("Current way point: " + this.getCurrentWayPoint());
//            return;
//        }
//
//        //check if we've reached the current way point
//        if(currentWayPoint.isReached(currentInputs)){
//            configureNextWayPointSuccess(pathGenerator, currentInputs, prevInputs);
//            System.out.println("Current way point: " + this.getCurrentWayPoint());
//        //otherwise check if we have passed the current way point
//        }else if(!currentWayPoint.reachable(currentInputs)){
//            configureNextWayPointMissed(pathGenerator, currentInputs, prevInputs);
//            System.out.println("Current way point: " + this.getCurrentWayPoint());
//        }
//        //fall of the edge if not valid
//    }
////        if(currReached||currPassed){
////            //if so, activate the next way point
////            this.toNextWayPoint();
////            currentWayPoint = this.getCurrentWayPoint();
////            //check if we've reached the final way point
////            if(currentWayPoint == null){
////                //if so, throw simulation ended
////                this.setHasReachedFinalWayPoint();
////            }
////        }
//
//
//    private void configureNextWayPointMissed(PathGenerator pathGenerator, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
//        Vector velocityApprox = this.getVelocityApprox(prevInputs, currentInputs);
//        Vector position = Controller.extractPosition(currentInputs);
//        Vector destination = this.getDestination();
//        Vector nextWayPointPos = pathGenerator.getNextWaypointMissed(position, velocityApprox, destination);
//        WayPoint nextWayPoint = new WayPoint(nextWayPointPos);
//        this.setCurrentWayPoint(nextWayPoint);
//    }
//
//    private void configureNextWayPointSuccess(PathGenerator pathGenerator, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
//        Vector velocityApprox = this.getVelocityApprox(prevInputs, currentInputs);
//        Vector position = Controller.extractPosition(currentInputs);
//        Vector destination = this.getDestination();
//        Vector nextWayPointPos = pathGenerator.getNextWaypointSuccess(position, velocityApprox, destination);
//        WayPoint nextWayPoint = new WayPoint(nextWayPointPos);
//        this.setCurrentWayPoint(nextWayPoint);
//    }
//
//    private void initWayPoint(PathGenerator pathGenerator, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
//        //get the velocity approx
//        Vector velocityApprox = this.getVelocityApprox(prevInputs, currentInputs);
//        //TODO delete the velocity approx after cleaned up
//        System.out.println("reminder delete the velocity approx dummy after configuring - AutopilotWaypoint line 309");
//        velocityApprox = new Vector(0,0,-50);
//        Vector position = Controller.extractPosition(currentInputs);
//        Vector wayPointPos =  pathGenerator.getNextWaypoint(position, velocityApprox, this.getDestination());
//        WayPoint nextWayPoint = new WayPoint(wayPointPos);
//        this.setCurrentWayPoint(nextWayPoint);
//    }
//
//    @Override
//    protected void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInput, float rollThreshold){
//        float roll = currentInput.getRoll();
//
//        if(roll >= rollThreshold&&isSteeringLeft(outputs)){
//            outputs.setRightWingInclination(0/* this.getMainStable()*/);
//            outputs.setLeftWingInclination(0 /*this.getMainStable()*/);
//        }
//        else if(roll <= - rollThreshold&&isSteeringRight(outputs)){
//            outputs.setLeftWingInclination(0 /*this.getMainStable()*/);
//            outputs.setRightWingInclination(0 /*this.getMainStable()*/);
//
//        }else{
//            // change nothing
//        }
//    }
//
//
//    /**
//     * Checks if the current output steers right
//     * @param outputs the outputs that are generated by the controller
//     * @return true if the drone is steering right
//     */
//    private boolean isSteeringRight(Controller.ControlOutputs outputs){
//        return false; //outputs.getRightWingInclination() < this.getMainStable();
//    }
//
//    /**
//     * Checks if the current output steers left
//     * @param outputs the outputs that are generated by the controller
//     * @return true if the drone is steering left
//     */
//    private boolean isSteeringLeft(Controller.ControlOutputs outputs){
//        return false; //outputs.getRightWingInclination() > this.getMainStable();
//    }
//
//    /**
//     * Calculates the angle between the next way point and the current heading vector of the drone (0,0,-1) @drone axis
//     * and also indicates the relative direction in which the drone needs to fly (contained within the sign)
//     * @return the heading difference calculated as the angle between the orthogonal projection of the heading
//     * vector and the reference vector (vector between current position and the next way point) on the xz plane
//     * the direction sign is determined by the cross product of the heading vector with the reference (negative is right, positive is left)
//     */
//    private float getHeadingDifference(){
//        AutopilotInputs_v2 inputs = null; //this.getCurrentInputs();
//        //get the orientation of the drone
//        Vector orientation = Controller.extractOrientation(inputs);
//        System.out.println("Current Orientation: " + orientation.scalarMult(RAD2DEGREE));
//        //then calculate the heading vector in the world axis system (the negative z-unit vector in the drone axis system)
//        Vector headingVectorDrone = new Vector(0,0,-1);
//        Vector headingVectorWorld = PhysXEngine.droneOnWorld(headingVectorDrone, orientation);
//        System.out.println("heading vector  world :" + headingVectorWorld );
//        //do an orthogonal projection of the heading vector on the XZ-plane
//        //the normal unit vector
//        Vector normal = new Vector(0, 1, 0);
//        //project the heading vector onto the plane
//        Vector headingVectorProjection = headingVectorWorld.orthogonalProjection(normal);
//
//        System.out.println("Heading vector projected: " + headingVectorProjection);
//
//        //get the current way point
//        WayPoint currentWayPoint = this.getCurrentWayPoint();
//        //then get the reference heading vector
//        Vector refHeadingVector = currentWayPoint.referenceHeadingVector(inputs);
//        System.out.println("RefHeadingVector in world axis: " + refHeadingVector.normalizeVector());
//        //project it on the xz-plane
//        Vector refHeadingProjection = refHeadingVector.orthogonalProjection(normal);
//        System.out.println("RefHeadingVector projected on XZ: " + refHeadingProjection.normalizeVector());
//
//        //do a cross product to figure out the direction in which to fly
//        Vector crossProduct = headingVectorProjection.crossProduct(refHeadingProjection);
//        System.out.println("Cross product, direction: " + crossProduct.normalizeVector());
//        //only keep the sign of the y-value
//        float direction = signum(crossProduct.getyValue());
//        float angleBetween = headingVectorProjection.getAngleBetween(refHeadingProjection);
//        System.out.println("the angle: " + angleBetween);
//        float steeringAngle = direction*angleBetween;
//
//        //check if Nan (can happen if the steering angle is "perfect")
//        if(Float.isNaN(steeringAngle)){
//
//            return 0f;
//        }
//
//        return steeringAngle;
//
//    }
//
//    /**
//     * Calculates the pitch difference, the angle between the reference vector (the vector between
//     * the current way point and the position of the drone) transformed to the drone axis system and
//     * projected onto the yz plane in the drone axis system, and the heading vector (0,0,-1) in the drone axis system.
//     * The direction is determined by the cross product of the heading vector and the projected and transformed reference
//     * vector.
//     * @return returns a pos angle if the drone needs to go up, and a negative angle if the drone needs to go down
//     */
//    private float getPitchDifference(){
//        AutopilotInputs_v2 inputs = null;//this.getCurrentInputs();
//        //get the difference vector
//        //first get the current way point
//        WayPoint currWayPoint = this.getCurrentWayPoint();
//        //get the position of the drone
//        Vector dronePos = Controller.extractPosition(inputs);
//        //get the position of the way point
//        Vector wayPointPos = currWayPoint.getPosition();
//        //get the ref vector
//        Vector ref = wayPointPos.vectorDifference(dronePos);
//        //transform it to the drone axis system
//        //first get the current orientation
//        Vector orientation = Controller.extractOrientation(inputs);
//        Vector refDrone = PhysXEngine.worldOnDrone(ref, orientation);
//        //then project it onto the yz plane: normal vector (1,0,0)
//        Vector normalYZ = new Vector(1,0,0);
//        Vector projRefDrone = refDrone.orthogonalProjection(normalYZ);
//        //calculate the angle between the heading vector (0,0,-1) and the reference
//        Vector headingVect = new Vector(0,0,-1);
//        float angle = abs(projRefDrone.getAngleBetween(headingVect));
//
//        //then get the vector product for the direction(the x-component)
//        float direction = headingVect.crossProduct(projRefDrone).getxValue();
//
//        float res = angle*signum(direction);
//        //check for NaN
//        if(Float.isNaN(res)){
//            return 0; // NaN comes from the angle
//        }
//        //else return the result
//        return res;
//
//    }
//
//    //TODO implement these methods accordingly
////    @Override
////    protected float getMainStable() {
////        return MAIN_STABLE_INCLINATION;
////    }
////
////    @Override
////    protected float getStabilizerStable() {
////        return HORIZONTAL_STABLE_INCLINATION;
////    }
////
////    @Override
////    protected float getRollThreshold() {
////        return ROLL_THRESHOLD;
////    }
////
////    @Override
////    protected float getInclinationAOAErrorMargin() {
////        return AOA_CALC_ERROR_MARGIN;
////    }
////
////    @Override
////    protected float getStandardThrust() {
////        return 0;
////    }
//
////    /**
////     * getter for the current way point
////     * @return the current way point
////     */
////    private WayPoint getCurrentWayPoint(){
////        int index = this.getCurrentWayPointIndex();
////        //get the wayPointPath
////        List<WayPoint> path = this.getWayPointPath();
////
////        //check if the wayPointPath size is equal to the index
////        if(path.size() == index){
////            //if so return null
////            return null;
////        }
////        // otherwise return the element at the current index
////        return path.get(index);
////    }
//
//    /**
//     * Getter for the final way point flag, the flag indicates if we've reached the final way point in the list
//     * @return true if the flag is active
//     */
//    private boolean hasReachedFinalWayPoint() {
//        return hasReachedFinalWayPoint;
//    }
//
//    /**
//     * Setter for the final way point flag, the flag indicates if we've reached the final way point in the list
//     * the flag can only be activated and not deactivated
//     */
//    private void setHasReachedFinalWayPoint() {
//        this.hasReachedFinalWayPoint = true;
//    }
//
////
////    /**
////     * increments the way point index
////     */
////    private void toNextWayPoint(){
////        this.currentWayPointIndex ++;
////    }
//
////    /**
////     * Getter for the current way point index, the index that indicates which way point
////     * we need to reach in the waypoint list
////     * @return the index of the current way point
////     */
////    private int getCurrentWayPointIndex() {
////        return currentWayPointIndex;
////    }
//
////    /**
////     * Getter for the WayPointPath, the path containing all the way points to follow
////     * @return a list containing all the way points to pass during a flight
////     */
////    private List<WayPoint> getWayPointPath() {
////        return wayPointPath;
////    }
//
////    /**
////     * Setter for the wayPointPath
////     * @param wayPointPath the path of way points to follow
////     */
////    private void setWayPointPath(List<WayPoint> wayPointPath){
////        this.wayPointPath = wayPointPath;
////    }
//
//    /**
//     * Getter for the currently active waypoint
//     * @return
//     */
//    private WayPoint getCurrentWayPoint(){
//        return this.currentWayPoint;
//    }
//
//    /**
//     * Setter for the current way point
//     * @param currentWayPoint the current way point to reach
//     */
//    private void setCurrentWayPoint(WayPoint currentWayPoint) {
//        this.currentWayPoint = currentWayPoint;
//    }
//
//    /**
//     * The generator used to generate the path to follow
//     * @return the generator
//     */
//    private PathGenerator getPathGenerator() {
//        return pathGenerator;
//    }
//
//    /**
//     * Getter for the destination of the autopilot
//     * @return the destination for the wayPoint controller
//     */
//    public Vector getDestination() {
//        return destination;
//    }
//
//    /**
//     * Setter for the destination of the way point controller
//     * @param destination the destination of the way point controller
//     */
//    public void setDestination(Vector destination) {
//        this.destination = destination;
//    }
//
//    /**
//     * Getter for the PID controller that controls the pitch of the drone
//     * @return the controller that controls the pitch of the drone
//     */
//    private Controller.PIDController getPitchPID() {
//        return pitchPID;
//    }
//
//
//    /**
//     * Getter for the bank PID controller (controls the banking inclination for the drone
//     * @return the banking controller
//     */
//    private Controller.PIDController getBankPID() {
//        return bankPID;
//    }
//
//    private final static float PITCH_GAIN = 1.0f;
//    private final static float PITCH_DERIVATIVE = 0.2f;
//    private final static float PITCH_INTEGRAL = 0.5f;
//
//    private Controller.PIDController pitchPID = new Controller.PIDController(PITCH_GAIN,  PITCH_INTEGRAL, PITCH_DERIVATIVE);
//
//    private final static float BANK_GAIN = 0.8f;
//    private final static float BANK_DERIVATIVE = 0.4f ;
//    private final static float BANK_INTEGRAL = 0.0f ;
//    private final static float BANK_HOR_STAB_EXTRA = (float) (-3*PI/180);
//
//    private Controller.PIDController bankPID = new Controller.PIDController(BANK_GAIN, BANK_INTEGRAL, BANK_DERIVATIVE);
//
//    private final static float BASE_THRUST = 550f;
//    private final static float THRUST_COEFF = 50;
//
//    /**
//     * The way point the autopilot is currently navigating to
//     */
//    private WayPoint currentWayPoint = null;
//
//    /**
//     * The path generator used to generate the path the controller needs to follow
//     */
//    private PathGenerator pathGenerator;
//
//    /**
//     * The variable that stores the destination
//     */
//    private Vector destination = new Vector(10,30,0);
//
////    /**
////     * List that stores the current wayPointPath
////     */
////    private List<WayPoint> wayPointPath = new ArrayList<>();
////
////    /**
////     * The current index for the way point
////     */
////    private int currentWayPointIndex = 0;
//
//    /**
//     * Flag that indicates if the final way point has been reached
//     */
//    private boolean hasReachedFinalWayPoint = false;
//
//    private final static float MAIN_STABLE_INCLINATION = (float) (5*PI/180);
//    private final static float HORIZONTAL_STABLE_INCLINATION = 0f;
//    private final static float ROLL_THRESHOLD = (float) (5*PI/180);
//    private final static float AOA_CALC_ERROR_MARGIN = (float) (2*PI/180);
//    private final static float HORIZONTAL_CAP_INCLINATION = (float) (10*PI/180);
//    private final static float MAIN_CAP_DELTA_INCLINATION = (float) (3*PI/180);
//    private final static float RAD2DEGREE = (float) (180/PI);
//    private final static float DUMMY_APPROX_VEL = 50f;
//
//    /**
//     * A class of way points used in navigation and PID control
//     */
//    private class WayPoint{
//
//
//        WayPoint(Vector position, float acceptanceRadius){
//            this.position = position;
//            this.acceptanceRadius = acceptanceRadius;
//        }
//        /**
//         * Constructor for a way point without the acceptance radius, the acceptance radius
//         * is set to a default value STANDARD_ACCEPTANCE_RADIUS
//         * @param position the position of the way point
//         */
//        WayPoint(Vector position){
//            this(position, STANDARD_ACCEPTANCE_RADIUS);
//        }
//
////        /**
////         * Calculates the pitch used as the reference for the autopilot
////         * @param inputs the inputs of the autopilot generated by the testbed (containing the necessary data)
////         * @return the pitch to the way point
////         */
////        private float referencePitch(AutopilotInputs_v2 inputs){
////            //first get the position of the drone
////            Vector dronePos = Controller.extractPosition(inputs);
////            //then get the distance between the drone and way point (used as the hypotenuse)
////            float hypotenuse = this.distanceToWayPoint(dronePos);
////            //then get the height difference (used for the opposite side)
////            float oppositeSide = this.getPosition().getyValue() - dronePos.getyValue();
////            //then get the reference pitch by arcSin(opposite/hypotenuse)
////            float pitch = (float) asin(oppositeSide/hypotenuse);
////
////            return pitch;
////        }
//
//
//
//        /**
//         * Calculates the vector that points from the drone to the selected way point
//         * @param inputs the autopilot inputs containing the position of the drone
//         * @return a Vector pointing from the drone to the way point
//         */
//        private Vector referenceHeadingVector(AutopilotInputs_v2 inputs){
//            //first get the position of the drone
//            Vector dronePosition = Controller.extractPosition(inputs);
//            //then get the way point position
//            Vector wayPointPos = this.getPosition();
//
//            //calculate the vector that points from the drone to the way point
//            Vector droneToWayPoint = wayPointPos.vectorDifference(dronePosition);
//
//            return droneToWayPoint;
//        }
//
//        /**
//         * Calculates the distance between the way point and the drone
//         * @param dronePosition the position of the drone
//         * @return the distance between the drone and the way point
//         */
//        private float distanceToWayPoint(Vector dronePosition){
//            return dronePosition.distanceBetween(this.getPosition());
//        }
//
//        /**
//         * Returns true if the way point has been reached for the given current radius
//         * @param inputs the current inputs provided by the testbed
//         * @return true if and only if the distance between the drone and the way point is smaller than or equal
//         * to the acceptance radius
//         */
//        private boolean isReached(AutopilotInputs_v2 inputs){
//            return getPosition().distanceBetween(Controller.extractPosition(inputs)) <= getAcceptanceRadius();
//        }
//
//        /**
//         * Returns true if the way point is in front of the drone
//         * @param inputs the current autopilot inputs (needed for the orientation and the position)
//         * @return true the current way point is in front of the drone
//         */
//        private boolean reachable(AutopilotInputs_v2 inputs){
//
//            Vector dronePosition = Controller.extractPosition(inputs);
//            Vector droneOrientation = Controller.extractOrientation(inputs);
//
//            Vector wayPointPosition = this.getPosition();
//            // take the difference
//            Vector diffVectorWorld = wayPointPosition.vectorDifference(dronePosition);
//            //transform it onto the drone axis system
//            Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, droneOrientation);
//            //then project the difference vector on the heading vector
//            Vector headingVector = new Vector(0,0,-1);
//            Vector projectedDiff = diffVectorDrone.projectOn(headingVector); //headingVector.projectOnVector(diffVectorDrone);
//            //check the scalar product of the projection, if positive we're behind, if not we're ahead
//            return projectedDiff.scalarProduct(headingVector) > 0;
//        }
//
//        /**
//         * Getter for the position of the way point in space
//         * @return the position of the way point
//         */
//        private Vector getPosition() {
//            return position;
//        }
//
//        /**
//         * Getter for the acceptance radius of the way point, the radius
//         * wherefore the drone needs to pass the way point before it is counted as a reach
//         * @return the acceptance radius for the way point
//         */
//        private float getAcceptanceRadius() {
//            return acceptanceRadius;
//        }
//
//
//        /**
//         * Setter for the position of the way point
//         * @param position the position to be set for the waypoint
//         */
//        public void setPosition(Vector position) {
//            this.position = position;
//        }
//
//
//        /**
//         * The position of the waypoint in space
//         */
//        private Vector position;
//
//        /**
//         * The radius which in the drone had to move to mark the way point as reached
//         */
//        private float acceptanceRadius;
//
//        private final static float STANDARD_ACCEPTANCE_RADIUS = 10f;
//
//        private final static float STANDARD_IGNORE_RADIUS = 200f;
//
//        @Override
//        public String toString() {
//            return "WayPoint{" +
//                    //"isVisited=" + isVisited +
//                    ", pos: " + position +
//                    ", acceptanceRadius: " + acceptanceRadius +
//                    '}';
//        }
//    }
//
//    private class dummyWayPointGenerator extends PathGenerator{
//
//        public dummyWayPointGenerator(float radius,Vector startPos, int nbPoints, float ascendPercentage) {
//            //generate a semi circle starting at the start Pos
//            //we want to ascend at at percentage of ascend percentage per block
//            float pathLen = (float) (radius*PI);
//            float totalAscend = ascendPercentage*pathLen;
//            float ascendPerBlock = totalAscend/nbPoints;
//
//            for(int index = 1; index != nbPoints+1; index ++){
//            float x = (float) (-radius*cos(PI/nbPoints * index) + radius);
//            float z = (float) (-sin(PI/nbPoints*index)*radius);
//            Vector wayPointPos = new Vector(startPos.getxValue() + x, startPos.getyValue()+ascendPerBlock*index, startPos.getzValue() + z);
//            wayPointList.add(wayPointPos);
//            }
//        }
//
//        @Override
//        public Vector getNextWaypoint(Vector position, Vector velocity, Vector destination) {
//            Vector nextWayPoint = wayPointList.get(wayPointIndex);
//            wayPointIndex++;
//            return nextWayPoint;
//        }
//
//        @Override
//        public Vector getNextWaypointMissed(Vector position, Vector velocity, Vector destination) {
//            return getNextWaypoint(position, velocity, AutopilotWayPointController.this.destination);
//        }
//
//        @Override
//        public Vector getNextWaypointSuccess(Vector position, Vector velocity, Vector destination) {
//            return getNextWaypoint(position, velocity, destination);
//        }
//
//        private List<Vector> wayPointList = new ArrayList<>();
//        private int wayPointIndex = 0;
//    }

}

