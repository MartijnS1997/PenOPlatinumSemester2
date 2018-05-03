package internal.Autopilot;

import AutopilotInterfaces.*;
import TestbedAutopilotInterface.Overseer.AutopilotDelivery;
import TestbedAutopilotInterface.Overseer.MapAirport;
import com.sun.istack.internal.Nullable;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

/**
 * Taxiing Controller
 * finite state machine for the controller
 * startTurn --> full brake --> taxiingToTarget --> full brake
 */
public class AutopilotTaxiingController extends Controller {

    public AutopilotTaxiingController(AutoPilot autopilot){
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //set the current inputs
        //this.updateInputs(inputs);
        //set the current outputs
        ControlOutputs outputs = new ControlOutputs(getStandardOutputs());

        getControlActionsActiveState(outputs,currentInputs,previousInputs);
        //maybe add AOA?


        return outputs;

    }

    @Override
    public void reset() {

    }

    /**
     * Setter for the next target of the taxiing controller
     * @param location the location to taxi to
     * @param orientation the direction in which the controller has to be oriented
     *                    if null the orientation doesn't matter
     */
    public void setTarget(Vector location, @Nullable Vector orientation ){
        this.target = location;
        if (orientation != null)
            this.wantedOrientation = orientation;
    }

    private void getControlActionsActiveState(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        TaxiingState state = this.getTaxiingState();
        System.out.println(state+"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        switch (state){
            case INIT_TURN:
                initialTurn(outputs, currentInputs, prevInputs);
                break;
            case FULL_BRAKE:
                fullBrake(outputs, currentInputs, prevInputs);
                break;
            case MOVING_TO_TARGET:
                moveToTarget(outputs, currentInputs, prevInputs);
                break;
        }
    }

    public void stayWithinAirportBorders(Vector[] airportBorders, ControlOutputs outputs, AutopilotInputs_v2 inputs){
        Vector topLeft = airportBorders[0];
        Vector topRight = airportBorders[1];
        Vector bottomLeft = airportBorders[2];
        Vector bottomRight = airportBorders[3];

        Vector position = Controller.extractPosition(inputs);
        float maxBrake = this.getConfig().getRMax();


        float x = position.getxValue();
        float z = position.getzValue();

        float x1 = topLeft.getxValue();
        float z1 = topLeft.getzValue();
        float x2 = bottomLeft.getxValue();
        float z2 = bottomLeft.getzValue();

        float x3 = topRight.getxValue();
        float z3 = topRight.getzValue();
        float x4 = bottomRight.getxValue();
        float z4 = bottomRight.getzValue();

        float d;

        //check on what side of a line A(x1,y1) & B(x2,y2) a point (x,y) is:
        //d = (x-x1)(y2-y1)-(y-y1)(x2-x1)
        //d < 0 = left, d > 0 right
        if (topLeft.getxValue() < topRight.getxValue()){
            d = (x-(x1+AIRPORT_BOUND_BUFFER))*(z2-z1) - (z-z1)*((x2+AIRPORT_BOUND_BUFFER)-(x1+AIRPORT_BOUND_BUFFER));
            if (d < 0){
                outputs.setRightBrakeForce(maxBrake);
            }

            d = (x-(x3-AIRPORT_BOUND_BUFFER))*(z4-z3) - (z-z3)*((x4-AIRPORT_BOUND_BUFFER)-(x3-AIRPORT_BOUND_BUFFER));
            if (d > 0){
                outputs.setLeftBrakeForce(maxBrake);
            }
        }
        else{
            d = (x-(x1-AIRPORT_BOUND_BUFFER))*(z2-z1) - (z-z1)*((x2-AIRPORT_BOUND_BUFFER)-(x1-AIRPORT_BOUND_BUFFER));
            if (d > 0){
                outputs.setRightBrakeForce(maxBrake);
            }

            d = (x-(x3+AIRPORT_BOUND_BUFFER))*(z4-z3) - (z-z3)*((x4+AIRPORT_BOUND_BUFFER)-(x3+AIRPORT_BOUND_BUFFER));
            if (d < 0){
                outputs.setLeftBrakeForce(maxBrake);
            }
        }
    }

    private final static float AIRPORT_BOUND_BUFFER = 2f;



    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        boolean reached = false;

        //if the wantedOrientation matters
        if (getWantedOrientation() != null) {
          //  if (targetReached(currentInputs) && checkHasFinishedTurn(abs(Controller.extractOrientation(currentInputs).getAngleBetween(getWantedOrientation())))) {

            if (targetReached(currentInputs) && orientationReached(currentInputs)) {
                reached = true;
            }
        }
        //if the wantedOrientation doesn't matter and is equal to null
        else{
            if (targetReached(currentInputs)){
                reached = true;
            }
        }
//        System.out.println("angle " + Controller.extractOrientation(currentInputs).getAngleBetween(getWantedOrientation()));
//        System.out.println("wanted " + getWantedOrientation());
//        System.out.println("current " + Controller.extractOrientation(currentInputs));
//        System.out.println(reached);

        return reached;

    }



    /*
    Initial turn constants
     */

    /**
     * The angle between the heading of the drone and the vector between the drone and the target
     * that has to be reached before the full brake may be initialized
     */
    private final static float MAX_ANGLE_ERROR = (float) (2*PI/180);
    private final static float THRUST_RATIO = 0.125f;

    /**
     * Getter for the control actions for the phase where we take out initial turn
     * @param outputs the outputs to write the control actions to
     */
    private void initialTurn(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){

        Vector target = this.getTarget();
        //AutopilotInputs_v2 inputs = null;// this.getCurrentInputs();
        float maxBrake = this.getConfig().getRMax();
        float maxThrust = this.getConfig().getMaxThrust();
        //calculate the angle between the heading vector and the target vector
        float angle = angleToTarget(target, currentInputs);
//        System.out.println("Angle between heading and diff: " + angle);
//        System.out.println();
        //check if we've reached our target
        if(checkHasFinishedTurn(angle)){
        //if(orientationReached(currentInputs)){
            //invoke the next controller
            this.setTaxiingState(TaxiingState.FULL_BRAKE);
            this.fullBrake(outputs, currentInputs, prevInputs);
            return;
        }
        //if not, continue normal procedure

        // if the angle is negative we need to steer to the right if it is positive we need to steer to the left
        // to go left we should activate the left brake, to go right activate the right brake
        if(angle > 0){
            //we need to go to the left
            outputs.setLeftBrakeForce(maxBrake);
        }else{
            outputs.setRightBrakeForce(maxBrake);
        }
        //keep a steady and slow output a minimal thrust (1/8th of max)
        outputs.setThrust(maxThrust*THRUST_RATIO);

        //Todo set the brake forces accordingly
    }

    /**
     * Checks if the drone has finished its initial turn
     * @param angle the angle between the drone heading vector and the vector from the drone to the target
     */
    private boolean checkHasFinishedTurn(float angle){
        //the angle between the heading vector of the drone and the target is small enough, we may go to the next state
        //the full brake state
        if(MAX_ANGLE_ERROR >= abs(angle)){
            return true;
        }
        //else do nothing
        return false;
    }

    /*
    Full brake constants
     */
    private final static float BRAKE_THRUST = 0f;
    private final static float STANDSTILL_VELOCITY = 0.1f;

    /**
     * The controller used to brake to our full capabilities... nothing special, just braking
     * @param outputs the outputs to write our control actions to
     */
    private void fullBrake(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
//        System.out.println("Entered full brake ##########");
        //we just brake to the full extent
        float maxBrake = this.getConfig().getRMax();
        //check if we've come to a standstill && reached the target
        if(droneInStandstill(currentInputs, prevInputs) && targetReached( currentInputs /*this.getCurrentInputs()*/)){
            //do not change the inputs, we're finished
            return;
        }


        //if we've only came to a standstill, but we didn't reach anything, call the move controller
        if(droneInStandstill(currentInputs, prevInputs)){
            this.setTaxiingState(TaxiingState.MOVING_TO_TARGET);
            //invoke the moving to target controller
            this.moveToTarget(outputs, currentInputs, prevInputs);
            return;
        }
        //doing full brake
        //otherwise, continue with full brake
        outputs.setRightBrakeForce(maxBrake);
        outputs.setLeftBrakeForce(maxBrake);
        outputs.setFrontBrakeForce(maxBrake);
        outputs.setThrust(BRAKE_THRUST);
    }


    /**
     * Checks if the drone came to an (approximate) standstill
     * @return true if the total velocity of the drone is <= STANDSTILL_VELOCITY
     */
    private boolean droneInStandstill(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        //AutopilotInputs_v2 currentInputs = null; //this.getCurrentInputs();
        //AutopilotInputs_v2 prevInputs = null; //this.getPreviousInputs();

        //approx the velocity
        Vector approxVelocity = getVelocityApprox(prevInputs, currentInputs);
        //check if the size exceeds the minimum
        boolean hasReachedMinimum = approxVelocity.getSize() <= STANDSTILL_VELOCITY;
        if(hasReachedMinimum){
            return true;
        }

        //if not return false
        return false;
    }

    /*
  Moving to target constants
   */

    // reference is 5m/s
    public final static float REACHING_DISTANCE = 5f; // the distance we need to enter before we can call it a day
    // also added a bit of padding for the brakes

    private void moveToTarget(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){

        //acquire the current inputs (needed for angle calculation)
        //AutopilotInputs_v2 currentInputs = null; //this.getCurrentInputs();
        //AutopilotInputs_v2 prevInputs = null; // this.getPreviousInputs();
        //check if we are done with moving
       // System.out.println("TARGET: " + targetReached(currentInputs));
        if(targetReached(currentInputs)){
            //if so, change the state and call the brake method, return afterwards
            this.setTaxiingState(TaxiingState.FULL_BRAKE);
            this.fullBrake(outputs, currentInputs, prevInputs);
            return;
        }
//        System.out.println("position: " + Controller.extractPosition(currentInputs));
        brakeTurningControls(outputs, currentInputs, prevInputs);
        cruiseControl(outputs, prevInputs,currentInputs);

        //the borders of the Airport we're currently in
        Vector[] airportBorders = getAutopilot().getCommunicator().getAirportAtCurrentLocation().getAirportBorders();
        //make sure you always stay within the borders of the Airport
        stayWithinAirportBorders(airportBorders,outputs,currentInputs);


    }

    private boolean targetReached(AutopilotInputs_v2 inputs){
        //extract the current state
        //check if the current target has been rached, this happens if we've got close enough
        //to the target
        Vector target = this.getTarget();
        //get the current pos
        Vector position = Controller.extractPosition(inputs);

        return position.distanceBetween(target) <=  REACHING_DISTANCE;
    }

//
//    private void turnToCorrectOrientation(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
//
//        //check if we are done with turning
//        if(orientationReached(currentInputs)){
//            //if so, change the state and call the brake method, return afterwards
//            this.setTaxiingState(TaxiingState.FULL_BRAKE);
//            this.fullBrake(outputs, currentInputs, prevInputs);
//            return;
//        }
////        System.out.println("position: " + Controller.extractPosition(currentInputs));
//        brakeTurningControls(outputs, currentInputs, prevInputs);
//        ///cruiseControl(outputs, prevInputs,currentInputs);
//
//
//    }

    private boolean orientationReached(AutopilotInputs_v2 inputs){
        Vector orientation = Controller.extractOrientation(inputs);

       // System.out.println("=======================================");
//        System.out.println(getWantedOrientation());
//        System.out.println(orientation);
     //   System.out.println(orientation.getAngleBetween(getWantedOrientation()));

        //if the angle between the current orientation en the wanted orientation is smaller than the maxOrientationError, the correct orientation has been reached
        return Math.abs(orientation.getAngleBetween(getWantedOrientation())) <= maxOrientationError();
    }

    private Vector wantedOrientation;

    public Vector getWantedOrientation(){
        return wantedOrientation;
    }

    //cosinus(AirportWidth/AirportLength)
    private float maxOrientationError(){
        MapAirport currentAirport = this.getAutopilot().getCommunicator().getAirportAtCurrentLocation();


        return (float) Math.acos(currentAirport.getRunwayWidth()/currentAirport.getRunwayLength());
    }


    /**
     * taxi to the correct gate
     * @param inputs
     * @param packageToDeliver
     */
    public void gateTaxiing(AutopilotInputs_v2 inputs, AutopilotDelivery packageToDeliver){
        //airport to go to
        int goToAirport;
        //gate to go to
        int goToGate;
        //position to taxi to
        Vector taxiTo = new Vector();


     //   System.out.println("##########################################"+packageToDeliver);

        //if the package hasn't been picked up yet, the drone has to taxi to the correct gate
        if (!packageToDeliver.isPickedUp()){
            goToAirport = packageToDeliver.getSourceAirport();
            goToGate = packageToDeliver.getSourceAirportGate();

            //go to the correct gate to get the package
            if (goToGate == 0) taxiTo = getAutopilot().getCommunicator().getAirportByID(goToAirport).getGateZeroPosition();
            if (goToGate == 1) taxiTo = getAutopilot().getCommunicator().getAirportByID(goToAirport).getGateOnePosition();

        }

        //if the package has already been picked up, the drone has to taxi to the correct gate to drop the package off
        else{
            goToAirport = packageToDeliver.getDestinationAirport();
            goToGate = packageToDeliver.getDestinationAirportGate();

            //go to the correct destination
            if (goToGate == 0) taxiTo = getAutopilot().getCommunicator().getAirportByID(goToAirport).getGateZeroPosition();
            if (goToGate == 1) taxiTo = getAutopilot().getCommunicator().getAirportByID(goToAirport).getGateOnePosition();
        }

        //set the destination of where the drone has to go to, orientation doesn't matter
        setTarget(taxiTo, null);


    }

    /**
     * Taxi to the correct runway
     * @param inputs
     */
    public void runwayTaxiing(AutopilotInputs_v2 inputs){
        Vector taxiTo;
        Vector takeOffDirection;
        //the airport where the drone is currently at
        MapAirport currentAirport = this.getAutopilot().getCommunicator().getAirportAtCurrentLocation();

        //current orientation of the drone
        Vector orientation = Controller.extractOrientation(inputs);
        //the orientation of both runways
        Vector gateZeroOrientation = currentAirport.getRunwayZeroTakeoffDirection();
        Vector gateOneOrientation = currentAirport.getRunwayOneTakeoffDirection();


        //go to the gate for which the drone has to turn the least and is therefore the easiest to reach
        if (Math.abs(orientation.getAngleBetween(gateZeroOrientation)) <  Math.abs(orientation.getAngleBetween(gateOneOrientation))) {
            taxiTo = currentAirport.getLocation();
            takeOffDirection = currentAirport.getRunwayZeroTakeoffDirection();
        }
        else {
            taxiTo = currentAirport.getLocation();
            takeOffDirection = currentAirport.getRunwayOneTakeoffDirection();
        }


        //taxiTo = Controller.extractPosition(inputs);

        //set the destination of the drone and the correct orientation so it can take off from the runway
        setTarget(taxiTo, takeOffDirection);
    }



    /**
     * The controller that regulates the velocity during the taxiing phase of the drone
     * @param outputs the outputs where the control actions are written to
     * @param prevInputs the previous inputs of the autopilot
     * @param currentInputs the current inputs of the autopilot
     * note: only invoke this function AFTER writing the brake controls, otherwise the behaviour
     *       of the drone is unknown
     */
    private void cruiseControl(ControlOutputs outputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs){
        //we need to get an approx for the velocity before we can make any further calculations
        Vector approxVel = Controller.getVelocityApprox(prevInputs, currentInputs);
        float totalVelocityApprox = approxVel.getSize();
        //get the total mass of the drone (needed for the calculations)
        float totalMass = this.getTotalMass();
        //get the velocity controller
        PIDController velocityController = this.getVelocityController();
        //get the delta time, note this is an extrapolation, we assume the time step stays the same across the simulation
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float errorVelocity = calcRefVelocity(currentInputs) - totalVelocityApprox;
//        System.out.println(approxVel);
        //now calculate the output of the PID
        float errorVelPID = velocityController.getPIDOutput(-errorVelocity, deltaTime);
//        System.out.println("error on velocity: "+ errorVelPID);
        //based on the error in the velocity we can calculate our control actions
        //if the error is positive we need to accelerate (setpoint - velocity = error)
        //if the error is negative we need to brake
        if(errorVelPID > 0){
//            System.out.println("Adjusting thrust");
            float maxThrust = this.getConfig().getMaxThrust();
            float thrust = getCorrectionThrust(totalMass, errorVelPID, maxThrust, deltaTime);
            outputs.setThrust(thrust);
        }else if(errorVelPID < 0){
//            System.out.println("Adjusting brakes");
            float maxBrake = this.getConfig().getRMax();
            Vector brakeVector = getCorrectionBrakes(totalMass, errorVelPID, maxBrake, deltaTime, outputs);
            setBrakeVector(outputs, brakeVector);
        }

        // we are finished
//        System.out.println(outputs);
//        System.out.println();
    }

    /**
     * Calculates the reference velocity used by the taxiing controller
     * slows down when closer to target
     * @param inputs the current inputs to extract the needed state data from
     * @return the velocity needed for the reference
     */
    private float calcRefVelocity(AutopilotInputs_v2 inputs){
        Vector target = this.getTarget();
        //extract the position
        Vector position = Controller.extractPosition(inputs);
        //get the distance to the target
        float distanceToTarget = position.distanceBetween(target);

        return distanceToTarget > LOW_VEL_RADIUS ? HIGH_REF_VELOCITY : LOW_REF_VELOCITY;
    }

    /**
     * The high reference velocity
     */
    private static float HIGH_REF_VELOCITY = 25f;
    /**
     * The low reference velocity
     */
    private static float LOW_REF_VELOCITY = 5f;
    /**
     * The radius before we go in low velocity mode
     */
    private static float LOW_VEL_RADIUS = 150f;

    /**
     * Calculates the corrective thrust action needed to keep the velocity up to level
     * @param totalMass the total mass, needed for the calcultations
     * @param deltaVelocity the error on the velocity
     * @param maxThrust the maximum thrust the drone can deliver
     * @param deltaTime the time difference
     * @return the thrust needed to make the error on the velocity zero, capped on the max thrust
     */
    private static float getCorrectionThrust(float totalMass, float deltaVelocity, float maxThrust, float deltaTime){
        //first calculate the thrust needed to reach the desired velocity
        float desiredThrust = deltaVelocity*totalMass/deltaTime;
        //now check if we can reach it
        return min(desiredThrust, maxThrust);

    }

    /**
     * Calculates the correction needed by the brakes to compensate for the error on the velocity
     * @param totalMass the total mass of the drone
     * @param deltaVelocity the velocity error
     * @param maxBrake the maximum brake force that the drone can exert on a tyre
     * @param deltaTime the time difference between two simulation steps
     * @param outputs the outptus that were already written before this method was invoked
     * @return a vector containing the brake forces needed to correct the erroneous velocity
     *         the front, left and right brake forces are the x, y and z components of the vector respectively
     */
    private static Vector getCorrectionBrakes(float totalMass, float deltaVelocity, float maxBrake, float deltaTime, ControlOutputs outputs){
        //TODO corrective forces of the brakes may override the forces calculated by the steering controller, maybe scale the brake force down until the calculated ones for turning are at cap
        //assumption, all the brakes carry the same brake force and this scalar may be divided by 3 to calc the force needed at each tyre
        float totalBrakeForce = -deltaVelocity*totalMass/deltaTime;//minus to make the result positive
        //now calc the brake force needed for each tyre and cap it to the max brake force
        float desiredTyreBrakeForce = min(totalBrakeForce/3f, maxBrake);


        //get the current brake vector
        Vector currentBrakeVector = extractBrakeVector(outputs);
        //cap the vector components
        return getCompatibleBrakeVector(maxBrake, desiredTyreBrakeForce, currentBrakeVector);
    }

    /**
     * note: ignore the whole documentation hogwash and read the example
     * Checks if the current desired brake force doesn't overwrite any of the control actions taken by the steering controls
     * @param maxBrake the maximum brake force
     * @param desiredTyreBrakeForce the desired brake force for the tyres
     * @param currentBrakeVector the brake vector currently stored in the outputs of the controller
     *                           front, left and right brake are the x, y and z components respectively
     * @return returns a new brake vector that is either the sum of the current brake force with the desired one
     * or a version where the desired brake force was capped in such a way that a uniform desired brake force vector
     * (with the same components on x,y,z) summed with the current brake vector produces the maximum brake force
     * for the largest component of the current brake force
     *
     * (eg if desired brake force is (1000,1000,1000) and current is (500,0,0)
     * with a maximum brake force of 1000, the resulting vector is (1000, 500, 500)
     */
    private static Vector getCompatibleBrakeVector(float maxBrake, float desiredTyreBrakeForce, Vector currentBrakeVector) {
        Vector desiredBrakeVector = new Vector(desiredTyreBrakeForce, desiredTyreBrakeForce, desiredTyreBrakeForce);
        //check if we will overwrite a previous control action
        Vector sumVector = currentBrakeVector.vectorSum(desiredBrakeVector);
        float maxVal = sumVector.getMaxComponent();
        //the difference between the max value of the sum and the maximum exertable brake force
        float diffBrakeForce = maxVal - maxBrake;
        //if the max value is larger than the max brake, we need to rescale
        if(maxVal > maxBrake){
            float modTyreBrakeForce = desiredBrakeVector.getzValue() - diffBrakeForce; // the size of the brake vector at z is equal to the break force exerted on the tyre (in ideal circumstances)
            Vector modBrakeVector = new Vector(modTyreBrakeForce, modTyreBrakeForce, modTyreBrakeForce);
            sumVector = modBrakeVector.vectorSum(currentBrakeVector);
        }
        return sumVector;
    }

    /**
     * Extracts a vector of brake forces from the given outputs
     * @param outputs the outputs that are currently generated (may be overwritten later)
     * @return an immutable vector containing the front, left and right brake force as its x, y and z components
     */
    private static Vector extractBrakeVector(ControlOutputs outputs) {
        float currentFrontBrake = outputs.getFrontBrakeForce();
        float currentLeftBrake = outputs.getLeftBrakeForce();
        float currentRightBrake = outputs.getRightBrakeForce();
        return new Vector(currentFrontBrake, currentLeftBrake ,currentRightBrake );
    }

    /**
     * Writes the provided brake vector to the provided outputs, overwriting the previous outputs
     * @param outputs the outputs where the brake vector is written to
     * @param brakeVector a vector containing the brake forces, the front, left and right brake forces
     *                    are respectively the x, y and z components of the vector
     */
    private static void setBrakeVector(ControlOutputs outputs, Vector brakeVector){
        outputs.setFrontBrakeForce(brakeVector.getxValue());
        outputs.setLeftBrakeForce(brakeVector.getyValue());
        outputs.setRightBrakeForce(brakeVector.getzValue());
    }

    /**
     * The controller for the brakes while navigating for turning the drone to the target
     * @param outputs the outputs to write the control actions to
     * @param currentInputs the current outputs used to extract data
     * @param prevInputs the current inputs used to extract data
     */
    private void brakeTurningControls(ControlOutputs outputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs) {
        //get the time difference between two steps, note that this is an extrapolation, we assume here that
        //the time between simulation steps stay the same
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        //get the max R:
        float maxBrake = this.getConfig().getRMax();
        //get the target
        Vector target = this.getTarget();
        //then we need to calculate our angle
        float angle = angleToTarget(target, currentInputs);
        //then get the PID controller
        PIDController brakeController = this.getBrakeController();
        //get the outputs
        float pidOutputs = brakeController.getPIDOutput(angle, deltaTime);
        //will give negative output if we need to go right, positive if we need to go to the left
//        System.out.println("brakePidOutputs " + pidOutputs);
        outputs.setLeftBrakeForce(min(abs(min(pidOutputs, 0)), maxBrake));
        outputs.setRightBrakeForce(min(abs(max(pidOutputs, 0)), maxBrake));
//        System.out.println("Turning " + (outputs.getRightBrakeForce() > 0? "right" : "left"));
    }

    /**
     * Calculates the angle between vector difference of the current target and the drone, and the drone's heading vector
     * positive angles indicate a need for steering to the left
     * and negative angles indicate a need for steering to the right
     * @param target the target of the controller
     * @return the angle between the target and the heading vector of the drone
     *         the sign of the angle indicates in which direction the controller needs to steer
     */
    private static float angleToTarget(Vector target, AutopilotInputs_v2 inputs){
        Vector position = Controller.extractPosition(inputs);
        Vector orientation = Controller.extractOrientation(inputs);
        //calculate the difference vector between the target and our current position
        Vector diffVector = target.vectorDifference(position);
        //now project the vector on the xz-plane && normalize
        Vector normal = new Vector(0,1,0); // the normal vector of the xz-plane
        Vector projDiffVector = diffVector.orthogonalProjection(normal); // project onto the plane

        //now calculate the current heading vector of the drone in the world axis system
        Vector headingDrone = new Vector(0,0,-1);
        //transform
        Vector headingWorld = PhysXEngine.droneOnWorld(headingDrone, orientation);
        //project
        Vector projHeadingWorld = headingWorld.orthogonalProjection(normal);

        //now get the angle between both
        float angle = abs(projHeadingWorld.getAngleBetween(projDiffVector));

//        System.out.println("Projected heading vector: " + projHeadingWorld);
//        System.out.println("Projected diff vector: " + projDiffVector);
        //calculate the direction, we use the vector product of the heading vector and the difference vector
        //a positive vector indicates that the target is located to the left, a negative angle indicates the
        //target is located at the right of the drone (the result is completely located on the y-axis, the normal)
        float direction = signum(projHeadingWorld.crossProduct(projDiffVector).scalarProduct(normal));
//        System.out.println("Direction: " + direction);

        float result = angle*direction;
        //check for NaN
        if(Float.isNaN(result)){
            return 0f;
        }

        return result;
    }



//    private float angleToWantedOrientation(AutopilotInputs_v2 inputs){
//        Vector orientation = Controller.extractOrientation(inputs);
//        Vector normal = new Vector(0,1,0); // the normal vector of the xz-plane
//        //now calculate the current heading vector of the drone in the world axis system
//        Vector headingDrone = new Vector(0,0,-1);
//        //transform
//        Vector headingWorld = PhysXEngine.droneOnWorld(headingDrone, orientation);
//        //project
//        Vector projHeadingWorld = headingWorld.orthogonalProjection(normal);
//
//        Vector wantedOrientation = getWantedOrientation();
//
//        //now get the angle between the heading vector and the wantedOrientation
//        float angle = abs(projHeadingWorld.getAngleBetween(wantedOrientation));
//
//        //calculate the direction, we use the vector product of the heading vector and the difference vector
//        //a positive vector indicates that the target is located to the left, a negative angle indicates the
//        float direction = signum(projHeadingWorld.crossProduct(wantedOrientation).scalarProduct(normal));
//
//        float result = angle*direction;
//        //check for NaN
//        if(Float.isNaN(result)){
//            return 0f;
//        }
//
//        return result;
//    }
//
//    private boolean orientationWithinErrorRange(AutopilotInputs_v2 inputs){
//        float angle = angleToWantedOrientation(inputs);
//
//
//        if(maxOrientationError() <= abs(angle)){
//            return true;
//        }
//
//        return false;
//    }


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

    /**
     * Getter for the target of the taxiing controller
     * @return the target of the taxiing controller
     */
    private Vector getTarget() {
        return target;
    }

    /**
     * Setter for the target of the taxiing controller (for use in the controller selector only)
     * @param target the target to navigate to
     */
    public void setTarget(Vector target) {
        this.target = target;
    }

    /**
     * Getter for the controller that controls the brakes of the drone for navigating to our target
     * @return the controller used for steering the wheels
     */
    public PIDController getBrakeController() {
        return brakeController;
    }

    /**
     * Getter for the controller that controls the velocity of the drone for navigating to our target
     * @return the controller used to keep our velocity
     */
    public PIDController getVelocityController() {
        return velocityController;
    }

    /**
     * Getter for the current taxi state, the state of our finite state taxiing machine
     * @return the current taxiing state
     */
    private TaxiingState getTaxiingState() {
        return taxiingState;
    }

    /**
     * Setter for the current Taxiing state
     * @param taxiingState the taxiing state of the drone
     */
    private void setTaxiingState(TaxiingState taxiingState) {
        this.taxiingState = taxiingState;
    }

    /**
     * The target of the taxiing controller
     */
    //TODO remove standard value if the functionality of the controller is tested and the controller selector can make use of it
    private Vector target = new Vector(-500, 0,1000);

    /**
     * The reference velocity used by the autopilot cruise control
     */
    private float referenceVelocity = 5f;
    /**
     * The PID controller used to determine the force that needs to be exerted on the brakes
     */
    private final static float BRAKE_GAIN = 500;
    private final static float BRAKE_INTEGRAL = 0;
    private final static float BRAKE_DERIVATIVE =0;
    private PIDController brakeController = new PIDController(BRAKE_GAIN, BRAKE_INTEGRAL, BRAKE_DERIVATIVE);

    /**
     * The controller used to determine the velocity of the drone
     */
    private final static float VELOCITY_GRAIN = 1;
    private final static float VELOCITY_INTEGRAL = 0;
    private final static float VELOCITY_DERIVATIVE = 0;
    private PIDController velocityController = new PIDController(VELOCITY_GRAIN, VELOCITY_INTEGRAL, VELOCITY_DERIVATIVE);

    /**
     * The current state of the taxiing, first we need to take an initial turn so we are in the right
     * position to taxi to the target, we always start with an init turn
     */
    private TaxiingState taxiingState = TaxiingState.INIT_TURN;


    private enum TaxiingState {
        INIT_TURN, FULL_BRAKE, MOVING_TO_TARGET
    }




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

    public StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

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

//  /**
// */
//    public AutopilotTaxiingController(AutoPilot autopilot) {
//        super(autopilot);
//    }
//
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
//
//        this.updateInputs(inputs);
//
//        AutopilotInputs_v2 currentInputs = getCurrentInputs();
//        AutopilotInputs_v2 previousInputs = getPreviousInputs();
//        ControlOutputs outputs = new ControlOutputs();
//        Vector velocity = getVelocityApprox(previousInputs,currentInputs);
//        Vector orientation = Controller.extractOrientation(inputs);
//        Vector position = Controller.extractPosition(inputs);
//        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));
//
//
//
//        //the z coordinate of the location the drone has to go to (has to be changed to something with the path)
//        float desiredZValue = 30;
//        //the x coordinate of the location the drone has to go to
//        float desiredXValue = 0;
//
//        if (!hasToTurn(desiredXValue,desiredZValue,orientation,position)){
//            goForward(velocity, outputs);
//        }
//        else{
//            uTurn(outputs);
//        }
//        //check if the drone has reached the desired Z position and break if he has
//        if (Math.abs(desiredZValue) - Math.abs(position.getzValue()) < Math.abs(velocity.getzValue())) {
//            setBrakeForce(outputs, 800,800,800);
//            outputs.setThrust(0);
//        }
//
//
//
//        //graveyard of things that didn't work either
//      /*  if (desiredZValue < 0) {
//
//            if (Math.abs(desiredZValue) - Math.abs(position.getzValue()) < Math.abs(velocity.getzValue())) {
//                if (Controller.extractOrientation(inputs).getxValue() > 0) {
//                    UTurn(outputs);
//                    System.out.println("a");
//
//                } else if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
//                //    setBrakeForce(outputs, 1000);
//                    outputs.setThrust(0);
//                    System.out.println("b");
//
//                } else {
//                    goForward(velocity,outputs);
//                    System.out.println("c");
//
////                    stabillizeOrientation(outputs, (float) -PI, Controller.extractOrientation(inputs));
//                }
//
//            }
//        }
//
//        if (desiredZValue > 0) {
//            if (Controller.extractOrientation(inputs).getxValue() > -PI) {
//              //  UTurn(outputs);
//                System.out.println("a");
//                turnSouth(outputs);
//
//            }
//            else if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
//               // setBrakeForce(outputs, 1000);
//                outputs.setThrust(0);
//                System.out.println("b");
//
//            }
//            else{
//            //    goForward(velocity,outputs);
//                System.out.println("c");
//
//                stabillizeOrientation(outputs,(float)-PI,Controller.extractOrientation(inputs));
//            }
//
//        }
//*/
//
//
//        //  System.out.println("x: "+Controller.extractPosition(inputs).getxValue());
//        //System.out.println("z: "+Controller.extractPosition(inputs).getzValue());
//
//        //System.out.println("velocity " + velocity);
//        System.out.println("orientation " + Controller.extractOrientation(inputs));
//
//
//        return outputs;
//    }
//
//    @Override
//    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
//        return false;
//    }
//
//    /**
//     * Check if the drone has to turn to reach his destination
//     * @param desiredXValue
//     * @param desiredZValue
//     * @param orientation
//     * @param currPos
//     * @return
//     */
//    private boolean hasToTurn(float desiredXValue, float desiredZValue, Vector orientation, Vector currPos){
//        //error allowed on the orientation
//        float delta = 0.25f;
//        //error allowed on the x position
//        float delta2 = 0.5f;
//
//        //if the current x position is (almost) correct the drone doesn't have to turn
//        if (desiredXValue > currPos.getxValue()-delta2 && desiredXValue < currPos.getxValue()+delta2) {
//            //if the the desired z position is smaller than the current and the x orientation is (almost) 0, the drone just has to drive straight forward
//            if (desiredZValue < currPos.getzValue()) {
//                if (orientation.getxValue() > 0 - delta && orientation.getxValue() < 0 + delta) {
//                    return false;
//                }
//            }
//            //if the the desired z position is bigger than the current and the x orientation is (almost) PI, the drone just has to drive straight forward
//            if (desiredZValue < currPos.getzValue()) {
//                if (Math.abs(orientation.getxValue()) > PI - delta && Math.abs(orientation.getxValue()) < PI + delta) {
//                    return false;
//                }
//            }
//        }
//
//
//        return true;
//    }
//
//    /**
//     * try to stabilize around an orientation
//     * @param outputs
//     * @param stabilizeAround
//     * @param orientation
//     */
//    private void stabillizeOrientation(ControlOutputs outputs, float stabilizeAround, Vector orientation){
//        if (stabilizeAround > orientation.getxValue()){
//            setBrakeForce(outputs,0,100,0);
//            outputs.setVerStabInclination(0);
//            outputs.setThrust(200);
//            System.out.println("l");
//
//        }
//        if (stabilizeAround < orientation.getxValue()){
//            setBrakeForce(outputs,0,0,100);
//            outputs.setVerStabInclination(0);
//            outputs.setThrust(200);
//            System.out.println("r");
//
//        }
//    }
//
//    /**
//     * set the force on all 3 breaks (front, left, right)
//     * @param outputs
//     * @param front
//     * @param left
//     * @param right
//     */
//    private void setBrakeForce(ControlOutputs outputs, float front, float left, float right){
//        outputs.setFrontBrakeForce(front);
//        outputs.setLeftBrakeForce(left);
//        outputs.setRightBrakeForce(right);
//    }
//
//
///*    private void turnNorth(ControlOutputs outputs){
//        outputs.setFrontBrakeForce(0);
//        outputs.setLeftBrakeForce(0);
//        outputs.setRightBrakeForce(300);
//        outputs.setVerStabInclination( (float) PI/3);
//        outputs.setThrust(400);
//    }*/
//
//
///*    private void turnSouth(ControlOutputs outputs){
//        outputs.setFrontBrakeForce(0);
//        outputs.setLeftBrakeForce(300);
//        outputs.setRightBrakeForce(0);
//        outputs.setVerStabInclination( (float) PI/3);
//        outputs.setThrust(400);
//    }*/
//
//    /**
//     turn 180°
//     */
//    private void uTurn(ControlOutputs outputs){
//        setBrakeForce(outputs,0,0,300);
//        outputs.setVerStabInclination( (float) PI/3);
//        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
//        outputs.setThrust(maxThrust/3);
//    }
//
//
//    /**
//     * go forward at a max speed of 10m/s
//     * @param velocity
//     * @param outputs
//     */
//    private void goForward(Vector velocity, ControlOutputs outputs){
//        outputs.setRightWingInclination(0);
//        outputs.setLeftWingInclination(0);
//        outputs.setHorStabInclination(0);
//        outputs.setVerStabInclination(0);
//        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));
//
//        if (velocity1D > TAXI_VELOCITY) {
//            outputs.setThrust(0);
//            setBrakeForce(outputs, 50f, 50f, 50f);
//        }
//        else{
//            setThrust(outputs);
//            setBrakeForce(outputs,0f,0f,0f);
//        }
//    }
//
//
//
//    private void setThrust(ControlOutputs outputs) {
//        //get the maximal thrust
//        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
//        //elapsed time:
//        float elapsedTime = this.getCurrentInputs().getElapsedTime();
//        //first get an approx of the current velocity
//        Vector velocity = this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs());
//        //then get a response from the PID
//        Vector velocityPID = this.getVelocityPID().getPIDOutput(velocity, elapsedTime);
//        //use the PID output for the corrective action (gives the error on the velocity)
//        //we we take the desired output thrust (for stable config) if the velocity is too high, pull back, to low go faster
//        //System.out.println("velocityPID: " + velocityPID);
//        float outputThrust = (10 - (TAXI_VELOCITY + velocityPID.getzValue()) / TAXI_VELOCITY) * STANDARD_THRUST;
//        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));
//    }
//
//
//
//    /**
//     * Constants
//     */
//    private final static float TAXI_VELOCITY = 10.0f;
//    private final static float STANDARD_THRUST = 128.41895f * 3.5f;
//    private final static float WING_INCL = (float) (PI / 180);
//    private final static float HORIZONTAL_STABILIZER = (float) (PI / (180));
//    private final static float PITCH_THRESHOLD = (float) (5 * PI / 180);
//    private static final float INIT_LANDING_HEIGHT = 8f;
//    private final static float MAIN_STABLE = (float) (5*PI/180);
//    private final static float STABILIZER_STABLE = 0;
//    private final static float ROLL_THRESHOLD = (float)(3*PI/180);
//    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);
//    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);
//    private final static float PLANE_HEIGHT_FROM_GROUND = 1.2f;
//
//    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
//
//    private VectorPID getVelocityPID() {
//        return this.velocityPID;
//    }
//
//
//    @Override
//    protected float getMainStableInclination() {
//        return MAIN_STABLE;
//    }
//
//    @Override
//    protected float getStabilizerStableInclination() {
//        return STABILIZER_STABLE;
//    }
//
//    @Override
//    protected float getRollThreshold() {
//        return ROLL_THRESHOLD;
//    }
//
//    @Override
//    protected float getInclinationAOAErrorMargin() {
//        return INCLINATION_AOA_ERROR_MARGIN;
//    }
//
//    @Override
//    protected float getStandardThrust() {
//        return STANDARD_THRUST;
//    }

