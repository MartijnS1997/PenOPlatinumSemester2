package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import TestbedAutopilotInterface.Overseer.MapAirport;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

/**
 * Created by Martijn on 2/05/2018.
 * A class configured for taxiing to the runway
 * TODO now the controller only takes in account the heading direction and not its location on the airport
 * TODO if needed we can build in airport dependent reference points by setting the target along a point starting from the current gate
 */
public class RunwayTaxiingController extends TaxiingController{

    public RunwayTaxiingController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        ControlOutputs outputs = new ControlOutputs(getStandardOutputs());

        //calculate the target position
        Vector target = this.calculateTargetPos(currentInputs);
        //check if we have reached the target, if so, brake
        if(this.takeOffOrientationReached(currentInputs, target)){
            //note that we want to come to a standstill after taxiing because the takeoff controller wil not
            //takeoff until it has received a new package order
            setMaxBrake(outputs);
            return outputs;
        }

        brakeTurningControls(outputs, currentInputs, previousInputs, target, getBrakeController());
        cruiseControl(outputs, previousInputs, currentInputs);

        //the borders of the Airport we're currently in
        //Vector[] airportBorders = getAutopilot().getCommunicator().getAirportAtCurrentLocation().getAirportBorders();
        //make sure you always stay within the borders of the Airport
        //stayWithinAirportBorders(airportBorders,outputs,currentInputs);

//        System.out.println("Thrust: " + outputs.getThrust());
        return outputs;
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //the drone has reached its objective if it is in the right orientation and has a velocity lower than the standstill velocity
        //first we check if the standstill was achieved (this requires less calculations)
        if(this.standstillAchieved(currentInputs, previousInputs)){
            //if we're standing still we also check if the drone is facing the right direction
            Vector target = this.calculateTargetPos(currentInputs);
            return takeOffOrientationReached(currentInputs, target);
        }
        return false;
    }

    @Override
    public void reset() {
        this.getBrakeController().reset();
    }

    /**
     * Checks if the angle between the target and the heading-orientation of the drone is smaller than the error margin
     * described in getMaxTakeoffErrorAngle()
     * @param currentInputs the inputs most recently received from the testbed
     * @param target the target for the drone
     * @return true if and only if the current angle between the heading of the drone and the target is smaller than the
     *         getMaxTakeoffErrorAngle()
     */
    private boolean takeOffOrientationReached(AutopilotInputs_v2 currentInputs, Vector target){
        //calculate the angle between the target and the current heading of the drone
        float absCurrentErrorAngle = abs(angleToTarget(currentInputs, target));
        //get the maximum allowed error
        float maxAngleError = getMaxTakeoffErrorAngle();
        //check if the absolute error is smaller than the maximum error and return
        return maxAngleError >= absCurrentErrorAngle;

    }

    /**
     * Checks if the drone is standing still, the drone has reached standstill if it goes slower than the 'standstill'
     * velocity
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return true if and only if the current velocity (approximate) is lower than the standstill velocity
     *         (accessed via getStandstillVelocity)
     */
    private boolean standstillAchieved(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get an approx for the velocity
        Vector velocityApprox = getVelocityApprox(currentInputs, previousInputs);
        //get the total velocity
        float absCurrentVelocity = velocityApprox.getSize();
        //check if we're standing still
        float standStillVelocity = this.getStandStillVelocity();
        return absCurrentVelocity <= standStillVelocity;
    }

    /**
     * Calculates the position of the target of the drone, the target is placed along the heading of the takeoff
     * runway at the getLookaheadDistance() away from the drone
     * @param currentInputs the inputs most recently received from the testbed
     * @return the calculated target position
     */
    private Vector calculateTargetPos(AutopilotInputs_v2 currentInputs){
        //extract the current position of the drone in ground coordinates
        Vector droneGroundPos = extractGroundPosition(currentInputs);

        //get the takeoff direction
        Vector takeoffDirection = this.getTakeoffDirection();
        float lookaheadDistance = getLookaheadDistance();
        //scale the direction to the lookahead distance, we need to set the target the lookahead distance away from the drone
        Vector relativeTargetPos = takeoffDirection.normalizeToLength(lookaheadDistance);

        //sum with the current ground position of the drone
        return droneGroundPos.vectorSum(relativeTargetPos);
    }

    public void configureRunwayController(AutopilotInputs_v2 currentInputs){
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();

        //we can't rely on the package that we have to deliver so we ask the overseer on which airport we're standing
        //and use it to get our takeoff direction
        MapAirport taxiingAirport = communicator.getAirportAtCurrentLocation();

        //now get the main heading of the airport
        Vector takeoffHeading = takeOffHeading(currentInputs, taxiingAirport);
        this.setAirport(taxiingAirport);
        this.setTakeoffDirection(takeoffHeading);
    }

    /**
     * Calculates the takeoff heading for the drone based on the current orientation and the airport the drone is
     * currently standing on
     * @param currentInputs the current inputs
     * @param taxiingAirport the airport the drone is currently standing on
     * @return the heading of the airport is the drone is facing along runway zero
     *         the heading of the airport mirrored along the origin if the drone is facing along runway one
     *         (scalar multiplication of the airport heading with -1)
     */
    private Vector takeOffHeading(AutopilotInputs_v2 currentInputs, MapAirport taxiingAirport){
        //first extract the orientation and get the heading of the drone in the heading axis system (we do not account
        //for roll and pitch, transform the heading to the world axis system
        Vector droneOrientation = extractOrientation(currentInputs);
        Vector droneHeadingVector = new Vector(0,0,-1);
        Vector droneHeading = PhysXEngine.headingOnWorld(droneHeadingVector, droneOrientation);

        //now get the heading of the airport, this coincides with the heading of runway zero
        Vector airportHeading = taxiingAirport.getHeadingVector();

        //do a scalar product and take the sign to get the heading vector required for the takeoff
        //if the takeoff direction is positive the target heading is along runway zero
        //if the direction is negative the target heading is along runway one
        float takeoffDir = signum(airportHeading.scalarProduct(droneHeading));

        //now do a scalar multiplication with the takeoff direction and return
        return airportHeading.scalarMult(takeoffDir);
    }

    /**
     * Getter for the target heading, this is the heading direction in which the drone has to face before
     * the drone can continue to the takeoff phase
     * @return a vector containing the heading needed for a takeoff: (x, 0, z) directions
     */
    private Vector getTakeoffDirection() {
        return takeoffDirection;
    }

    /**
     * Setter for the target heading of the drone, this is the heading direction in which the drone has to face before
     * it can takeoff (must be set upon configuring the drone)
     * @param takeoffDirection the target heading to be reached by the controller
     */
    private void setTakeoffDirection(Vector takeoffDirection) {
        this.takeoffDirection = takeoffDirection;
    }

    /**
     * Getter for the airport the drone is currently manoeuvring at
     * @return a map airport object containing the info about the current airport
     */
    private MapAirport getAirport() {
        return airport;
    }

    /**
     * Setter for the airport the drone is currently manoeuvring at
     * note: should be called when the controller is configured
     * @param airport the airport the drone is currently at
     */
    private void setAirport(MapAirport airport) {
        this.airport = airport;
    }

    /**
     * Getter for the maximum error angle, this is the angle the drone may maximally deviate from the desired takeoff
     * direction, if this angle is breached the drone may takeoff
     * @return the heading error on the angle maximally allowable
     */
    private static float getMaxTakeoffErrorAngle() {
        return maxTakeoffErrorAngle;
    }

    /**
     * Getter for the lookahead distance, this is the distance between the drone and the calculated target
     * may be set further from the drone to smooth controls or steer more aggressively
     * @return the distance to place the reference from the drone
     */
    private static float getLookaheadDistance() {
        return lookaheadDistance;
    }

    /**
     * The target heading of the drone, this is the direction in which the drone has to face to make a good takeoff
     * TODO for now we only consider the takeoff direction and don't check if the drone is actually on the airport
     */
    private Vector takeoffDirection;

    /**
     * The airport the drone is currently manoeuvring at
     */
    private MapAirport airport;

    /**
     * Getter for the maximum heading error angle for the controller before it can say it has finished
     * an error angle of 1 degree results in an error of 1.75m every 100m of the takeoff
     * and an error angle of 0.5 degree results in an error of 0.85m per 100m (see which works best)
     */
    private final static float maxTakeoffErrorAngle = (float) (1*PI/180f);

    /**
     * The lookahead distance, this is the distance between the target and the drone
     */
    private final static float lookaheadDistance = 50f;

    /*
    Controller instances
     */

    /**
     * Getter for the controller that is responsible for regulating the brakes of the drone
     * @return the brake controller
     */
    private PIDController getBrakeController() {
        return brakeController;
    }

    /**
     * The PID controller used to determine the force that needs to be exerted on the brakes
     */
    private final static float BRAKE_GAIN = 500;
    private final static float BRAKE_INTEGRAL = 0;
    private final static float BRAKE_DERIVATIVE = 0;
    private PIDController brakeController = new PIDController(BRAKE_GAIN, BRAKE_INTEGRAL, BRAKE_DERIVATIVE);
}
