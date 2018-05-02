package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import TestbedAutopilotInterface.Overseer.AutopilotDelivery;
import TestbedAutopilotInterface.Overseer.MapAirport;
import internal.Helper.Vector;

/**
 * Created by Martijn on 2/05/2018.
 * A controller made for taxiing to the gate of the airport
 */
public class GateTaxiingController extends TaxiingController{

    public GateTaxiingController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        ControlOutputs outputs = new ControlOutputs(getStandardOutputs());

        //check if we have reached the target, if so, brake
        if(this.targetReached(currentInputs)){
            this.setMaxBrake(outputs);
//            System.out.println("braking");
            return outputs;
        }

        brakeTurningControls(outputs, currentInputs, previousInputs, getTaxiTarget(), getBrakeController());
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
        if(targetReached(currentInputs)){
            Vector approxVelocity = getVelocityApprox(currentInputs, previousInputs);
            float standstillVelocity = this.getStandStillVelocity();
            float absVel = approxVelocity.getSize();
            return absVel <= standstillVelocity;
        }
        return false;
    }


    /**
     * Checks if the drone is close enough to the target
     * @param inputs the inputs received from the testbed
     * @return true if and only if the drone is close enough to the target
     */
    private boolean targetReached(AutopilotInputs_v2 inputs) {
        //extract the current state
        //check if the current target has been reached, this happens if we've got close enough
        //to the target
        Vector target = this.getTaxiTarget();
        //get the current pos
        Vector position = Controller.extractPosition(inputs);
//        System.out.println("distanceToTarget: " + position.distanceBetween(target));
//        System.out.println("position: " + position);

        return position.distanceBetween(target) <= getTargetReachedDistance();
    }


    /**
     * Configures the controller, must be called by the autopilot finite state machine to configure the controller correctly
     * @param inputs the inputs received from the testbed
     * @param packageToDeliver the package the drone needs to deliver
     */
    public void configureGateTaxiing(AutopilotInputs_v2 inputs, AutopilotDelivery packageToDeliver) {
        //airport to go to
        int targetAirportID;
        //gate to go to
        int targetGateID;
        //position to taxi to
        Vector taxiTarget = null;
        //Check if the package should be picked up or delivered to the gate (select the right destination)
        if (!packageToDeliver.isPickedUp()) {
            //the package is not picked up yet, we need to taxi to the source gate
            targetAirportID = packageToDeliver.getSourceAirport();
            targetGateID = packageToDeliver.getSourceAirportGate();
            System.out.println("taxiing to source airport");
        } else {
            System.out.println("taxiing to destination airport");
            targetAirportID = packageToDeliver.getDestinationAirport();
            targetGateID = packageToDeliver.getDestinationAirportGate();
        }

        //now get the position of the target
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        MapAirport targetAirport = communicator.getAirportByID(targetAirportID);

        //get the location of the gate at the target airport
        taxiTarget = targetAirport.getGateLocation(targetGateID);


        //set the destination of where the drone has to go to, orientation doesn't matter
        System.out.println("TaxiTarget: " + taxiTarget);
        setTaxiTarget(taxiTarget);
        setTaxiAirport(targetAirport);
    }

    /**
     * Getter for the taxi target, this is the location to which the controller has to taxi
     * normally this is the destination or source gate of the package to deliver
     * @return the target for the controller
     */
    private Vector getTaxiTarget() {
        return taxiTarget;
    }

    /**
     * Setter for the target to which the drone has to taxi
     * @param taxiTarget the target to taxii to
     */
    private void setTaxiTarget(Vector taxiTarget) {
        this.taxiTarget = taxiTarget;
    }

    /**
     * Getter for the airport on which the drone has to taxi
     * @return the airport where the drone is currently taxiing on (assuming the navigator did its work)
     */
    private MapAirport getTaxiAirport() {
        return taxiAirport;
    }

    /**
     * Setter for the airport on which the drone is currently taxiing (assuming the navigator did its work properly
     * @param taxiAirport the airport on which the
     */
    private void setTaxiAirport(MapAirport taxiAirport) {
        this.taxiAirport = taxiAirport;
    }

    /**
     * Getter for the distance the drone has to be to the target before the controller counts it as 'target reached'
     * if the drone is closer than this distance, the next controller is invoked
     * @return the distance minimally needed to the target before calling the next controller
     */
    private float getTargetReachedDistance() {
        return targetReachedDistance;
    }

    /**
     * Getter for the brake controller, this is the controller used for braking during the taxiing to gate phase
     * @return the brake controller
     */
    private PIDController getBrakeController() {
        return brakeController;
    }

    /**
     * The target for the taxiing controller, the target is the gate for which the package is destined
     */
    private Vector taxiTarget;

    /**
     * The airport we're currently taxiing at
     */
    private MapAirport taxiAirport;

    /**
     * The distance from the target before it gets counted as a 'target reached'
     */
    private final static float targetReachedDistance = 5.0f;

    /*
    Some controllers needed for the taxiing
     */
    /**
     * The PID controller used to determine the force that needs to be exerted on the brakes
     */
    private final static float BRAKE_GAIN = 500;
    private final static float BRAKE_INTEGRAL = 0;
    private final static float BRAKE_DERIVATIVE = 0;
    private PIDController brakeController = new PIDController(BRAKE_GAIN, BRAKE_INTEGRAL, BRAKE_DERIVATIVE);

}
