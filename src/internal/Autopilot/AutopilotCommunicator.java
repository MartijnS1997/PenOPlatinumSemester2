package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.AutopilotDelivery;
import TestbedAutopilotInterface.Overseer.AutopilotInfo;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import TestbedAutopilotInterface.Overseer.MapAirport;

import java.util.Set;

/**
 * Created by Martijn on 27/03/2018.
 * A class that assists the autopilot with the overseer communication
 */
public class AutopilotCommunicator {

    /**
     * Constructor for the overseer communication class
     * @param autopilot the autopilot used to verify our communication with the overseer
     * @param overseer the overseer that is currently governing all the autopilots
     */
    public AutopilotCommunicator(AutoPilot autopilot, AutopilotOverseer overseer){
        this.autopilot = autopilot;
        this.overseer = overseer;
    }

    /**
     * communicates with the overseer for the current details of the flight
     * --> note that the state is always sent first before accessing the data of the different drones
     * this means that on the first iteration all the drones will see default value for all the info they have
     * received, so they need to be able to cope with the init values
     */
    protected void communicateWithOverseer(){
        //get the overseer and the autopilot we represent
        AutopilotOverseer overseer = this.getOverseer();
        AutoPilot autopilot = this.getAutopilot();
        sendStateToOverseer(autopilot, overseer);
        readForDeliveryRequest(autopilot, overseer);
        //only query for the altitude after you've updated your status (if the drone wasn't
        //registered yet, it will assign a cruising altitude)
        float cruisingAltitude = overseer.getCruisingAltitude(autopilot);
        this.setAssignedCruiseAltitude(cruisingAltitude);
    }

    /**
     * Getter for the set that contains all the information about the other drones that the current drone is
     * sharing its airspace with
     * @return a set containing all the autopilot info objects containing info from all drones with a different
     *         id from the caller.
     */
    protected Set<AutopilotInfo> getOtherAutopilotsInformation(){
        AutopilotOverseer overseer = this.getOverseer();
        AutoPilot autopilot = this.getAutopilot();
        Set<AutopilotInfo> infoSet = overseer.getOtherAutopilotInfo(autopilot);
        return infoSet;
    }

    /**
     * Getter for the airport with the same id, calls the map of the overseer to retrieve the airport from
     * @param airportID the airport ID used to search the map
     * @return the airport with the corresponding ID, returns null if the airport is not present on the map
     * --> wrapper to shield the overseer from the rest of the autopilot
     */
    protected MapAirport getAirportByID(int airportID){
       return  this.getOverseer().getAirportByID(airportID);
    }

    /**
     * Getter for the airport at which the drone is currently located/standing on
     * @return the airport at which the drone is currently located, returns null if the drone isn't located at
     *         any airport
     * --> wrapper to shield the overseer from the rest of the autopilot
     */
    protected MapAirport getAirportAtCurrentLocation(){
        return this.getOverseer().getAirportAt(this.getAutopilot());
    }

    /**
     * Reads the queue for the next delivery request
     * if the queue is empty the current request is set to null
     * @param autopilot the autopilot to get the queue entry for
     * @param overseer th overseer to get the delivery request from
     */
    private void readForDeliveryRequest(AutoPilot autopilot, AutopilotOverseer overseer){
        //check if we are currently serving a request
        if(!mayRequestNextDelivery()){
            return;
        }
        //if not look for a new one
        //get the queue
        AutopilotDelivery nextDelivery = overseer.getNextDeliveryRequest(autopilot);
        //System.out.println(deliveryQueue);
        //now poll the queue, if not empty, take a new one, if empty return null
        //set the delivery request
//        System.out.println("nextDelivery: " + nextDelivery);
        this.setCurrentRequest(nextDelivery);
    }

    /**
     * Checks if the communicator may request a new package from the drone
     * @return true if the current package is a null reference or
     * if the current package is delivered
     */
    private boolean mayRequestNextDelivery(){
        AutopilotDelivery currentDelivery = this.getCurrentRequest();
        //if the current delivery is null, we may request for a new one
        //if the current delivery is delivered we also may request a new one
        return currentDelivery == null ||currentDelivery.isDelivered();
    }

    /**
     * Sends the current location and orientation of the drone to the overseer so it can calculate the control
     * actions needed for the autopilots + package delivery
     * @param autopilot the autopilot needed to verify with to overseer
     * @param overseer the overseer to send the current state to
     */
    private void sendStateToOverseer(AutoPilot autopilot, AutopilotOverseer overseer){
        //get the state machine
        AutopilotFiniteStateMachine stateMachine = autopilot.getStateMachine();
        //get the inputs to send to the overseer
        AutopilotInfo info = stateMachine.getAutopilotInfo();
        //send the info to the overseer
        overseer.autopilotStatusUpdate(autopilot, info);
    }

    /**
     * Requests permission to land on the specified airport
     * if the request is declined returns false
     * @param airportID the id of the airport to request landing for
     * @return true if the request is accepted, false is not
     */
    protected boolean requestLanding(int airportID){
        AutopilotOverseer overseer = this.getOverseer();
        return overseer.reserveAirport(this.getAutopilot(), airportID);
    }

    /**
     * Deletes the reservation that was previously made by the drone
     * note: must be called after every takeoff
     */
    protected void removeRequest(){
        AutopilotOverseer overseer = this.getOverseer();
        overseer.releaseAirport(this.getAutopilot());
    }

    /**
     * Getter for the overseer that this class will communicate with, the overseer
     * provides data about the other autopilots that are sharing the airspace with the drone
     * @return the overseer that is currently guiding all the drones
     */
    private AutopilotOverseer getOverseer() {
        return overseer;
    }

    /**
     * Getter for the assigned cruising altitude of the autopilot assigned by the overseer
     * @return the cruising altitude of the drone provided by the overseer
     */
    protected float getAssignedCruiseAltitude() {
        return assignedCruiseAltitude;
    }

    /**
     * Setter for the assigned cruising altitude, see getter for more info
     * @param assignedCruiseAltitude the cruising altitude assigned by the overseer
     */
    private void setAssignedCruiseAltitude(float assignedCruiseAltitude) {
        this.assignedCruiseAltitude = assignedCruiseAltitude;
    }

    /**
     * Getter for the delivery that the autopilot is currently handling, it contains all the information
     * necessary for delivering the packet
     * @return the current delivery request
     */
    protected AutopilotDelivery getCurrentRequest() {
        return currentRequest;
    }

    /**
     * sets the current request that is handled by the drone to null (we have delivered the package)
     */
    protected void packageDelivered(){
        this.currentRequest = null;
    }

    /**
     * Setter for the delivery request that is currently handled by the autopilot
     * @param currentRequest the current request
     */
    private void setCurrentRequest(AutopilotDelivery currentRequest) {
        this.currentRequest = currentRequest;
    }

    /**
     * Getter for the autopilot that the communicator represents by the overseer (needed for validation)
     * @return an AutoPilot object associated with the overseer communication
     */
    private AutoPilot getAutopilot() {
        return autopilot;
    }

    /**
     * The cruising altitude assigned by the overseer
     */
    private float assignedCruiseAltitude;

    /**
     * The current delivery request the autopilot is handling for the overseer,
     * these request get read from the queue that is associated with the autopilot in the overseer
     */
    private AutopilotDelivery currentRequest;

    /**
     * The autopilot that the communicator represents with the overseer
     */
    private AutoPilot autopilot;

    /**
     * The overseer currently responsible for the inter autopilot communication
     */
    private AutopilotOverseer overseer;

}
