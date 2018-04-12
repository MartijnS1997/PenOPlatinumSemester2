package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import TestbedAutopilotInterface.Overseer.DeliveryPackage;
import TestbedAutopilotInterface.Overseer.MapAirport;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Created by Martijn on 27/03/2018.
 * A class that assists the autopilot with the overseer communication
 */
public class OverseerCommunication {

    /**
     * Constructor for the overseer communication class
     * @param autopilot the autopilot used to verify our communication with the overseer
     * @param overseer the overseer that is currently governing all the autopilots
     */
    public OverseerCommunication(AutoPilot autopilot, AutopilotOverseer overseer){
        this.autopilot = autopilot;
        this.overseer = overseer;
    }

    /**
     * communicates with the overseer for the current details of the
     */
    protected void overseerCommunication(){
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

    private void readForDeliveryRequest(AutoPilot autopilot, AutopilotOverseer overseer){
        DeliveryPackage currentDeliveryRequest = this.getCurrentRequest();
        //check if we are currently serving a request
        if(currentDeliveryRequest != null){
            return;
        }
        //if not look for a new one
        //get the queue
        ConcurrentLinkedQueue<DeliveryPackage> deliveryQueue = overseer.getDeliveryRequest(autopilot);
        //System.out.println(deliveryQueue);
        //now poll the queue, if not empty, take a new one, if empty return null
        DeliveryPackage newRequest = !deliveryQueue.isEmpty() ? deliveryQueue.poll() : null;
        //set the delivery request
        this.setCurrentRequest(newRequest);
    }

    /**
     * Sends the current location and orientation of the drone to the overseer so it can calculate the control
     * actions needed for the autopilots + package delivery
     * @param autopilot the autopilot needed to verify with to overseer
     * @param overseer the overseer to send the current state to
     */
    private void sendStateToOverseer(AutoPilot autopilot, AutopilotOverseer overseer){
        //get the inputs to send to the overseer
        AutopilotInputs_v2 inputs = autopilot.getCurrentInputs();
        //send the info to the overseer
        overseer.autopilotStatusUpdate(autopilot, inputs);
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
    protected DeliveryPackage getCurrentRequest() {
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
    private void setCurrentRequest(DeliveryPackage currentRequest) {
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
    private DeliveryPackage currentRequest;

    /**
     * The autopilot that the communicator represents with the overseer
     */
    private AutoPilot autopilot;

    /**
     * The overseer currently responsible for the inter autopilot communication
     */
    private AutopilotOverseer overseer;

}
