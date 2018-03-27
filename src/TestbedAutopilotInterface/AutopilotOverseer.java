package TestbedAutopilotInterface;

import AutopilotInterfaces.*;
import internal.Autopilot.AutoPilot;

import java.util.Map;
import java.util.concurrent.*;

//TODO to notify the threads that they need to deliver a new package, we need a queue for each connection to communicate with
//sequence:
//generate a new autopilot testbed with an integrated queue for delivery messages and a concurrentmap to write their state to
//(we can also use the broadcast queue to enter the states of all the drones

//note need a map of the world to navigate the drones and make decisions on what drone does what
/**
 * Created by Martijn on 27/03/2018.
 * A class of autopilot overseers to coordinate correct interaction between the drones
 */
public class AutopilotOverseer implements AutopilotModule, Callable<Void> {

    /**
     * Constructor for an autopilot overseer used to coordinate all the autopilots
     */
    public AutopilotOverseer() {

    }


    /**
     * Mainloop of the overseer
     * @return absolutely nothing
     * @throws Exception
     */
    @Override
    public Void call() throws Exception {
        return null;
    }

    /**
     * Defines the parameters for all airports
     * @param length the length of the airport
     * @param width the width of the airport
     */
    @Override
    public void defineAirportParams(float length, float width) {

    }

    /**
     * Define a new airport located  ath the given location
     * @param centerX the x-center coordinate
     * @param centerZ the y-center coordinate
     * @param centerToRunway0X the pointer of the first runway
     * @param centerToRunway0Z the pointer of the second runway
     */
    @Override
    public void defineAirport(float centerX, float centerZ, float centerToRunway0X, float centerToRunway0Z) {

    }

    /**
     * Defines a drone at the given airport standing at the given gate and pointing to the given runway
     * @param airport the airport number to place the drone at
     * @param gate the gate to place the drone at
     * @param pointingToRunway the runway the drone points to
     * @param config the configuration of the drone
     */
    @Override
    public void defineDrone(int airport, int gate, int pointingToRunway, AutopilotConfig config) {

    }

    /**
     * Notifies the given drone that a certain time has passed
     * @param drone
     * @param inputs
     * note: we will not use this method because we're going full multithread
     */
    @Override
    public void startTimeHasPassed(int drone, AutopilotInputs inputs) {

    }

    /**
     * This we will also not use
     */
    @Override
    public AutopilotOutputs completeTimeHasPassed(int drone) {
        return null;
    }

    /**
     * Setter for package delivery
     * @param fromAirport the sender airport
     * @param fromGate the sender gate to retrieve the package from
     * @param toAirport the destination airport
     * @param toGate the destination gate for the package
     */
    @Override
    public void deliverPackage(int fromAirport, int fromGate, int toAirport, int toGate) {

    }

    /**
     * Notifies the simulation has ended, breakdown all activity (close the threads)
     */
    @Override
    public void simulationEnded() {

    }

    /**
     * Invoked by the autopilot, used to store (or initialize) the state data associated with the
     * autopilot invoking the method (synchronized to prevent thread issues)
     * @param autoPilot the autopilot updating his status (used for identification)
     * @param inputs the current inputs of the autopilot
     * note: has to be called every time when using the autopilot
     */
    public synchronized void autopilotStatusUpdate(AutoPilot autoPilot, AutopilotInputs_v2 inputs){
        //TODO check if concurrency issues arise, use a check-in first and lock the map
        //get the id
        String autopilotId = autoPilot.getID();
        //get the map we write to
        ConcurrentMap<String, AutopilotInputs_v2> activeAutopilots = this.getActiveAutopilots();
        //replace the old entry
        activeAutopilots.put(autopilotId, inputs);
    }



    /**
     * Setter for the operational autopilots only to be used by the overseer
     * @param autopilotData the data to set
     */
    private void setActiveAutopilots(ConcurrentHashMap<String, AutopilotInputs_v2> autopilotData){
        this.activeAutopilots = autopilotData;
    }

    /**
     * Getter for the map that stores all the delivery requests based on the ID
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryRequest>> getDeliveryRequests(){
        return this.deliveryRequests;
    }

    /**
     * Getter for the queue containing the data needed for the autopilot
     * @param autoPilot the autopilot to extract the delivery requests for
     * @return the Queue of pending requests
     */
    public ConcurrentLinkedQueue<DeliveryRequest> getDeliveryRequest(AutoPilot autoPilot){
        //get the ID from the autopilot that is invoking the getter
        String value = autoPilot.getID();
        //retrieve the corresponding queue for the drone
        ConcurrentLinkedQueue<DeliveryRequest> queue = this.getDeliveryRequests().get(value);
        return queue;
    }

    /**
     * Getter for the active autopilot conditions, the active autopilots are identified by their ID and pass their
     * current input every time they are invoked
     * @return a map containing all the autopilot id's as values and their inputs
     */
    private ConcurrentMap<String, AutopilotInputs_v2> getActiveAutopilots() {
        return activeAutopilots;
    }

    /**
     * The hash map that stores all the active autopilots in the world
     * String contains the ID and inputs the current inputs of the autopilot
     */
    private ConcurrentMap<String, AutopilotInputs_v2> activeAutopilots = new ConcurrentHashMap<>();

    /**
     * A map used to store the all the request made to the overseer, the autopilots can query for their requests
     * to the broadcast
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryRequest>> deliveryRequests = new ConcurrentHashMap<>();

}
