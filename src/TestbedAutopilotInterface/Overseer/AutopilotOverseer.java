package TestbedAutopilotInterface.Overseer;

import AutopilotInterfaces.*;
import internal.Autopilot.AutoPilot;
import internal.Helper.Vector;

import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;

//TODO to notify the threads that they need to deliver a new package, we need a queue for each connection to communicate with
//sequence:
//generate a new autopilot testbed with an integrated queue for delivery messages and a concurrentmap to write their state to
//(we can also use the broadcast queue to enter the states of all the drones

//note need a map of the world to navigate the drones and make decisions on what drone does what
//TODO check if drones have crashed during the simulation, delete them if needed
//--> the overseer must set the assigned drone
//TODO also add the overseer to the world so it can read the set of packages that are pending
/**
 * Created by Martijn on 27/03/2018.
 * A class of autopilot overseers to coordinate correct interaction between the drones
 */
public class AutopilotOverseer implements AutopilotModule, Callable<Void>, PackageService {

    /**
     * Constructor for an autopilot overseer used to coordinate all the autopilots
     */
    public AutopilotOverseer(int initNbOfAutopilots) {
        //Bartje is overseer??
        this.initNbOfAutopilots = initNbOfAutopilots;
    }

    //all we must do is periodically check if there is any drone without any packages to deliver
    //if so we start the search for package allocation
    //TODO implement the calling for the delivery planner and allocate the packages
    //TODO implement mechanism to periodically check for empty drone queues
    //note we only support single package submission for now (may be added later)
    /**
     * Main loop of the overseer
     * @return absolutely nothing
     * @throws Exception
     */
    @Override
    public Void call() throws Exception {
        //only start the delivery if all drones to spread the deliveries across have been added
//        while(this.getActiveAutopilots().size() != this.getInitNbOfAutopilots()){
//            //do nothing
//        }
//
//        //call the main loop
//        overseerMainLoop();

        return null;
    }

    private void overseerMainLoop(){
        //get the airport map
        OverseerAirportMap airportMap = this.getAirportMap();
        //create the delivery planner
        DeliveryPlanner planner = new DeliveryPlanner(airportMap);
        this.setPlanner(planner);

        //loop until the simulation has been ended by the testbed
        while(!this.isSimulationEnded()){
            //distribute the packages
            distributePackages();
            //then wait until one of the drone's queues is empty
            while(!hasIdleDrone()){
                //TODO we may also use the autopilots to wake the overseer thread if they spot that their queue is empty
                //TODO make mechanism that the planner is not invoked if there are no packages to deliver
            }
        }
    }

    /**
     * Distributes the packages that have not been assigned yet to a drone
     * starts the delivery planner (may require some work --> beam search)
     */
    private void distributePackages(){
        //get the planner
        DeliveryPlanner planner = this.getPlanner();
        //initialize the search
        //first get the list of all packages
        Set<DeliveryPackage> submittedDeliveries = this.getSubmittedPackages();
        //get the map of the drones and their current assigned deliveries
        Map<String, List<DeliveryPackage>> assignedDeliveries = this.getUndeliveredPackagesPerDrone();
        //get the map of the drones and their current positions
        Map<String, Vector> autopilotPositions = this.getAutopilotPositions();
        //initialize the search
        planner.initializeSearch(submittedDeliveries, assignedDeliveries, autopilotPositions);
        //execute the search
        Map<String, List<DeliveryPackage>> deliveryScheme = planner.executeSearch();
        //and apply the found scheme
        assignDeliveriesToQueue(deliveryScheme);
    }

    /**
     * Assigns all the packages that are currently in the delivery buffer
     * to the currently active autopilots
     */
    @Deprecated
    private void deliverPackages(){
        //get the buffer
        Set<DeliveryPackage> packages = this.getUnassignedPackages();
        System.out.println("The nb of unassigned packages: " + packages.size());
        //also the airport map
        OverseerAirportMap airportMap = this.getAirportMap();
        //and also a a copy of the positions of the drone
        Map<String, Vector> dronePositions = this.getAutopilotPositions();
        //the position of the drones
        System.out.println("drone positions: " + dronePositions);
        //create the new delivery search
        DeliveryPlanner planning = new DeliveryPlanner(packages, airportMap, dronePositions);

        //call the planner (for now no new thread will be used)
        Map<String, List<DeliveryPackage>> deliveryScheme =  planning.call();
        assignDeliveriesToQueue(deliveryScheme);

        //add all the buffered packages to the pending package set
        this.addPackagesToDeliver(packages);
        //after this we're done
    }

    /**
     * Assigns the given delivery scheme to the delivery queues of the autopilots managed by the overseer
     * @param deliveryScheme the scheme found by the planner, now to be set as the actual delivery scheme for
     *                       the autopilots
     */
    private void assignDeliveriesToQueue(Map<String, List<DeliveryPackage>> deliveryScheme) {
        //assign the delivery planning to the drones
        Map<String, ConcurrentLinkedQueue<DeliveryPackage>> deliveryRequests = this.getDeliveryRequests();
        for(String droneID: deliveryScheme.keySet()){
            //add the deliveries sequentially to the queue (so we maintain the order)
            ConcurrentLinkedQueue<DeliveryPackage> droneDeliveryQueue = deliveryRequests.get(droneID);
            List<DeliveryPackage> droneDeliveryScheme = deliveryScheme.get(droneID);

            for(DeliveryPackage delivery: droneDeliveryScheme){
                delivery.setDeliveryDroneID(droneID);
                droneDeliveryQueue.add(delivery);
            }
        }
    }

    /**
     * Getter for the positions of the active autopilots, copies the map of inputs maintained by the autopilots
     * into a map with as key the drone ID and as value the current position of that drone
     * This method is also synchronized so no changes can be made to the map during the copying
     * after that we can use the copy to start the package distribution
     * @return a map with as key the id of the drone and as value the position of the drone linked to the ID
     */
    private synchronized Map<String, Vector> getAutopilotPositions(){
        Map<String, AutopilotInputs_v2> activeAutopilots = this.getActiveAutopilots();
        Map<String, Vector> autopilotPositions = new HashMap<>();
        for(String droneID: activeAutopilots.keySet()){
            //first get the inputs associated with the ID
            AutopilotInputs_v2 inputs = activeAutopilots.get(droneID);
            //then extract the inputs
            Vector position = extractPosition(inputs);
            //then put the positions in the map
            autopilotPositions.put(droneID, position);
        }
        //return the map
        return autopilotPositions;
    }

    /**
     * Gets the undelivered packages per drone
     * All the queues for each drone are converted into a list (with the same order)
     * containing the deliveries for each drone
     * @return a map containing the drone ID as the keys and the corresponding deliveries as lists for the value
     */
    private synchronized Map<String, List<DeliveryPackage>> getUndeliveredPackagesPerDrone(){
        //first get for each drone the packages that it still needs to deliver
        Map<String, ConcurrentLinkedQueue<DeliveryPackage>> assignedDeliveries = this.getDeliveryRequests();
        //initialize the map that will contain the undelivered packages
        Map<String, List<DeliveryPackage>> deliveriesPerDrone = new HashMap<>();

        for(String droneID:  assignedDeliveries.keySet()){
            ConcurrentLinkedQueue<DeliveryPackage> droneQueue = assignedDeliveries.get(droneID);
            //convert the queue to a list
            List<DeliveryPackage> deliveries = new LinkedList<>(droneQueue);
            deliveriesPerDrone.put(droneID, deliveries);
        }

        return deliveriesPerDrone;
    }

    /**
     * Gets all the unassigned packages currently in the to deliver set
     * --> call may be synchronized in future to be absolutely sure that no packages were added while
     *     scanning for unassigned ones
     * @return a set of all the packages that do not have a drone assigned to deliver them
     */
    private synchronized Set<DeliveryPackage> getUnassignedPackages(){
        Set<DeliveryPackage> allPackages = this.getSubmittedPackages();
        //filter for all the packages that have no drone assigned yet to deliver them
        Set<DeliveryPackage> unassignedPackages =  allPackages.stream()
                                                   .filter(delivery -> delivery.getDeliveryDroneID() == null)
                                                   .collect(Collectors.toSet());
        //return that set
        return unassignedPackages;
    }

    /**
     * Defines the parameters for all airports
     * @param length the length of the airport
     * @param width the width of the airport
     */
    @Override
    public void defineAirportParams(float length, float width) {
        //create the airport map
        this.airportMap = new OverseerAirportMap(width, length);
    }

    /**
     * Define a new airport located  at the given location with the specified heading for runway zero
     * @param centerX the x-center coordinate
     * @param centerZ the y-center coordinate
     * @param centerToRunway0X the pointer of the first runway
     * @param centerToRunway0Z the pointer of the second runway
     */
    @Override
    public void defineAirport(float centerX, float centerZ, float centerToRunway0X, float centerToRunway0Z) {
        OverseerAirportMap airportMap = this.getAirportMap();
        //convert the center coordinate to a vector
        Vector center = new Vector(centerX, 0, centerZ);
        //convert the heading to a vector
        Vector heading = new Vector(centerToRunway0X, 0, centerToRunway0Z);
        airportMap.addAirport(center, heading);
    }

    /**
     * Defines a drone at the given airport standing at the given gate and pointing to the given runway
     * @param airport the airport number to place the drone at
     * @param gate the gate to place the drone at
     * @param pointingToRunway the runway the drone points to
     * @param config the configuration of the drone
     */
    @Deprecated
    @Override
    public void defineDrone(int airport, int gate, int pointingToRunway, AutopilotConfig config) {

    }

    /**
     * Notifies the given drone that a certain time has passed
     * @param drone
     * @param inputs
     * note: we will not use this method because we're going full multithread
     */
    @Deprecated
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
     * Adds a package to the delivery buffer waiting for delivery (they will later be added to a drone
     * if one drone has no packages to deliver
     * @param fromAirport the sender airport
     * @param fromGate the sender gate to retrieve the package from
     * @param toAirport the destination airport
     * @param toGate the destination gate for the package
     */
    @Override
    public synchronized void deliverPackage(int fromAirport, int fromGate, int toAirport, int toGate) {
        //create the package
        DeliveryPackage delivery = new DeliveryPackage(fromAirport, fromGate, toAirport, toGate);
        this.addPackageToDeliver(delivery);
    }

    /**
     * Notifies the simulation has ended, breakdown all activity (close the threads)
     */
    @Override
    public void simulationEnded() {
        this.setSimulationEnded();
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
        //check if the drone was already present in the overseer
        if(activeAutopilots.get(autopilotId ) == null){
            //assign it an cruising altitude
            setCruisingAltitude(autopilotId);
            //assign a queue
            setDeliveryQueue(autoPilot);

        }
        //replace the old entry
//        System.out.println("Autopilot ID: " + autopilotId + ", Autopilot location: " + extractPosition(inputs));
        activeAutopilots.put(autopilotId, inputs);
    }

    /**
     * Getter for the queue containing the data needed for the autopilot
     * @param autoPilot the autopilot to extract the delivery requests for
     * @return the Queue of pending requests
     */
    public ConcurrentLinkedQueue<DeliveryPackage> getDeliveryRequest(AutoPilot autoPilot){
        //get the ID from the autopilot that is invoking the getter
        String value = autoPilot.getID();
        //retrieve the corresponding queue for the drone
        ConcurrentLinkedQueue<DeliveryPackage> queue = this.getDeliveryRequests().get(value);
        return queue;
    }

    /**
     * Assigns a delivery queue to the given autopilot
     * warning if invoked when the queue already exists, it will be erased
     * @param autopilot the autopilot to create a queue for
     */
    private void setDeliveryQueue(AutoPilot autopilot){
        String autopilotID = autopilot.getID();
        this.getDeliveryRequests().put(autopilotID, new ConcurrentLinkedQueue<>());
    }

    /**
     * Checks if there are any drones with empty queues
     * @return true if there is a drone with an empty queue
     */
    private boolean hasIdleDrone(){
        //get the map of all the queues
        Map<String, ConcurrentLinkedQueue<DeliveryPackage>> deliveryQueues = this.getDeliveryRequests();
        //iterate trough all the queues and check if one is empty
        for(String droneID: deliveryQueues.keySet()){
            //get the queue associated with the id
            ConcurrentLinkedQueue<DeliveryPackage> queue = deliveryQueues.get(droneID);
            //check if empty
            if(queue.isEmpty()){
                return true;
            }
        }

        return false;
    }

    /**
     * Getter for the airport where the autopilot is currently at
     * gives null if the drone is not present on an airport (should have crashed by now)
     * @param autopilot the autopilot to get the airport for
     * @return the MapAirport object containing all the information needed for the autopilot
     * --> the position used by the method is the position submitted by the autopilot during the communication
     */
   public MapAirport getAirportAt(AutoPilot autopilot){
        //get the position of the autopilot
       String autopilotID = autopilot.getID();
       AutopilotInputs_v2 inputs = this.getActiveAutopilots().get(autopilotID);
       Vector position = extractPosition(inputs);
       OverseerAirportMap airportMap = this.getAirportMap();
       return airportMap.getAirportAt(position);
   }

    /**
     * Getter for the airport with the requested ID, if no such airport exists, return null
     * @param airportID the id used to get the airport
     * @return the airport with the same id as the argument, returns null if no such airport exists
     */
   public MapAirport getAirportByID(int airportID){
       OverseerAirportMap map = this.getAirportMap();
       return map.getAirport(airportID);
   }

    /**
     * Extracts the position from the inputs received by the autopilots
     * @param inputs the inputs used to extract the position from
     * @return a vector containing the position expressed in the inputs
     */
   public static Vector extractPosition(AutopilotInputs_v2 inputs){
       float xPos = inputs.getX();
       float yPos = inputs.getY();
       float zPos = inputs.getZ();

       return new Vector(xPos, yPos, zPos);
   }

    /**
     * Getter for the cruising altitude of the specified autopilot
     * @param autopilot the autopilot to retrieve the cruising altitude for
     * @return the cruising altitude for the provided autopilot
     */
    public float getCruisingAltitude(AutoPilot autopilot){
        ConcurrentMap<String, Float> altitudeMap = this.getCruisingAltitudes();
        String autopilotID = autopilot.getID();
        return altitudeMap.get(autopilotID);
    }

    /**
     * Determines a cruising altitude for the given drone (plz don't use if the drone already has an assigned altitude)
     * @param autopilotID the id used to identify the drone in the altitude list
     */
    private void setCruisingAltitude(String autopilotID){
        //get the max of all the values
        ConcurrentMap<String, Float> altitudeMap = this.getCruisingAltitudes();
        int numberOfDrones = altitudeMap.keySet().size();
        Set<Float> altitudes = new HashSet<>(altitudeMap.values());
        Optional<Float> maxAltitudeOpt = altitudes.stream().reduce((a, b) -> a >= b ? a : b );
        float assignedAltitude;

        if(!maxAltitudeOpt.isPresent()){
            assignedAltitude = BASE_ALTITUDE;
        }else{
            assignedAltitude = maxAltitudeOpt.get() + ALTITUDE_DELTA;
        }

        altitudeMap.put(autopilotID, assignedAltitude);
    }

    /**
     * Tries to reserve the airport with the corresponding airport ID for the specified autopilot
     * returns false if such reservation was not possible
     * @param autopilot the autopilot to reserve the airport for
     * @param airportID the id of the airport the drone wants to reserve
     * @return true if the overseer was able to reserve the airport
     *         false if the airport is already reserved by another drone
     */
    public synchronized boolean reserveAirport(AutoPilot autopilot, int airportID){
        OverseerAirportMap airportMap = this.getAirportMap();
        String autopilotID = autopilot.getID();
        return airportMap.reserveAirport(autopilotID, airportID);
    }

    /**
     * Cancels the reservation made by the specified autopilot.
     * A reservation cancellation makes the airport available for other drones to land on
     * @param autopilot the autopilot to release the reservation for
     */
    public synchronized void releaseAirport(AutoPilot autopilot){
        OverseerAirportMap airportMap = this.getAirportMap();
        String autopilotID = autopilot.getID();
        airportMap.cancelReservation(autopilotID);
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
     * Setter for the operational autopilots only to be used by the overseer
     * @param autopilotData the data to set
     */
    private void setActiveAutopilots(ConcurrentHashMap<String, AutopilotInputs_v2> autopilotData){
        this.activeAutopilots = autopilotData;
    }

    /**
     * Getter for the map that stores all the delivery requests based on the ID
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryPackage>> getDeliveryRequests(){
        return this.deliveryRequests;
    }

    /**
     * Getter for the cruising altitudes of all drones governed by the overseer (one overseer to rule them all)
     * @return a map containing the cruising altitudes for each drone (ID)
     */
    private ConcurrentMap<String, Float> getCruisingAltitudes() {
        return cruisingAltitudes;
    }

    /**
     * Getter for the map that contains all the airports currently active in the world
     * @return an overseerAirportMap object containing all the information about the airports for the overseer
     */
    private OverseerAirportMap getAirportMap() {
        return airportMap;
    }


    /**
     * Getter for all the packages that have been assigned to be delivered by a certain drone
     * @return the set of all packages that need to be delievered
     */
    @Override
    public synchronized Set<DeliveryPackage> getAssignedPackages() {
        //get the set of all the packages
        Set<DeliveryPackage> packagesToDeliver = this.getSubmittedPackages();
        //get all the packages that have drone assigned to deliver them
        Set<DeliveryPackage> assignedPackages = packagesToDeliver.stream()
                                                .filter(delivery -> delivery.getDeliveryDroneID() != null)
                                                .collect(Collectors.toSet());
        return assignedPackages;
    }

    @Override
    public synchronized Set<DeliveryPackage> getAllUndeliveredPackages() {
        Set<DeliveryPackage> packagesToDeliver = this.getSubmittedPackages();
        //get all the packages that need to be delivered
        Set<DeliveryPackage> undelivered = packagesToDeliver.stream()
                .filter(delivery -> !delivery.isDelivered())
                .collect(Collectors.toSet());

        return undelivered;
    }

    @Override
    public synchronized Set<DeliveryPackage> getAllUndeliveredAssignedPackages() {
        Set<DeliveryPackage> packagesToDeliver = this.getSubmittedPackages();
        //get all the packages that have drone assigned and need to be delivered
        Set<DeliveryPackage> assignedAndUndeliveredPackages = packagesToDeliver.stream()
                .filter(delivery -> delivery.getDeliveryDroneID() != null&&!delivery.isDelivered())
                .collect(Collectors.toSet());
        return assignedAndUndeliveredPackages;
    }

    /**
     * Getter for the packages that have been submitted to the overseer, this set contains
     * 1. packages that have been submitted but not yet assigned
     * 2. packages that have been assigned but not yet delivered
     * 3. packages that have been delivered
     * @return the packages pending to be delivered
     */
    @Override
    public Set<DeliveryPackage> getSubmittedPackages() {
        return submittedPackages;
    }

    /**
     * Adds extra pending packages to the already existing set of submitted packages
     * @param packages the packages to be added to the pending set
     */
    private void addPackagesToDeliver(Set<DeliveryPackage> packages) {
        this.submittedPackages.addAll(packages);
    }

    /**
     * Adds the specified package to the set of submitted packages
     * @param delivery the delivery to add to the set
     */
    private void addPackageToDeliver(DeliveryPackage delivery){
        this.getSubmittedPackages().add(delivery);
    }

    /**
     * Getter for the number of autopilots that area added to the world upon initialization
     * we use this to check for package allocation
     * @return the number of autopilots initially in the world
     */
    private int getInitNbOfAutopilots() {
        return initNbOfAutopilots;
    }

    /**
     * Getter for the planner used by the autopilot overseer to distribute the packages amongst the drones
     * @return the planner used by the overseer
     */
    private DeliveryPlanner getPlanner() {
        return planner;
    }

    /**
     * Setter for the planner used by the overseer, used to distribute the packages amongst the drones
     * @param planner the planner to set
     */
    private void setPlanner(DeliveryPlanner planner) {
        this.planner = planner;
    }

    /**
     * The hash map that stores all the active autopilots in the world
     * String contains the ID and inputs the current inputs of the autopilot
     */
    private ConcurrentMap<String, AutopilotInputs_v2> activeAutopilots = new ConcurrentHashMap<>();

    /**
     * Getter for the number of autopilots that are active upon creation of the world
     */
    private int initNbOfAutopilots;

    /**
     * A map used to store the all the request made to the overseer, the autopilots can query for their requests
     * to the broadcast
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryPackage>> deliveryRequests = new ConcurrentHashMap<String, ConcurrentLinkedQueue<DeliveryPackage>>();

    /**
     * A map containing the cruising altitude of each autopilot identified by the identifier string
     */
    private ConcurrentMap<String, Float> cruisingAltitudes = new ConcurrentHashMap<>();

    /**
     * The airport map used by the overseer to assign packages to the autopilots
     * and for the governed autopilots to get the landing info from
     */
    private OverseerAirportMap airportMap;

    /**
     * The packages that are submitted to the overseer, the set contains the following kind of packages
     * 1. submitted but not yet assigned for delivery
     * 2. assigned but not yet delivered
     * 3. delivered
     */
    private Set<DeliveryPackage> submittedPackages = new HashSet<>();

    /**
     * The planner used for distributing/assigning the packages to the drones (as optimally as possible)
     */
    private DeliveryPlanner planner;

    /**
     * The base altitude to assign to the drones (incremented from here)
     */
    private final static float BASE_ALTITUDE = 30f;

    /**
     * The minimal cruising altitude difference between two drones
     */
    private final static float ALTITUDE_DELTA = 15f;

    /*
    Some flags with their setters
     */

    /**
     * Getter for the simulation ended flag
     * used to signal to the overseer that the simulation has ended and that it may wrap up its activities
     * (e.g. planning the delivery sequence for the drones)
     * @return the flag value (true if the simulation has ended, false if not)
     */
    private boolean isSimulationEnded() {
        return simulationEnded;
    }

    /**
     * Used to toggle the simulation ended flag and signal to the overseer that the simulation has been ended
     * (for more info see the getter)
     */
    private void setSimulationEnded() {
        this.simulationEnded = true;
    }

    private boolean simulationEnded = false;


}
