package TestbedAutopilotInterface.Overseer;

import AutopilotInterfaces.*;
import internal.Autopilot.AutoPilot;
import internal.Helper.Vector;

import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;

//sequence:
//generate a new autopilot testbed with an integrated queue for delivery messages and a concurrentmap to write their state to
//(we can also use the broadcast queue to enter the states of all the drones

//note need a map of the world to navigate the drones and make decisions on what drone does what
//TODO check if drones have crashed during the simulation, delete them if needed
//--> the overseer must set the assigned drone
//TODO change the autopilot state saved by the overseer to autopilotInfo saved by the overseer (used for collision detection)
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
    public  Void call() throws Exception {
        //only start the delivery if all drones to spread the deliveries across have been added
        while(this.getAutopilotInfoCenter().getSize() != this.getInitNbOfAutopilots()||!this.autopilotsInitialized()){
            //do nothing
//            System.out.println("nb of autopilots: " + this.getAutopilotInfoCenter().allAutopilotsInitialized());
        }

        System.out.println("ready to rumble");
        //call the main loop
        overseerMainLoop();

        return null;
    }

    private void overseerMainLoop(){
        //get the airport map
        OverseerAirportMap airportMap = this.getAirportMap();
        //create the delivery planner
        DeliveryPlanner planner = new DeliveryPlanner(airportMap);
        this.setPlanner(planner);

        AutopilotInfoCenter infoCenter = this.getAutopilotInfoCenter();

        //loop until the simulation has been ended by the testbed
        while(!this.isSimulationEnded()){
           //distribute the packages
        distributePackages();
            //then wait until one of the drone's queues is empty
            while(infoCenter.getNbOfIdleDrones() == 0 || !hasPackagesAdded()){
                //TODO we may also use the autopilots to wake the overseer thread if they spot that their queue is empty
                //TODO make mechanism that the planner is not invoked if there are no packages to deliver
            }
        }
    }

    /**
     * Distributes the packages that have not been assigned yet to a drone
     * starts the delivery planner (may require some work --> beam search)
     */
    private synchronized void distributePackages(){
        long startMillis = System.currentTimeMillis();
        //get the planner
        DeliveryPlanner planner = this.getPlanner();
        //initialize the search
        //first get the list of all packages
        Set<PlannerDelivery> submittedDeliveries = this.getSubmittedPlannerDeliveries();

        //get the map of the drones and their current assigned deliveries
        Map<String, List<PlannerDelivery>> assignedDeliveries = this.getUndeliveredPackagesPerDrone();
        //get the map of the drones and their current positions
        AutopilotInfoCenter infoCenter = this.getAutopilotInfoCenter();
        Map<String, Vector> autopilotPositions = infoCenter.getAutopilotPositions();
        //initialize the search
        planner.initializeSearch(submittedDeliveries, assignedDeliveries, autopilotPositions);
        //execute the search
        Map<String, List<PlannerDelivery>> deliveryScheme = planner.executeSearch();
        long endMillis = System.currentTimeMillis();
        System.out.println("totalTime in ms = " + (endMillis-startMillis));
        //and apply the found scheme
//        System.out.println();
        planner.printSchedule(deliveryScheme, getAirportMap());
        assignDeliveriesToQueue(deliveryScheme);
        notifyAll();

    }

    /**
     * Checks if all autopilots are initialized by filtering for all autopilots that do not have the init
     * flight reported as the state of the autopilot (meaning they're ready to fly)
     * @return true if all the autopilots are in a different state than init-flight
     */
    private boolean autopilotsInitialized(){
        //get all the states
        AutopilotInfoCenter infoCenter = this.getAutopilotInfoCenter();
        return infoCenter.allAutopilotsInitialized();
//        Set<AutopilotInfo> infoSet = new HashSet<>(infoMap.values());
//        Set<AutopilotInfo> initSet = infoSet.stream().filter(info -> info.getAutopilotState() != AutopilotState.INIT_FLIGHT).collect(Collectors.toSet());
//        return initSet.size() == infoSet.size();
    }

    /**
     * Assigns the given delivery scheme to the delivery queues of the autopilots managed by the overseer
     * @param deliveryScheme the scheme found by the planner, now to be set as the actual delivery scheme for
     *                       the autopilots
     */
    private void assignDeliveriesToQueue(Map<String, List<PlannerDelivery>> deliveryScheme) {
        //assign the delivery planning to the drones
        Map<String, ConcurrentLinkedQueue<DeliveryPackage>> deliveryRequests = this.getDroneDeliveryMap();
        for(String droneID: deliveryScheme.keySet()){
            //add the deliveries sequentially to the queue (so we maintain the order)
            ConcurrentLinkedQueue<DeliveryPackage> droneDeliveryQueue = deliveryRequests.get(droneID);
            List<PlannerDelivery> droneDeliveryScheme = deliveryScheme.get(droneID);

            for(PlannerDelivery delivery: droneDeliveryScheme){
                long sequenceNumber = getNextSequenceNumber();
                delivery.setDeliveryDroneID(droneID);
                delivery.setSequenceNumber(sequenceNumber);
                droneDeliveryQueue.add((DeliveryPackage)delivery);
            }
        }
    }

//    /**
//     * Getter for the positions of the active autopilots, copies the map of inputs maintained by the autopilots
//     * into a map with as key the drone ID and as value the current position of that drone
//     * This method is also synchronized so no changes can be made to the map during the copying
//     * after that we can use the copy to start the package distribution
//     * @return a map with as key the id of the drone and as value the position of the drone linked to the ID
//     */
//    private synchronized Map<String, Vector> getAutopilotPositions(){
//        Map<String, AutopilotInfo> infoEntries = this.getAutopilotInfo();
//        Map<String, Vector> autopilotPositions = new HashMap<>();
//        for(String droneID: infoEntries.keySet()){
//            //first get the inputs associated with the ID
//            AutopilotInfo info = infoEntries.get(droneID);
//            //then extract the inputs
//            Vector position = info.getCurrentPosition();
//            //then put the positions in the map
//            autopilotPositions.put(droneID, position);
//        }
//        //return the map
//        return autopilotPositions;
//    }

    /**
     * Gets the undelivered packages per drone
     * All the queues for each drone are converted into a list (with the same order)
     * containing the deliveries for each drone
     * @return a map containing the drone ID as the keys and the corresponding deliveries as lists for the value
     */
    private synchronized Map<String, List<PlannerDelivery>> getUndeliveredPackagesPerDrone(){
        //adjust the flag (no packages can be added during this call
        this.setPackagesAdded(false);

        //first get for each drone the packages that it still needs to deliver
        Map<String, ConcurrentLinkedQueue<DeliveryPackage>> assignedDeliveries = this.getDroneDeliveryMap();
        //initialize the map that will contain the undelivered packages
        Map<String, List<PlannerDelivery>> deliveriesPerDrone = new HashMap<>();

        for(String droneID:  assignedDeliveries.keySet()){
            ConcurrentLinkedQueue<DeliveryPackage> droneQueue = assignedDeliveries.get(droneID);
            //convert the queue to a list
            List<PlannerDelivery> deliveries = new LinkedList<>(droneQueue);
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
    private synchronized Set<PlannerDelivery> getUnassignedPackages(){
        Set<PlannerDelivery> allPackages = this.getSubmittedPlannerDeliveries();
        //filter for all the packages that have no drone assigned yet to deliver them
        Set<PlannerDelivery> unassignedPackages =  allPackages.stream()
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
//        System.out.println(fromAirport +" " + toAirport);
        DeliveryPackage delivery = new DeliveryPackage(fromAirport, fromGate, toAirport, toGate);
        this.addPackageToDeliver(delivery);
        this.setPackagesAdded(true);
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
     * @param autopilot the autopilot updating his status (used for identification)
     * @param autopilotInfo the information object created by the autopilot, containing all data needed for
     *                      the collision detection
     * note: has to be called every time when using the autopilot
     */
    public void autopilotStatusUpdate(AutoPilot autopilot, AutopilotInfo autopilotInfo){
        //TODO check if concurrency issues arise, use a check-in first and lock the map
        //get the id
        String autopilotId = autopilot.getID();
        //get the map we write to
        AutopilotInfoCenter infoCenter = this.getAutopilotInfoCenter();
        //check if the drone was already present in the overseer
        if(!infoCenter.hasEntry(autopilot)){
            //assign it an cruising altitude
            setCruisingAltitude(autopilotId);
            //assign a queue
            setDeliveryQueue(autopilot);

        }
        //replace the old entry
//        System.out.println("Autopilot ID: " + autopilotId + ", Autopilot location: " + extractPosition(inputs));
        infoCenter.updateEntry(autopilot, autopilotInfo);
    }

    /**
     * Getter for the autopilot information set containing the autopilot info objects describing all the drones
     * that share the airspace with the same drone
     * @param autopilot the autopilot that has called the method, the info about this autopilot will be excluded
     * @return a set containing all the autopilot info's from drones with a different ID from the caller
     */
    public synchronized Set<AutopilotInfo> getOtherAutopilotInfo(AutoPilot autopilot){
        AutopilotInfoCenter infoCenter = getAutopilotInfoCenter();
        return infoCenter.getInfoOtherAutopilots(autopilot);
    }

    /**
     * Getter for the delivery that is currently assigned next to the autopilot
     * @param autoPilot the autopilot to extract the delivery requests for
     * @return the next autopilot delivery
     *         if the queue is empty, returns null
     */
    public AutopilotDelivery getNextDeliveryRequest(AutoPilot autoPilot){
        //get the ID from the autopilot that is invoking the getter
        String value = autoPilot.getID();
        //retrieve the corresponding queue for the drone
        ConcurrentLinkedQueue<DeliveryPackage> queue = this.getDroneDeliveryMap().get(value);
        //and return the next element
        return queue.peek() != null ? queue.poll() : null;
//        System.out.println("Elements in queue: " + (queue.peek()!=null));
//        System.out.println("returned queue element: " + delivery);

    }

    /**
     * Assigns a delivery queue to the given autopilot
     * warning if invoked when the queue already exists, it will be erased
     * @param autopilot the autopilot to create a queue for
     */
    private void setDeliveryQueue(AutoPilot autopilot){
        String autopilotID = autopilot.getID();
        this.getDroneDeliveryMap().put(autopilotID, new ConcurrentLinkedQueue<>());
    }

    /**
     * Getter for the airport where the autopilot is currently at
     * gives null if the drone is not present on an airport (should have crashed by now)
     * @param autopilot the autopilot to get the airport for
     * @return the MapAirport object containing all the information needed for the autopilot
     * --> the position used by the method is the position submitted by the autopilot during the communication
     */
   public synchronized MapAirport getAirportAt(AutoPilot autopilot){
        //get the position of the autopilot
       AutopilotInfo info = this.getAutopilotInfoCenter().getEntry(autopilot);
       Vector position = info.getCurrentPosition();
       OverseerAirportMap airportMap = this.getAirportMap();
       return airportMap.getAirportAt(position);
   }

    /**
     * Getter for the airport with the requested ID, if no such airport exists, return null
     * @param airportID the id used to get the airport
     * @return the airport with the same id as the argument, returns null if no such airport exists
     */
   public synchronized MapAirport getAirportByID(int airportID){
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
    private synchronized void setCruisingAltitude(String autopilotID){
        //get the max of all the values
        ConcurrentMap<String, Float> altitudeMap = this.getCruisingAltitudes();
        int numberOfDrones = altitudeMap.keySet().size();
        Set<Float> altitudes = new HashSet<>(altitudeMap.values());
//        System.out.println(altitudeMap);

        float assignedAltitude;

        if(altitudes.size() == 0){
            assignedAltitude = BASE_ALTITUDE;
        }else{
            Float maxAltitudeOpt = Collections.max(altitudes);
            assignedAltitude = maxAltitudeOpt + ALTITUDE_DELTA;
        }

        altitudeMap.put(autopilotID, assignedAltitude);
//        System.out.println(altitudeMap);
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
     * note: the autopilot should call this method every time it has finished the takeoff phase
     *       even if it hasn't made a reservation to begin with (on init, the drones are all standing on the airport)
     *       the method just has no effect if there was no reservation for the airport (fail safe)
     */
    public synchronized void releaseAirport(AutoPilot autopilot){
        OverseerAirportMap airportMap = this.getAirportMap();
        String autopilotID = autopilot.getID();
        airportMap.cancelReservation(autopilotID);
    }


    /**
     * Getter for the autopilot information center, is used to save all the states that are reported by the autopilots
     * with the same overseer
     */
    private AutopilotInfoCenter getAutopilotInfoCenter() {
        return infoCenter;
    }

    /**
     * Getter for the map that contains the ID's of the drones with as value the deliveries that were assigned to them
     * (in order)
     * @return the map containing the drone id's as keys and the queue with the packages to deliver assigned to them
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryPackage>> getDroneDeliveryMap(){
        return droneDeliveryMap;
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
    public synchronized Set<WorldDelivery> getAssignedWorldDeliveries() {
        //get the set of all the packages
        Set<WorldDelivery> packagesToDeliver = this.getSubmittedWorldDeliveries();
        //get all the packages that have drone assigned to deliver them
        Set<WorldDelivery> assignedPackages = packagesToDeliver.stream()
                                                .filter(delivery -> delivery.getDeliveryDroneID() != null)
                                                .collect(Collectors.toSet());
        return assignedPackages;
    }

    @Override
    public synchronized Set<WorldDelivery> getAllUndeliveredWorldDeliveries() {
        Set<WorldDelivery> packagesToDeliver = this.getSubmittedWorldDeliveries();
        //get all the packages that need to be delivered
        Set<WorldDelivery> undelivered = packagesToDeliver.stream()
                .filter(delivery -> !delivery.isDelivered())
                .collect(Collectors.toSet());

        return undelivered;
    }

    @Override
    public synchronized Set<WorldDelivery> getAllUndeliveredAssignedWorldDeliveries() {
        Set<WorldDelivery> packagesToDeliver = this.getSubmittedWorldDeliveries();
        //get all the packages that have drone assigned and need to be delivered
        Set<WorldDelivery> assignedAndUndeliveredPackages = packagesToDeliver.stream()
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
     * note: if the set itself is altered (eg packages deleted) no harm is done to the set of the overseer
     * since the set itself is a copy, only the packages can be altered by the methods provided in the interface
     */
    @Override
    public Set<WorldDelivery> getSubmittedWorldDeliveries() {
        return new HashSet<WorldDelivery>(submittedPackages);
    }

    /**
     * Getter for all the submitted packages
     * this method should be used in the overseer for accessing and changing the submitted packages
     * --> the other method is only for the end of the bargain with the world
     * @return the set containing all the submitted packages
     */
    private Set<DeliveryPackage> getSubmittedDeliveryPackages(){
        return submittedPackages;
    }

    /**
     * Getter for the planner deliveries, this method encapsulates the packages needed for the planner
     * to plan all the deliveries
     * @return a set containing all the deliveries that were submitted
     */
    public Set<PlannerDelivery> getSubmittedPlannerDeliveries(){
        return new HashSet<PlannerDelivery>(submittedPackages);
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
        this.getSubmittedDeliveryPackages().add(delivery);
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
     * Getter for the next sequence number to be assigned, after the call the sequence number is auto incremented
     * such that no two packages can have the same sequence number during the simulation
     * --> only call once when assigning the sequence number to a single package, two calls will result in a different
     *     sequence number
     * @return the next sequence number
     */
    private long getNextSequenceNumber() {
        return nextSequenceNumber++;
    }

    /**
     * An object that contains all the info about the drones that are currently active in the world
     */
    private AutopilotInfoCenter infoCenter = new AutopilotInfoCenter();

    /**
     * Getter for the number of autopilots that are active upon creation of the world
     */
    private int initNbOfAutopilots;

    /**
     * A map used to store the all the request made to the overseer, the autopilots can query for their requests
     * to the broadcast
     */
    private ConcurrentMap<String, ConcurrentLinkedQueue<DeliveryPackage>> droneDeliveryMap = new ConcurrentHashMap<String, ConcurrentLinkedQueue<DeliveryPackage>>();

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
     * The sequence number that will be assigned to the next package that will be delivered
     * The sequence number is assigned to a package that will be delivered by a drone to tell the testbed
     * which packages (if they have the same source airport) should be assigned first to the drone
     * meaning that the package with the lower sequence number must be passed first to the drone
     * note that there is no inherent relation between the sequence number of packages that must be delivered
     * by different drones, if the sequence number of a package to be delivered by drone A is lower than the one
     * to be delivered by drone B doesn't mean that the package of A will be delivered earlier. The sequence number
     * only indicates the order in which the packages must be delivered for a SINGLE drone
     */
    private long nextSequenceNumber = 0L;

    /**
     * The base altitude to assign to the drones (incremented from here)
     */
    private final static float BASE_ALTITUDE = 100f;

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

    /**
     * Getter for the flag that is set if packages were added to the overseer (and are not assigned yet)
     * @return true if the flag is set to true
     * --> flag is set to true on every invocation of deliver package
     * --> the flag is set to false on every invocation of distribute packages
     */
    private boolean hasPackagesAdded() {
        return packagesAdded;
    }

    /**
     * Setter for the packages added flag --> see getter for more info
     * @param packagesAdded the value for the flag
     */
    private void setPackagesAdded(boolean packagesAdded) {
        this.packagesAdded = packagesAdded;
    }

    /**
     * Setter for the package added flag
     * --> is set false by the package distributor and is set true by the add package method
     */
    private boolean packagesAdded = false;


}
