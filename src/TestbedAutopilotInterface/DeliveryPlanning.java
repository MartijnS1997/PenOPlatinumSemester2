package TestbedAutopilotInterface;

import AutopilotInterfaces.AutopilotInputs_v2;
import internal.Helper.Vector;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 27/03/2018.
 * A planner for the package assignment of the drones
 * --> cost search with iterative deepening
 * --> in the first iteration of creating the algorithm we deepCopy ALL the drone state on every new node
 * --> first test: hill climbing 1 (no iterative deepening, first check if basics work)
 */
public class DeliveryPlanning implements Callable<Map<String, List<DeliveryPackage>>>{



    //Todo add a method to resume search based on the current assigned deliveries
    //--> maybe clear the heuristic values of the drones? (case where we have an idle drone)
    //--> we can re-init root based on the queues that are currently present in the overseer
    //    and re calculate the heuristic values

    /**
     * Constructor for a delivery planning, implements search to find a good/optimal delivery allocation for the
     * drones (currently implementing hill climbing 1)
     * @param deliveryPackages the packages to deliver
     * @param airportMap the map containing all the airports of the overseer
     * @param activeDrones a map containing the drone ID's as keys and the current location as values
     */
    public DeliveryPlanning(Collection<DeliveryPackage> deliveryPackages,
                            OverseerAirportMap airportMap, Map<String, AutopilotInputs_v2> activeDrones){

        //first add the packages and overseer map to the instance
        this.airportMap = airportMap;
        this.deliveries = deliveryPackages;
        //then init the drones
        initRoot(activeDrones);
    }

    /**
     * adds the the provided packages to the search queue, when the delivery planning is called again we will resume
     * the search with the packages added (previous allocations aren't changed because we want to service the previous
     * submitted packages first)
     * @param packages a collection of packages to add
     */
    public void addPackages(Collection<DeliveryPackage> packages){
        //we need to clear the queue if we want to continue the search with the same previous allocation
        //of the packages
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        //get the first node --> the new root for our search
        DeliveryNode newRoot= this.getFirstNode();
        //add the delivery to the root
        newRoot.addPackagesToDeliver(packages);
        //clear the nodeQueue
        nodeQueue.clear();
        //add the root to the queue
        nodeQueue.add(newRoot);
        //we're done
    }

    @Override
    public Map<String, List<DeliveryPackage>> call() throws Exception {
        return initSearch();
    }

    /**
     * Starts the search for the optimal delivery method for all the packages
     * @return a map with as keys the id's of the drones and value a list containing the packages that it needs
     *         to deliver, the first package to deliver is at index 0
     */
    private Map<String, List<DeliveryPackage>> initSearch(){

       while(!goalReached()){
           //get the first node
           DeliveryNode firstNode = this.removeFirstNode();
           //expand the first node
           List<DeliveryNode> children = expandNode(firstNode);
           addNodesHillClimb(children);
//           addNodesTotalCost(children);
       }
       //the goal is reached, report the first node
       DeliveryNode goalNode = this.getFirstNode();
       //print the total time needed:
       System.out.println("\nDeliveryTime: "+ goalNode.getCostValue()+"\n");
       //return a map with as key the drone id and an ordered list for the deliveries that it has to make
       return convertNodeToDeliveryScheme(goalNode);
    }

    /**
     * Converts the provided node into a map containing the drone ID as the key and its assigned deliveries (in order)
     * to a map (to form as schedule)
     * @param node the node to convert
     * @return a map with as key the drone ID and value the assigned deliveries (first one to deliver at first index)
     */
    private static Map<String, List<DeliveryPackage>> convertNodeToDeliveryScheme(DeliveryNode node){
        //get the drones of the node
        Map<String, DeliveryDrone> drones = node.getDroneMap();
        Map<String, List<DeliveryPackage>> deliverySchedule = new HashMap<>();
        //iterate trough all the drones
        for(DeliveryDrone drone: drones.values()){
           //get the deliveries assigned to the drone
            List<DeliveryPackage> assignedDeliveries = drone.getAssignedDeliveries();
            //get the ID of the drone
            String droneID = drone.getDroneID();
            //add the key - value to the map
            deliverySchedule.put(droneID, assignedDeliveries);
        }

        //return the schedule
        return deliverySchedule;
    }

    private boolean goalReached(){
        //get the first element, if it does not contain any packages to deliver, we've reached our goal
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        DeliveryNode firstNode = nodeQueue.get(0);
        //if the size of the packages to deliver is zero, we're finished
        return firstNode.getPackagesToDeliver().size() == 0;
    }

    /**
     * Removes the head of the list while returning the value that was located at the head
     * @return the delivery node that was at the head of the queue
     */
    private DeliveryNode removeFirstNode(){
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        return nodeQueue.remove(0);
    }

    /**
     * Get the first node in the queue
     * @return the first node
     */
    private DeliveryNode getFirstNode(){
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        return nodeQueue.get(0);
    }

    /**
     * Initializes the root node by adding all the drones that are currently active in the overseer
     * @param activeDrones a map with key the id of the drone and value the current state
     */
    private void initRoot(Map<String, AutopilotInputs_v2> activeDrones){
        //create a map to put the newly generated drones in
        Map<String, DeliveryDrone> drones  = new HashMap<>();

        //then iterate trough all the entries and generate the delivery drones
        for(String droneID: activeDrones.keySet()){
            //get the inputs associated with the drone
            AutopilotInputs_v2 inputs = activeDrones.get(droneID);
            //extract the position from the inputs
            Vector position = AutopilotOverseer.extractPosition(inputs);
            //create the drone
            DeliveryDrone deliveryDrone = new DeliveryDrone(position, droneID);
            //add to the drone map
            drones.put(droneID, deliveryDrone);
        }

        //get the packages that need to be delivered
        Collection<DeliveryPackage> deliveries = this.getDeliveries();

        //create the root of the search tree
        DeliveryNode root = new DeliveryNode(drones,deliveries);
        //add the root to the queue
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        nodeQueue.add(root);

    }

    /**
     * Expands the given node
     * for each drone-delivery combination a new child node is created
     * @param parentNode the node to expand
     * @return a list containing all the children of the parent
     */
    private List<DeliveryNode> expandNode(DeliveryNode parentNode){

        //get all the drones in the node and all the packages that still need to be delivered
        Map<String, DeliveryDrone> drones = parentNode.getDroneMap();
        Set<DeliveryPackage> packages = parentNode.getPackagesToDeliver();

        //the list containing all the new nodes
        List<DeliveryNode> childNodes = new ArrayList<>();

        //for every package-drone combination create a new node & add the package
        for(DeliveryPackage delivery : packages){
            for(DeliveryDrone drone: drones.values()){

                //create the child
                DeliveryNode childNode = new DeliveryNode(drones, packages);
                //add the delivery
                childNode.addDeliveryToDrone(drone.getDroneID(), delivery);
                //add the child to the expanded node
                childNodes.add(childNode);
            }
        }

        //System.out.println("expanding node, packages to deliver: " + packages.size());
        return childNodes;

    }

    /**
     * Getter for the map that contains all the information about the airports in the world
     * @return the airport map used by the overseer
     */
    private OverseerAirportMap getAirportMap() {
        return airportMap;
    }

    /**
     * Getter for the packages that need to be delivered
     * @return a collection of delivery packages
     */
    private Collection<DeliveryPackage> getDeliveries() {
        return deliveries;
    }

    //instances used to get heuristics and delivery specs
    /**
     * The map containing all the airports in the world
     */
    private OverseerAirportMap airportMap;

    /**
     * The collection of deliveries to be made to the airports
     */
    private Collection<DeliveryPackage> deliveries;

    /**
     * Getter for the queue used to search the tree
     * @return a list of delivery nodes
     */
    private List<DeliveryNode> getNodeQueue() {
        return nodeQueue;
    }

    /**
     * adds the provided nodes to the queue, sorts them first based on their cost value in such a way
     * that the largest value will come first
     * @param deliveryNodes the nodes to be added
     */
    private void addNodesHillClimb(List<DeliveryNode> deliveryNodes){
        //first sort the nodes based on cost value
        //Sort sorts from small to large, to get the right effect we should set the largest as "smaller" than the
        //smaller one in the comparator
        deliveryNodes.sort(new Comparator<DeliveryNode>() {
            @Override
            public int compare(DeliveryNode node1, DeliveryNode node2) {
                float costNode1 = node1.getApproxCost();
                float costNode2 = node2.getApproxCost();
                //check for the values
                if (costNode1 > costNode2) {
                    //1 means set node 1 forwards (higher index)
                    return 1;
                }
                if (costNode1 < costNode2) {
                    //-1 means set node1 one backwards (lower index)  in the list (in bubble sort)
                    return -1;
                }
                //0 means let them both stay where they are (in bubble sort)
                return 0;
            }
        });

        //now that the nodes are sorted add them in front of the queue
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        nodeQueue.addAll(0, deliveryNodes);
    }

    /**
     * Implements the node adding for branch and bound total cost search (no optimisation)
     * --> added heuristic, but no pruning yet
     * Todo implement insertion for faster iterations
     * @param childNodes the nodes to add to the queue
     */
    private void addNodesTotalCost(List<DeliveryNode> childNodes){
        //add the nodes to the list, sort the list afterwards
        //TODO use insertion to quickly insert the new values
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        nodeQueue.addAll(childNodes);
        nodeQueue.sort(new Comparator<DeliveryNode>() {
            @Override
            public int compare(DeliveryNode node1, DeliveryNode node2) {
                float costNode1 = node1.getApproxCost();
                float costNode2 = node2.getApproxCost();
                //check for the values
                if (costNode1 > costNode2) {
                    //1 means set node 1 forwards (higher index)
                    return 1;
                }
                if (costNode1 < costNode2) {
                    //-1 means set node1 one backwards (lower index)  in the list (in bubble sort)
                    return -1;
                }
                //0 means let them both stay where they are (in bubble sort)
                return 0;
            }
        });

//        System.out.println("Size of queue: "+ nodeQueue.size());
//        System.out.println("First node packets delivered: " + getFirstNode().getPackagesToDeliver().size());

    }

    /**
     * The queue used to keep track of the visited and to visit nodes
     * we use a linked list so the adding of new nodes to the queue is more efficient
     */
    private List<DeliveryNode> nodeQueue = new LinkedList<>();

    /**
     * A node of our delivery search tree
     * Has the following properties:
     * --> a cost value (the maximum travel time of all the drones)
     * --> a list of all the drones and their assigned packages
     * --> a method to add a package to a given drone in the node
     */
    private class DeliveryNode{

        /**
         * Constructor for a node in the search tree, upon construction all the entries of the parent are
         * deep copied so the different layers of the three do not interfere
         * @param parentDrones a map containing the drones and their assigned packages from the parent of the node
         *                     the key is the drone ID and the value the delivery drone itself
         */
        private DeliveryNode(Map<String, DeliveryDrone> parentDrones, Collection<DeliveryPackage> packagesToDeliver) {
            //create a deep copy of the map we've received
            Map<String, DeliveryDrone> childDrones = new HashMap<>();
            for(String droneID : parentDrones.keySet()){
                //get the drone & deep copy it
                DeliveryDrone drone = parentDrones.get(droneID);
                DeliveryDrone copyDrone = drone.deepCopy();
                //add the copied drone to the child map
                childDrones.put(droneID, copyDrone);
            }

            //create shallow copy of the packages Set
            Set<DeliveryPackage> packages = new HashSet<>();
            for(DeliveryPackage delivery : packagesToDeliver){
                packages.add(delivery);
            }

            //set the copies for the node
            this.droneMap = childDrones;
            this.packagesToDeliver = packages;
        }

        /**
         * Adds packages to the packages to deliver set (used for submitting new packages)
         * @param packages the packages to deliver on top of the previously set packages
         */
        private void addPackagesToDeliver(Collection<DeliveryPackage> packages){
            Set<DeliveryPackage> packagesToDeliver = this.getPackagesToDeliver();
            packagesToDeliver.addAll(packages);
        }

        /**
         * Adds the provided package to the drone with the corresponding ID in the map
         * @param droneID the id of the drone to add the delivery for
         * @param delivery the DeliveryPackage to be added to the drone with the given ID
         */
        private void addDeliveryToDrone(String droneID, DeliveryPackage delivery){
            if(this.isCompleted){
                throw new IllegalStateException("Node already configured");
            }

            //remove the package from the set of packages to deliver
            Set<DeliveryPackage> packages = this.getPackagesToDeliver();
            packages.remove(delivery);

            //get the drone we want the package to deliver to
            Map<String, DeliveryDrone> drones = this.getDroneMap();
            DeliveryDrone drone = drones.get(droneID);
            updateDronePosition(delivery, drone);


            //add the new package to deliver
            drone.addDelivery(delivery);

            //set the added delivery to the node
            this.setCurrentDelivery(delivery);
            this.setCurrentDeliveryDroneID(droneID);

            //we're finished
            isCompleted = true;
        }

        /**
         * Updates the position of the drone and the total travel time for the deliveries
         * @param delivery the delivery to add to the drone and update it for
         * @param drone the drone to update
         */
        private void updateDronePosition(DeliveryPackage delivery, DeliveryDrone drone) {
            //calculate the new value of the drone based on distance

            //get id of the source and destination airport
            float deliveryDistance = getDeliveryDistance(delivery, drone);
            //the destination airport
            int destinationAirportID = delivery.getDestinationAirport();
            MapAirport destinationAirport = airportMap.getAirport(destinationAirportID);
            Vector destinationAirportLocation = destinationAirport.getLocation();
            //move the drone to the next airport
            drone.setPosition(destinationAirportLocation);
            //increment the time
            drone.incrementTotalTravelTime(deliveryDistance);
        }

        /**
         * Getter for the total travelling distance needed for the given drone to deliver the provided package
         * @param delivery the delivery
         * @param drone the drone to calculate the distance for
         * @return the distance needed to fly to the source airport and to fly from source to destination
         */
        private float getDeliveryDistance(DeliveryPackage delivery,DeliveryDrone drone) {
            Vector dronePos = drone.getPosition();
            int sourceAirportID = delivery.getSourceAirport();
            int destinationAirportID = delivery.getDestinationAirport();
            //get the map of the world
            OverseerAirportMap airportMap = DeliveryPlanning.this.getAirportMap();
            //get the source airport and the airport to deliver to
            MapAirport sourceAirport = airportMap.getAirport(sourceAirportID);
            MapAirport destinationAirport = airportMap.getAirport(destinationAirportID);
            //get the location of the source and destination airport
            Vector sourceAirportLocation = sourceAirport.getLocation();
            Vector destinationAirportLocation = destinationAirport.getLocation();
            //get the distance (from current position, to the source to the destination
            float droneToSourceDistance = dronePos.distanceBetween(sourceAirportLocation);
            float sourceToDestination = destinationAirportLocation.distanceBetween(sourceAirportLocation);
            return droneToSourceDistance + sourceToDestination;
        }


        /**
         * Checks if the node has delivered the provided package
         * @return true if the node has a drone that has the same delivery assigned
         */
        private boolean deliversPackage(DeliveryPackage deliveryPackage){
            //check if the to deliver contains the package
            Set<DeliveryPackage> toDeliver = this.getPackagesToDeliver();
            //if the packages to deliver does not contain the specified package, we've already delivered it
            return !toDeliver.contains(deliveryPackage);

        }

        /**
         * Gets the drone that delivers the requested package
         * returns null if the node does not deliver the package
         * @param delivery the delivery to check for
         * @return the drone id of the drone that delivers the package, if the package is not
         *         yet delivered by the node return null
         */
        private String getDeliveryDroneID(DeliveryPackage delivery){
            //check if we deliver the package
            if(this.deliversPackage(delivery)){
                return null;
            }
            //get the set of all drones
            Map<String, DeliveryDrone> droneMap = this.getDroneMap();
            for(DeliveryDrone drone: droneMap.values()){
                //get the deliveries of the drone
                List<DeliveryPackage> droneDeliveries = drone.getAssignedDeliveries();
                //check if the drone delivers the requested package
                if(droneDeliveries.contains(delivery)){
                    //if so return the delivery id
                    return drone.getDroneID();
                }
            }

            return null;
        }

        /**
         * Getter for the total approximated cost for the given node
         * @return the sum of the cost and the heuristic
         */
        private float getApproxCost(){
            return getCostValue() + getHeuristicValue();
        }


        /**
         * Getter for the cost value of the node
         * the value is calculated as the maximum delivery time of all the drones in the map
         * @return the highest delivery time of all the drones
         */
        private float getCostValue(){
            //check if we've calculated the cost before
            float savedCost = this.getSavedCost();
            if(savedCost > 0){
                return savedCost;
            }
            //if not:
            //get all the drones
            Collection<DeliveryDrone> deliveryDrones = this.getDroneMap().values();
            //get the max value of the nodes
            //first map to a collection of floats (the time travelled)
            Collection<Float> deliveryTimes = deliveryDrones.stream().map(DeliveryDrone::getTotalTravelTime).collect(Collectors.toList());
            //then get the maximum value
            float calcCost = Collections.max(deliveryTimes);
            this.setSavedCost(calcCost);
            //return the maximum

            return calcCost;
        }

        /**
         * Calculates the heuristic value for the node
         * the heuristic that we use works as following, take all the packages that need to be delivered
         * and split between the drones based on their distance to drones then take the distance the furthest
         * from the drone
         * note: probable update, find the convex hull, the time needed to go around it is the minimum time
         * to deliver all the packages in the hull
         * @return the heuristic value for the node
         */
        private float getHeuristicValue(){
            float heuristic = this.getSavedHeuristic();
            if(heuristic >= 0){
                return this.getSavedHeuristic();
            }
            //first get all the packages and all the drones
            Set<DeliveryPackage> packages = this.getPackagesToDeliver();
            //a list of the drones (we'll need the index)
            List<DeliveryDrone> drones = new ArrayList<>(this.getDroneMap().values());
            //the map containing the drone id's as keys and the package with the maximum delivery distance
            //for the drone with the corresponding id
            float maxDeliveryTime = 0;
            //iterate trough all the packages and find the drone the closest to the package
            for(DeliveryPackage delivery: packages){
                //use map to map the distance of the package to each drone, then find the index of the max
                //and assign the package to that drone
                List<Float> deliveryDistances = drones.stream().map(drone -> getDeliveryDistance(delivery, drone)).collect(Collectors.toList());
                //find the index of the minimum value
                float packageMin = Collections.min(deliveryDistances);

                //if the new minimum delivery time is larger than the previous max, set the max time
                if(maxDeliveryTime < packageMin){
                    maxDeliveryTime = packageMin;
                }
                //continue to the next iteration
            }
            //save the heuristic
            this.setSavedHeuristic(maxDeliveryTime);
            return maxDeliveryTime;
        }

        /**
         * Getter for the delivery that was last added to the node
         * @return the last added delivery
         * note: is used for path deletion
         */
        private DeliveryPackage getCurrentDelivery(){
            return this.currentDelivery;
        }

        /**
         * Setter for the delivery that was last added to the node
         * @param currentDelivery the delivery to set
         */
        private void setCurrentDelivery(DeliveryPackage currentDelivery){
            this.currentDelivery = currentDelivery;
        }

        /**
         * Getter for the drone that is currently used to deliver the assigned package (used for pruning)
         * @return the ID of the drone used to deliver the package
         */
        private String getCurrentDeliveryDroneID() {
            return currentDeliveryDroneID;
        }

        /**
         * Setter for the drone that is currently used to deliver the assigned package (used for pruning
         * @param currentDeliveryDroneID the ID to set
         */
        private void setCurrentDeliveryDroneID(String currentDeliveryDroneID) {
            this.currentDeliveryDroneID = currentDeliveryDroneID;
        }

        /**
         * Getter for the cost that was saved after the first call of the cost value calculation
         * purpose is to avoid redundant calculations (cost value may be called multiple times for)
         * --> returns -1f if not initialized, for use withing the private class only!
         * @return a floating point number containing the value for the cost
         */
        private float getSavedCost() {
            return cost;
        }

        /**
         * Setter for the cost that was saved after the calculation of it
         * @param cost the cost value to assign to the node
         */
        private void setSavedCost(float cost) {
            this.cost = cost;
        }

        /**
         * Saved value of the heuristic, avoids recomputation during the search (tradeoff between computation and
         * memory but is only one tiny float, so don't worry)
         * If the heuristic isn't calculated yet, returns -1f
         * DO NOT USE OUTSIDE OF THE DELIVERY NODE CLASS!
         * @return the heuristic saved for the node
         */
        private float getSavedHeuristic() {
            return heuristic;
        }

        /**
         * Setter for the heuristic value of the node (see getter for more info)
         * DO NOT USE OUTSIDE OF THE DELIVERY NODE CLASS!
         * @param heuristic the heuristic value of the node to set
         */
        private void setSavedHeuristic(float heuristic) {
            this.heuristic = heuristic;
        }

        /**
         * Getter for the map containing all the delivery drones used by the node
         * Key = ID, value = drone
         * @return a map containing the drones with their state used for the search
         */
        private Map<String, DeliveryDrone> getDroneMap() {
            return droneMap;
        }

        /**
         * Getter for the set containing the packages that still need to be delivered (and are not assigned to a drone)
         * @return a set containing delivery packages
         */
        private Set<DeliveryPackage> getPackagesToDeliver() {
            return packagesToDeliver;
        }

        /**
         * A map containing all the drones and their currently assigned packages
         * with as keys the ID's of the given drones and as value the drones themselves (used for fast adding)
         */
        private Map<String, DeliveryDrone> droneMap = new HashMap<>();

        /**
         * The set containing all packages that still need to be delivered by the drones
         */
        private Set<DeliveryPackage> packagesToDeliver = new HashSet<>();

        /**
         * The delivery that was added in this node (used for pruning)
         */
        private DeliveryPackage currentDelivery;

        /**
         * The id of the drone that was lastly used to deliver the package
         */
        private String currentDeliveryDroneID;

        /**
         * Flag to indicate if an extra package was added to the node
         */
        private boolean isCompleted = false;

        /**
         * The cost value saved for faster computation
         */
        private float cost = -1f;

        /**
         * The heuristic value of the node, saved for faster computation
         */
        private float heuristic = -1f;
    }

    /**
     * A simplified drone class used for our search algorithm
     * the instances of the drone are
     * --> position
     * --> identifier
     * --> assigned packages
     * --> the total travel time (used as cost)
     */
    private class DeliveryDrone{
        /**
         * Constructor for a simplified drone to be used for the search
         * @param position the position of the drone
         * @param droneID the id of the drone
         */
        private DeliveryDrone(Vector position, String droneID){
            this.droneID = droneID;
            this.position = position;
        }

        /**
         * Creates a deep copy of the drone
         * only the packages are not deep copied since they are not altered in any way during the search (shallow copy)
         * @return a copy of the instance, all the values are the same except that the
         *         variables are not dependent in any way (except for the packages, but we do not change them)
         */
        private DeliveryDrone deepCopy(){
            //first deepCopy the ID and the position
            String id = getDroneID();
            Vector position = getPosition().deepCopy();
            float totalTime = getTotalTravelTime();

            //then copy all the deliveries into a new list (we don't need to copy the packages, since we do not change them)
            List<DeliveryPackage> deliveries = new ArrayList<>();
            for(DeliveryPackage delivery: this.getAssignedDeliveries()){
                deliveries.add(delivery);
            }

            //create the new drone (deep copied)
            DeliveryDrone copyDrone = new DeliveryDrone(position, id);
            copyDrone.setTotalTravelTime(totalTime);
            copyDrone.setAssignedDeliveries(deliveries);

            return copyDrone;
        }

        /**
         * Getter for the list of all packages to be delivered by the drone
         * @return the list of packages to be delivered (info in the packages)
         */
        private List<DeliveryPackage> getAssignedDeliveries() {
            return assignedDeliveries;
        }

        /**
         * Setter for the deliveries to be made by the drone
         * warning: overwrites the current deliveries assigned to the drone, only use for deepcopy
         * @param deliveries the deliveries to set
         */
        private void setAssignedDeliveries(List<DeliveryPackage> deliveries){
            this.assignedDeliveries = deliveries;
        }

        /**
         * Adds the delivery to the list of deliveries to be made
         * @param delivery the delivery to be made
         */
        private void addDelivery(DeliveryPackage delivery){
            //first get the list of already assigned deliveries
            List<DeliveryPackage> deliveries = this.getAssignedDeliveries();
            //add the delivery
            deliveries.add(delivery);
        }

        /**
         * Getter for the id of the drone
         * @return a string containing the ID of the drone
         */
        private String getDroneID() {
            return droneID;
        }

        /**
         * Getter for the position of the drone in the world
         * @return the position of the drone
         */
        private Vector getPosition() {
            return position;
        }

        /**
         * Setter for the position of the drone in the world
         * @param position a vector containing the position of the drone to be set in the world
         */
        public void setPosition(Vector position) {
            this.position = position;
        }

        /**
         * Getter for the time that is travelled in total by the drone during the simulation
         * @return the total travel time in seconds
         */
        private float getTotalTravelTime() {
            return totalTravelTime;
        }

        /**
         * Increments the total travel time by the given time step
         * @param deltaTime the time step to increment the total travel time with
         */
        private void incrementTotalTravelTime(float deltaTime){
            //if the delta time is negative, ignore
            if(deltaTime < 0) {
                return;
            }
            this.totalTravelTime += deltaTime;
        }

        /**
         * Setter for the total travel time of the drone
         * warning: does overwrite the old total travel time, for adding packages use the increment
         *          this method should only be used while cloning the drones
         * @param totalTravelTime the total time to be set
         */
        private void setTotalTravelTime(float totalTravelTime){
            this.totalTravelTime = totalTravelTime;
        }

        /**
         * The list of deliveries assigned to the drone, the package to be delivered first is the package at index 0
         */
        private List<DeliveryPackage> assignedDeliveries = new ArrayList<>();

        /**
         * The id of the drone
         */
        private String droneID;

        /**
         * The current position of the drone
         */
        private Vector position;

        /**
         * The current total travel time of the drone
         */
        private float totalTravelTime = 0;
    }

}
