package TestbedAutopilotInterface;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import internal.Helper.Vector;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 27/03/2018.
 * A planner for the package assignment of the drones
 * --> heuristic search with iterative deepening
 * --> in the first iteration of creating the algorithm we deepCopy ALL the drone state on every new node
 * --> first test: hill climbing 1 (no iterative deepening, first check if basics work)
 */
public class DeliveryPlanning {

    public DeliveryPlanning(Collection<DeliveryPackage> deliveryPackages,
                            OverseerAirportMap airportMap, Map<String, AutopilotInputs_v2> activeDrones){

        //first add the packages and overseer map to the instance
        this.airportMap = airportMap;
        this.deliveries = deliveryPackages;
        //then init the drones
    }

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

        //create the root of the search tree
        DeliveryNode root = new DeliveryNode(drones);
        //add the root to the queue
        List<DeliveryNode> nodeQueue = this.getNodeQueue();
        nodeQueue.add(root);

    }

    //TODO add deep deepCopy of the airport map & deliveries such that we can execute on a different thread
    //TODO add method that traverses the search tree

    //what we need:
    //a list of the drones and their current assigned deliveries and their current location
    //a list of the packages that need to be delivered
    //a list of all the airports
    //a function to calculate the distance between them
    //a function to calculate the delivery time (may be constant)
    //a variable that saves the current tree depth

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

    //instances used for the iterative deepening


    /**
     * Getter for the search depth used for iterative deepening
     * @return a float respresenting the search depth
     */
    private float getSearchDepth() {
        return searchDepth;
    }

    /**
     * Setter for the search depth used for iterative deepening
     * @param searchDepth the new search depth to use
     */
    private void setSearchDepth(float searchDepth) {
        this.searchDepth = searchDepth;
    }

    /**
     * Getter for the queue used to search the tree
     * @return a list of delivery nodes
     */
    private List<DeliveryNode> getNodeQueue() {
        return nodeQueue;
    }

    /**
     * adds the provided nodes to the queue, sorts them first based on their heuristic value in such a way
     * that the largest value will come first
     * @param deliveryNodes the nodes to be added
     */
    private void addNodes(List<DeliveryNode> deliveryNodes){
        //first sort the nodes based on heuristic value
        //Sort sorts from small to large, to get the right effect we should set the largest as "smaller" than the
        //smaller one in the comparator
        deliveryNodes.sort(new Comparator<DeliveryNode>() {
            @Override
            public int compare(DeliveryNode node1, DeliveryNode node2) {
                float heuristicNode1 = node1.getHeuristicValue();
                float heuristicNode2 = node2.getHeuristicValue();
                //check for the values
                if (heuristicNode1 < heuristicNode2) {
                    //1 means set node2 one forward in the list (in bubble sort)
                    return 1;
                }
                if (heuristicNode1 > heuristicNode2) {
                    //-1 means set node2 one backwards in the list (in bubble sort)
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
     * The depth boundary to search
     */
    private float searchDepth = 0.0f;

    /**
     * The queue used to keep track of the visited and to visit nodes
     * we use a linked list so the adding of new nodes to the queue is more efficient
     */
    private List<DeliveryNode> nodeQueue = new LinkedList<>();

    /**
     * A node of our delivery search tree
     * Has the following properties:
     * --> a heuristic value (the maximum travel time of all the drones)
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
        private DeliveryNode(Map<String, DeliveryDrone> parentDrones) {
            //create a deep copy of the map we've received
            Map<String, DeliveryDrone> childDrones = new HashMap<>();
            for(String droneID : parentDrones.keySet()){
                //get the drone & deep copy it
                DeliveryDrone drone = parentDrones.get(droneID);
                DeliveryDrone copyDrone = drone.deepCopy();
                //add the copied drone to the child map
                childDrones.put(droneID, copyDrone);
            }

            this.droneMap = childDrones;
        }

        /**
         * Adds the provided package to the drone with the corresponding ID in the map
         * @param droneID the id of the drone to add the delivery for
         * @param delivery the DeliveryPackage to be added to the drone with the given ID
         */
        private void addDelivery(String droneID, DeliveryPackage delivery){
            if(this.isCompleted){
                throw new IllegalStateException("Node already configured");
            }

            //get the drone we want the package to deliver to
            Map<String, DeliveryDrone> drones = this.getDroneMap();
            DeliveryDrone drone = drones.get(droneID);

            //calculate the new value of the drone based on distance
            Vector dronePos = drone.getPosition();
            //get id of the airport
            int airportID = delivery.getDestinationAirport();
            //get the map of the world
            OverseerAirportMap airportMap = DeliveryPlanning.this.getAirportMap();
            //get the airport
            MapAirport airport = airportMap.getAirport(airportID);
            //get the location of the airport
            Vector airportLocation = airport.getLocation();
            //get the distance
            float distanceBetween = airportLocation.distanceBetween(dronePos);

            //move the drone to the next airport
            drone.setPosition(airportLocation);
            //increment the time
            drone.incrementTotalTravelTime(distanceBetween);
            //add the new package to deliver
            drone.addDelivery(delivery);

            //we're finished
            isCompleted = true;
        }


        /**
         * Getter for the heuristic value of the node
         * the value is calculated as the maximum delivery time of all the drones in the map
         * @return the highest delivery time of all the drones
         */
        private float getHeuristicValue(){
            //check if we've calculated the heuristic before
            float savedHeuristic = this.getSavedHeuristic();
            if(savedHeuristic > 0){
                return savedHeuristic;
            }
            //if not:
            //get all the drones
            Collection<DeliveryDrone> deliveryDrones = this.getDroneMap().values();
            //get the max value of the nodes
            //first map to a collection of floats (the time travelled)
            Collection<Float> deliveryTimes = deliveryDrones.stream().map(DeliveryDrone::getTotalTravelTime).collect(Collectors.toList());
            //then get the maximum value
            float calcHeuristic = Collections.max(deliveryTimes);
            this.setSavedHeuristic(calcHeuristic);
            //return the maximum

            return calcHeuristic;
        }

        /**
         * Getter for the heuristic that was saved after the first call of the heuristic value calculation
         * purpose is to avoid redundant calculations (heuristic value may be called multiple times for)
         * --> returns -1f if not initialized, for use withing the private class only!
         * @return a floating point number containing the value for the heuristic
         */
        public float getSavedHeuristic() {
            return heuristic;
        }

        /**
         * Setter for the heuristic that was saved after the calculation of it
         * @param heuristic the heuristic value to assign to the node
         */
        public void setSavedHeuristic(float heuristic) {
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
         * A map containing all the drones and their currently assigned packages
         * with as keys the ID's of the given drones and as value the drones themselves (used for fast adding)
         */
        private Map<String, DeliveryDrone> droneMap = new HashMap<>();

        /**
         * Flag to indicate if an extra package was added to the node
         */
        private boolean isCompleted = false;

        private float heuristic = -1f;
    }

    /**
     * A simplified drone class used for our search algorithm
     * the instances of the drone are
     * --> position
     * --> identifier
     * --> assigned packages
     * --> the total travel time (used as heuristic)
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
         * The list of deliveries assigned to the drone
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
