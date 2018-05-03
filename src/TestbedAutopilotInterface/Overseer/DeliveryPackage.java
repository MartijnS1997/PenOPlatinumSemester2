package TestbedAutopilotInterface.Overseer;

/**
 * Created by Martijn on 27/03/2018.
 * A class of packages to deliver with the drones to the airport
 */
public class DeliveryPackage implements AutopilotDelivery, WorldDelivery, PlannerDelivery {

    /**
     * Constructor for a package to deliver
     * @param sourceAirport the airport where the package comes from
     * @param sourceAirportGate the gate where to retrieve the package
     * @param destinationAirport the destination airport where to deliver the package
     * @param destinationAirportGate the destination gate where the packages are dropped off
     */
    public DeliveryPackage(int sourceAirport, int sourceAirportGate, int destinationAirport, int destinationAirportGate){
        this.sourceAirport = sourceAirport;
        this.sourceAirportGate = sourceAirportGate;
        this.destinationAirport = destinationAirport;
        this.destinationAirportGate = destinationAirportGate;
    }

    /**
     * Getter for the flag that indicates if the package is delivered or not
     * @return true if the flag is set to true, false if not
     */
    public boolean isDelivered(){
        return this.delivered;
    }

    /**
     * Setter for the delivery flag, can only be toggled once (a packet that is delivered will stay delivered)
     */
    public void setDelivered(){
        this.delivered = true;
    }

    /**
     * Getter for the source airport number, the airport to retrieve the package from
     * @return an integer for identifying the source airport
     */
    public int getSourceAirport() {
        return sourceAirport;
    }

    /**
     * Getter for the source airport gate, the gate where to retrieve the package from
     * @return an integer for identifying the gate
     */
    public int getSourceAirportGate() {
        return sourceAirportGate;
    }

    /**
     * Getter for the destination airport, the airport to deliver the package to
     * @return an integer for identifying the destination airport
     */
    public int getDestinationAirport() {
        return destinationAirport;
    }

    /**
     * Setter for the destination airport gate, the gate to deliver the package to
     * @return an integer for identifying the gate of the destination airport
     */
    public int getDestinationAirportGate() {
        return destinationAirportGate;
    }

    /**
     * Getter for the ID of the drone that is assigned to deliver the package
     * @return a string containing the ID of the drone assigned with the task to deliver the package
     */
    public String getDeliveryDroneID() {
        return deliveryDroneID;
    }

    /**
     * Setter for the ID of the drone that is assigned to deliver the package
     * @param deliveryDroneID a string with the ID of the drone that has to delivered said package
     */
    public void setDeliveryDroneID(String deliveryDroneID) {
        this.deliveryDroneID = deliveryDroneID;
    }

    /**
     * Getter for the sequence number, this number is set by the planner and is an indication in which order the
     * packages should be delivered by a single drone.
     * If the sequence number of one package is lower than the other and the same drone should deliver it, the
     * package with the lower sequence number should be delivered first
     * @return the sequence number of the package
     */
    public long getSequenceNumber(){
        return this.sequenceNumber;
    }

    /**
     * Setter for the sequence number, this number indicates in which order the packages should be delivered
     * by a specific drone (see getter for more information)
     * @param sequenceNumber the sequence number of the package >= 0;
     */
    @Override
    public void setSequenceNumber(long sequenceNumber) {
        this.sequenceNumber = sequenceNumber;
    }

    /**
     * Checker for the validity of the sequence number
     * @param sequenceNumber the sequence number to assign to the package
     * @return true if and only if the sequence number >= 0
     */
    private boolean isValidSequenceNumber(long sequenceNumber){
        return sequenceNumber >= 0;
    }

    /**
     * Checks if the package is loaded onto the drone or not, if it is, it means that the drone should navigate
     * to the destination airport, if not, the drone should navigate to the source airport
     * @return true if the drone is carrying a package
     */
    public boolean isPickedUp() {
        return pickedUp;
    }

    /**
     * Setter for the is loaded flag, indicates if the package is already picked up
     * --> can only be invoked once and should only be invoked by the
     */
    public void setPickedUp() {
        this.pickedUp = true;
    }

    /**
     * The sources and destinations of the packet
     */
    private final int sourceAirport;
    private final int sourceAirportGate;
    private final int destinationAirport;
    private final int destinationAirportGate;
    private String deliveryDroneID;
    private long sequenceNumber;


    /*
     * Packet flags
     */

    /**
     * Flag that indicates if the package is already on its destination
     */
    private boolean delivered = false;
    /**
     * Flag that indicates if the package is already picked up from the source airport
     */
    private boolean pickedUp = false;

    @Override
    public String toString() {
        return "DeliveryPackage " +
                "sourceAirport: " + sourceAirport +
                ", destinationAirport: " + destinationAirport +
                ", deliveryDroneID: " + deliveryDroneID;
    }
}
