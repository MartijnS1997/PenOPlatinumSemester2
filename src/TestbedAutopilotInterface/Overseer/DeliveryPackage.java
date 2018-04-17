package TestbedAutopilotInterface.Overseer;

/**
 * Created by Martijn on 27/03/2018.
 * A class of packages to deliver with the drones to the airport
 */
public class DeliveryPackage {

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
    protected void setDeliveryDroneID(String deliveryDroneID) {
        this.deliveryDroneID = deliveryDroneID;
    }

    /**
     * The sources and destinations of the packet
     */
    private int sourceAirport;
    private int sourceAirportGate;
    private int destinationAirport;
    private int destinationAirportGate;
    private String deliveryDroneID;

    /*
     * Packet flags
     */

    /**
     * Flag that indicates if the package is already on its destination
     */
    private boolean delivered = false;

    @Override
    public String toString() {
        return "DeliveryPackage " +
                "sourceAirport: " + sourceAirport +
                ", destinationAirport: " + destinationAirport +
                ", deliveryDroneID: " + deliveryDroneID;
    }
}
