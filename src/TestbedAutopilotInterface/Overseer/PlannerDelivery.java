package TestbedAutopilotInterface.Overseer;

/**
 * Created by Martijn on 27/04/2018.
 * An interface that separates the actions that can be done on a delivery by the overseer
 * --> the overseer can set the drone ID but cannot alter the is picked up and delivered flag
 */
public interface PlannerDelivery extends AutopilotDelivery{
    /**
     * Getter for the source airport of the package, this is the airport where the drone should pick up the package
     * @return the id of the source airport
     */
    int getSourceAirport();

    /**
     * Getter for the source gate of the package, this is the gate where the package should be picked up by the drone
     * @return the id of the source gate
     */
    int getSourceAirportGate();

    /**
     * Getter for the destination airport of the package, this is the airport where the package should be delivered to
     * @return the id of the airport
     */
    int getDestinationAirport();

    /**
     * Getter for the destination gate of the package, this is the gate where the package should be delivered to
     * @return the id of the gate
     */
    int getDestinationAirportGate();

    /**
     * Getter for the delivery flag, this indicates if the package is already delivered by the drone or not
     * --> is checked and changed by the world
     * @return the value of the is delivered flag, is true if the package is delivered to the destination gate and airport
     */
    boolean isDelivered();

    /**
     * Getter for the is picked up flag, this indicates if the package is picked up at the source airport or not
     * --> is checked and set by the world
     * @return true if and only if the drone visited the source gate at the source airport
     */
    boolean isPickedUp();

    /**
     * Getter for the delivery drone ID, this is the ID of the drone that was assigned to deliver the package
     * @return a string containing the ID of the drone that has to deliver the package
     */
    String getDeliveryDroneID();

    /**
     * Setter for the delivery drone ID
     * the planner should be able to assign ID's to packages after the planning phase is finished
     */
    void setDeliveryDroneID(String droneID);

    /**
     * Setter for the sequence number of the drone
     * the sequence number indicates in which order the packages should be given to the drones by the testbed
     * @param sequenceNumber the sequence number for the package
     */
    void setSequenceNumber(long sequenceNumber);
}
