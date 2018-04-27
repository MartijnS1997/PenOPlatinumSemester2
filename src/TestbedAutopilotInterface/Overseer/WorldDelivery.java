package TestbedAutopilotInterface.Overseer;

/**
 * Created by Martijn on 27/04/2018.
 * An interface to restrict the actions that the world can do on a package
 */
public interface WorldDelivery {
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
     * Setter for the delivery flag, can be only set once and will be toggled to true
     */
    void setDelivered();

    /**
     * Getter for the is picked up flag, this indicates if the package is picked up at the source airport or not
     * --> is checked and set by the world
     * @return true if and only if the drone visited the source gate at the source airport
     */
    boolean isPickedUp();

    /**
     * Setter for the picked up flag, can only be invoked once and will be toggled to true
     */
    void setPickedUp();

    /**
     * Getter for the ID of the drone that is assigned to deliver the package, is used by the world
     * to see if it may pass the package to the drone that is currently located at the gate
     * @return the ID of the drone that is assigned with the delivery
     */
    String getDeliveryDroneID();
}
