package TestbedAutopilotInterface.GUI;

/**
 * Created by Martijn on 29/04/2018.
 * A class for passing the data about a package to the GUI for a single frame in the GUI queue
 * note: we may not implement this interface in the PackageDelivery class because this would go against the isolated
 * semantics of the GUI state interfaces
 *
 * note: the source airport + source gate fully identifies the pick up location for the package
 * note: the destination airport + destination gate fully identifies the drop off location for the package
 */
public interface DeliveryGuiState {

    /**
     * Getter for the id of the source airport, this is the airport where the package should be picked up
     * @return the id of the source airport
     */
    int getSourceAirport();

    /**
     * Getter for the source gate, this is the gate at the source airport where the package should be picked up
     * @return the source gate
     */
    int getSourceGate();

    /**
     * Getter for the id of the destination airport, this is the airport where the package should be dropped off
     * @return the id of the destination airport
     */
    int getDestinationAirport();

    /**
     * Getter for the destination gate, this is the gate at the destination airport where the package should be dropped off
     * @return the destination gate of the airport
     */
    int getDestinationGate();

    /**
     * Getter for the id of the drone that is responsible for picking up the package and delivering it
     * @return A string containing the ID of the drone
     */
    String getDeliveryDrone();

    /**
     * Getter for the flag if the package has been picked up by the delivery drone at the source airport
     */
    boolean isPickedUp();

    /**
     * Getter for the flag if the package is delivered to the destination airport (by the assigned drone)
     */
    boolean isDelivered();

}
