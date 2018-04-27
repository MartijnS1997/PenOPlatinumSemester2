package TestbedAutopilotInterface.Overseer;

/**
 * Created by Martijn on 27/04/2018.
 * Interface to separate the actions that may be done on a package with the autopilot
 * --> an autopilot may query all the parameters but may not alter them, this is only possible for the
 *     world & overseer to do
 */
public interface AutopilotDelivery {

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
}
