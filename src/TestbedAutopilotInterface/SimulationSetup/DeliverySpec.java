package TestbedAutopilotInterface.SimulationSetup;

/**
 * Created by Martijn on 7/04/2018.
 * A class of delivery specifications, used by the simulation generator
 */
public interface DeliverySpec {

    /**
     * Getter for the airport that is the source of the package
     * @return the airport id of the source airport
     */
    int getSourceAirport();

    /**
     * Getter for the gate to get the package at
     * @return integer containing the id of the gate
     */
    int getSourceAirportGate();

    /**
     * Getter for the airport to deliver the package to
     * @return the airport id of the destination airport
     */
    int getDestinationAirport();

    /**
     * Getter for the destination gate
     * @return the id of the gate where the package is destined
     */
    int getDestinationAirportGate();

}
