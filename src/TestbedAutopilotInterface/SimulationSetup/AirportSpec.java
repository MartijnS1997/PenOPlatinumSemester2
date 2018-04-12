package TestbedAutopilotInterface.SimulationSetup;

import internal.Helper.Vector;

/**
 * Created by Martijn on 31/03/2018.
 * An interface used to initialize the airports
 */
public interface AirportSpec {
    /**
     * Getter for the position of the airport in the world, by default the y-coordinate is 0
     * @return a vector containing the position of the airport in the world (x, 0, z)
     */
    Vector getPosition();

    /**
     * Getter for the orientation of the airport in the world, the primary runway (runway 0)
     * is given to indicate the orientation of the airport (see overseer class for more info on the
     * airport interface)
     * @return a vector containing the direction of the primary runway (x, 0, z)
     */
    Vector getPrimaryRunWay();

    /**
     * Getter for the width of the runway for the airport
     * @return the width of the runway
     */
    float getRunwayWidth();

    /**
     * Getter for the length of the runway for the airport
     * @return the length of the runway
     */
    float getRunwayLength();
}
