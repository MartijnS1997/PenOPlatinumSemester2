package TestbedAutopilotInterface.GUI;

import internal.Helper.Vector;

/**
 * Created by Martijn on 27/03/2018.
 * An interface for testbed-gui communication, is responsible for passing the needed parameters
 * of the airport to render it
 * note: extra functionality may be added later
 */
public interface AirportGuiState {

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
}
