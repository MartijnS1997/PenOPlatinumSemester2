package TestbedAutopilotInterface.GUI;

import java.util.Map;
import java.util.Set;

/**
 * Created by Martijn on 26/03/2018.
 * An interface of elements that will be added to the queue for simulating the world
 */
public interface GUIQueueElement {

    /**
     * Getter for the states of the drones that are simulated
     * in the testbed, the map contains the drone ID as key and the state
     * of the drone as the key
     * @return a map with as key the done ID and the value the state of the drone
     */
    Map<String, DroneGuiState> getDroneStates();

    /**
     * Getter for the positions and color of the cubes that are present in the world
     * @return a set with the states of the cubes
     */
    Set<CubeGuiState> getCubePositions();

    /**
     * Getter for he positions of the airports present within the world
     * the first vector contains the position of the airport and the second one the orientation
     * (the first and the final element contain data, y is by definition zero)
     * @return a set of airport gui states with all the info needed to render the airports
     */
    Set<AirportGuiState> getAirport();


}
