package TestbedAutopilotInterface;

import internal.Helper.Vector;

/**
 * Created by Martijn on 26/03/2018.
 * An interface of drone states used for GUI communication
 * Other aspects needed for rendering the drones may be added later on
 */
public interface DroneGuiState {

    /**
     * Getter for the position of the drone at the time of recording the state
     * @return the state of the drone
     */
    Vector getPosition();

    /**
     * Getter for the velocity of the drone at the time of recording the state
     * @return the state of the drone
     */
    Vector getVelocity();

    /**
     * Getter for the orientation of the drone at the time of recording the state
     * @return the orientation of the drone
     */
    Vector getOrientation();

}
