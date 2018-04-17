package TestbedAutopilotInterface.GUI;

import internal.Helper.Vector;

/**
 * Created by Martijn on 26/03/2018.
 * An interface of gui cube states
 * used to pass the position and color of the cube to the GUI
 * other functionalities may be added layer
 */
public interface CubeGuiState {

    /**
     * Getter for the position of the cubes
     * @return a vector containing the position of the cube
     */
    Vector getPosition();


//    /**
//     * Getter for the color of the cubes
//     * @return a vector containing the HSV color values of the cube (H, AIRPORT_FIT_EXCEPTION, V)
//     */
//    Vector getHSVColor();
}

