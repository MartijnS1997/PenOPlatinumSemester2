package internal.Autopilot;

import internal.Helper.Vector;

/**
 * Created by Martijn on 17/04/2018.
 * Specifies a turn to be made by the autopilot airport navigation controller
 */
public interface AutopilotTurn {

    /**
     * Getter for the center of the turn to be made
     * note that the center is only specified in x-z coordinates and that the y-component needs to be zero
     * such that the turn center is specified as (x,0,z)
     * @return a vector containing the center of the turn
     */
    Vector getTurnCenter();

    /**
     * Getter for the entry point of the turn, this is the point where the drone has to fly to to make the turn
     * @return the entry point of the turn (vector with coordinates in meters)
     */
    Vector getEntryPoint();

    /**
     * Getter for the radius of the turn to be made
     * @return the radius of the turn (meters)
     */
    float getTurnRadius();

    /**
     * Getter for the angle of the turn, this is the rotation angle around the center of the turn (clockwise or
     * counterclockwise) that the drone has to make before exiting the turn
     * @return positive if the drone has to rotate counterclockwise for getTurnAngle radians, negative if
     * the drone has to rotate clockwise for getTurnAngle radians
     */
    float getTurnAngle();



}
