package TestbedAutopilotInterface.SimulationSetup;

import internal.Helper.Vector;

/**
 * Created by Martijn on 3/04/2018.
 * An interface of drone specifications used to initialize the world with (provided to testbed server at init)
 */
public interface DroneSpec {

    /**
     * Getter for the position of the drone, this is the location at which the drone
     * will be generated during construction of the world by the drone builder
     * @return a vector containing the position of the drone
     */
    Vector getDronePosition();

    /**
     * Getter for the orientation of the drone, this is the orientation the drone is assigned during
     * generation of the world by, the drone builder
     * @return a vector containing the orientation of the drone
     */
    default Vector getDroneOrientation(){
        return new Vector();
    }

    /**
     * Getter for the velocity of the drone, this is the velocity the drone is assigned during
     * generation of the world, by the drone builder
     * @return a vector containing the velocity of the drone
     */
    default Vector getDroneVelocity(){
        return new Vector();
    }

    /**
     * Getter for the rotation of the drone, this is the rotation the drone is assigned during
     * generation of the world, by the drone builder
     * @return a vector containing the rotation of the drone
     */
    default Vector getDroneRotation(){
        return new Vector();
    }
}
