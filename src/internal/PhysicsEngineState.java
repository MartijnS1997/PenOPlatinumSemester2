package internal;

/**
 * Created by Martijn on 7/11/2017.
 */
public interface PhysicsEngineState {

    /**
     * Get the position output of the physics engine
     * @return the next position calculated by the engine
     */
    Vector getPosition();

    /**
     * Get the velocity output of the physics engine
     * @return the next velocity calculated by the engine
     */
    Vector getVelocity();

    /**
     * Get the orientation output of the physics engine
     * @return the next orientation calculated by the engine
     */
    Vector getOrientation();

    /**
     * Get the rotation output of the physics engine
     * @return the next rotation calculated by the engine
     */
    Vector getRotation();

}
