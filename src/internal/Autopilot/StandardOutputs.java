package internal.Autopilot;

/**
 * Created by Martijn on 16/04/2018.
 * Getter for the standard outputs of a controller, used by the control outputs class
 */
public interface StandardOutputs {

    /**
     * Getter for the standard right main wing inclination, used as a default value for the control outputs of a drone
     * @return a float between -PI/2 and PI/2 indicating the standard inclination of the right main wing
     */
    float getStandardRightMainInclination();

    /**
     * Getter for the standard left main wing inclination, used as a default value for the control outputs of a drone
     * @return a float between -PI/2 and PI/2 indicating the standard inclination of the left main wing
     */
    float getStandardLeftMainInclination();

    /**
     * Getter for the standard horizontal stabilizer inclination, used as a default value for the control outputs of a drone
     * @return a float between -PI/2 and PI/2 indicating the standard inclination of the horizontal stabilizer
     */
    float getStandardHorizontalStabilizerInclination();

    /**
     * Getter for the standard vertical stabilizer inclination, used as a default value for the control outputs of a drone
     * @return a float between -PI/2 and PI/2 indicating the standard inclination of the vertical stabilizer
     */
    float getStandardVerticalStabilizerInclination();

    /**
     * Getter for the standard thrust, used as a default for the control outputs of a drone
     * @return a thrust value between 0 and maxThrust (drone specific) to pass to the drone
     */
    float getStandardThrust();

    /**
     * Getter for the standard front brake force used for the outputs of the drone
     * @return a floating point number between 0 and maxR (drone specific)
     */
    default float getStandardFrontBrakeForce(){
        return 0;
    }

    /**
     * Getter for the standard left brake force used for the outputs of the drone
     * @return a floating point number between 0 and maxR (drone specific)
     */
    default float getStandardLeftBrakeForce(){
        return 0;
    }
    /**
     * Getter for the standard right brake force used for the outputs of the drone
     * @return a floating point number between 0 and maxR (drone specific)
     */
    default float getStandardRightBrakeForce(){
        return 0;
    }

}
