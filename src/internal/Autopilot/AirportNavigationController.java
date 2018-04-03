package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;

/**
 * Created by Martijn on 3/04/2018.
 * A class of controllers used to navigate between the airports present in the world
 * TODO implement this class for the final stage of the project
 */
public class AirportNavigationController extends AutopilotFlightController {


    public AirportNavigationController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        return null;
    }

    @Override
    protected float getMainStableInclination() {
        return 0;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return 0;
    }

    @Override
    protected float getRollThreshold() {
        return 0;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return 0;
    }
}
