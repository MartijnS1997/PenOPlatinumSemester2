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
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        return null;
    }

}
