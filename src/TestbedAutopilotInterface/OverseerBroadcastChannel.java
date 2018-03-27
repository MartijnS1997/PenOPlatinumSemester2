package TestbedAutopilotInterface;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import internal.Autopilot.AutoPilot;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ConcurrentMap;

/**
 * Created by Martijn on 27/03/2018.
 * An interface for package delivery request to be sent to the autopilot via a queue
 */
public class OverseerBroadcastChannel {

    //TODO see which information is needed for the autopilot, do we burden it with maintaining
    //TODO map of all the airports or is it the responsibility of the overseer to just give the coordinates
    //TODO and maintain the map only for itself? (i would go for option two)



}
