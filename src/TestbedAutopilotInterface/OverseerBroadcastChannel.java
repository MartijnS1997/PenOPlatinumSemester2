package TestbedAutopilotInterface;

import AutopilotInterfaces.AutopilotInputs;

import java.util.Map;
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

    /**
     * Getter for all the operational autopilots within the world (identified via their ID number)
     * @return a map containing the ID of the autopilot and their corresponding position
     */
    public ConcurrentMap<String, AutopilotInputs> getOperationalAutopilots(){
      return null;
    }

    /**
     * Getter for the queue used to transfer requests to the autopilots (the delivery requests)
     * @return a queue of deliveryRequests
     */
    public ConcurrentLinkedQueue<DeliveryRequest> getDeliveryQueue(){
        return null;
    }

    //TODO add functionality to retrieve requests specifically for the one drone



}
