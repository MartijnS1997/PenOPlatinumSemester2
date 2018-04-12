package TestbedAutopilotInterface.SimulationSetup;

import java.util.List;
import java.util.Set;

/**
 * Created by Martijn on 7/04/2018.
 * An interface for describing the simulation environment containing all the information needed to setup
 * the entire simulation
 */
public interface SimulationEnvironment {

    /**
     * Getter for the specifications of all the drones to be added to a simulation environment
     * @return a list of droneSpec objects containing the specs of the drones
     */
    List<DroneSpec> getDroneSpecifications();

    /**
     * Getter for the specifications of all the airports to be added to the simulation environment
     * @return a list of airport specifications
     */
    List<AirportSpec> getAirportSpecifications();

    /**
     * Getter for the deliveries to be added to the simulation environment (these packages need to be delivered by
     * the drones)
     * @return a Set of delivery specifications to be delivered
     */
    Set<DeliverySpec> getDeliveryPackages();

}
