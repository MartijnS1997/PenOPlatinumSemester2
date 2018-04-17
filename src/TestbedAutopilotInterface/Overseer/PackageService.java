package TestbedAutopilotInterface.Overseer;

import java.util.Set;

/**
 * Created by Martijn on 9/04/2018.
 * An interface to a package service, to be used by the world class to check which packages need to be given
 * to which drone and which ones are delivered
 */
public interface PackageService {

    /**
     * Gets all the packages that need to be delivered by the package service
     * @return a Set of DeliveryPackages that need to be delivered
     */
    Set<DeliveryPackage> getSubmittedPackages();

    /**
     * Getter for all the packages that have an assigned drone
     * @return all the delivery packages that have a drone assigned to them
     */
    Set<DeliveryPackage> getAssignedPackages();

    /**
     * Getter for all the packages that have not been delivered yet & should be in the future
     * @return the set of all packages that have not been delivered yet
     */
    Set<DeliveryPackage> getAllUndeliveredPackages();

    /**
     * Getter for all the packages that have been assigned to a drone to be delivered but
     * are not delivered yet
     * @return a set of packages that have not been delivered yet but have a delivery drone ID
     *         different from null
     */
    Set<DeliveryPackage> getAllUndeliveredAssignedPackages();
}
