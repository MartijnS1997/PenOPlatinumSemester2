package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.AutopilotInfo;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import java.util.Comparator;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 30/04/2018.
 * A class responsible for avoiding collisions with other drones during flight
 * --> should be active only when the drone is crossing the airspace of other drones, namely during descend and takeoff
 */
//TODO first implement the collision detection system, if that works, we can do something about avoidance
public abstract class CollisionAvoidanceSystem {

    /**
     * Constructor for the collision avoidance system
     * @param communicator the communicator used for the collision avoidance system, used to gain the info about the
     *                     other drones
     */
    public CollisionAvoidanceSystem(AutopilotCommunicator communicator){
        this.communicator = communicator;
    }

    /**
     * Getter for the control commands needed to avoid collisions
     * @param currentInputs the current inputs
     * @param previousInputs the previous inputs
     * @return the commando issued by the collision avoidance system needed to stay out of harm
     */
    protected abstract CasCommand getCASCommand(AutopilotInputs_v2 currentInputs,AutopilotInputs_v2 previousInputs);


    /**
     * Getter for the set of drones that must be monitored by the collision detection system
     * @param currentPosition the current position of the drone
     * @return the set of drones that are closer to the drone than the monitor radius
     */
    protected Set<AutopilotInfo> getThreats(Vector currentPosition){
        //get the autopilot info from the drone
        AutopilotCommunicator communicator = this.getCommunicator();
        Set<AutopilotInfo> infoSet = communicator.getOtherAutopilotsInformation();
        //now filter the info set on the autopilot info's that will be subjected to further examination
        float monitorDistance = this.getThreatDistance();
        Set<AutopilotInfo> toMonitor = infoSet.stream()
                .filter(info -> (info.getCurrentPosition()).distanceBetween(currentPosition) <= monitorDistance)
                .collect(Collectors.toSet());
        return toMonitor;
    }

    protected boolean isAscending(AutopilotInputs_v2 currentInputs){
        return currentInputs.getPitch() >= 0;
    }

    /**
     * Checks if the two directions are the same
     * @param ownDirection the direction of the drone calling the method
     * @param other the other drone to check the direction for
     * @return true of the scalar product of both directions is positive
     */
    protected static boolean isFlyingInSameDirection(Vector ownDirection, AutopilotInfo other){
        Vector otherDirection = getNormalizedDirection(other);
        //take the scalar product
        float scalarProd = otherDirection.scalarProduct(ownDirection);
        return scalarProd > 0;
    }

    /**
     * Checks if the other drone has a higher altitude than the calling drone
     * @param currentInputs the inputs most recently received from the testbed by the calling drone
     * @param other the info about the other drone
     * @return true if and only id the y-value of the position of the other drone is greater than the
     *         y-value of the current inputs
     */
    protected static boolean threatHasHigherAltitude(AutopilotInputs_v2 currentInputs, AutopilotInfo other){
        float currentAltitude = Controller.extractAltitude(currentInputs);
        float otherAltitude = other.getCurrentPosition().getyValue();
        return currentAltitude < otherAltitude;
    }

    /**
     * Getter for the heading of the drone in the world axis system starting in the roll axis systeem
     * @param currentInputs
     * @return
     */
    protected static Vector getHeadingVector(AutopilotInputs_v2 currentInputs) {
        //get the heading of the caller
        Vector headingDrone = new Vector(0,0,-1);
        Vector orientation = Controller.extractOrientation(currentInputs);
        return PhysXEngine.droneOnWorld(headingDrone, orientation);
    }

    /**
     * Getter for the normalized direction vector. This vector is calculated by the difference of
     * the current position and the previous position (from prev to current), where the difference is normalized
     * afterwards (gives only the orientation of the velocity)
     * @param autopilotInfo the info to extract the direction approximation from
     * @return a vector of size 1 pointing from the previous drone position to the current drone position
     */
    protected static Vector getNormalizedDirection(AutopilotInfo autopilotInfo){
        Vector currentPosition = autopilotInfo.getCurrentPosition();
        Vector previousPosition = autopilotInfo.getPreviousPosition();
        Vector diffVector = currentPosition.vectorDifference(previousPosition);
        return diffVector.normalizeVector();
    }

    /**
     * ONLY USE IF DIRECTIONS OF THREAT AND DRONE ARE DIFFERENT
     * @param currentInputs
     * @param threatInfo
     * @return
     */
    protected static boolean isBehindThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){
        //get the vector pointing from the threat to the drone
        Vector dronePos = Controller.extractPosition(currentInputs);
        Vector threatPos = threatInfo.getCurrentPosition();

        Vector threatDirection = getNormalizedDirection(threatInfo);

        //diff vector
        Vector diff = dronePos.vectorDifference(threatPos);

        //take scalar product, if positive the drone is still in front of the threat
        //if not the drone has already passed it
        float scalar = diff.scalarProduct(threatDirection);

        return scalar <= 0;
    }

    /**
     * Getter for the comparator used to compare the different threats posed to the drone, we use the compator
     * to sort for which threat to handle first
     * @param currentPosition the position of the drone that is using the CAS
     * @return a comparator configured to return the most important threats first
     */
    protected static Comparator<AutopilotInfo> getThreatComparator(Vector currentPosition){
        return new Comparator<AutopilotInfo>() {
            @Override
            public int compare(AutopilotInfo info1, AutopilotInfo info2) {
                Vector info1Pos = info1.getCurrentPosition();
                Vector info2Pos = info2.getCurrentPosition();
                float info1DistTo = info1Pos.distanceBetween(currentPosition);
                float info2DistTo = info2Pos.distanceBetween(currentPosition);
                if (info1DistTo < info2DistTo) {
                    return -1;
                }
                if (info1DistTo > info2DistTo) {
                    return 1;
                }
                return 0;
            }
        };
    }

    /**
     * Getter for the communicator of the autopilot with the overseer
     * @return the communicator used to gain information about the other drones from the overseer
     */
    private AutopilotCommunicator getCommunicator() {
        return communicator;
    }

    /**
     * Getter for the monitor radius of the collision avoidance system
     * Drones outside of this radius are ignored
     * @return the monitor radius in meters
     */
    protected float getThreatDistance() {
        return threatDistance;
    }

    /**
     * Setter for the threat distance, this is the distance for which the drones are monitored
     * and actions are taken to avoid a collision
     * @param threatDistance the distance to set
     */
    protected void setThreatDistance(float threatDistance){
        this.threatDistance = threatDistance;
    }

    /**
     * Checks if the threat distance is valid
     * @param threatDistance the threat distance to check
     * @return true if the threat distance is strictly positve
     */
    private static boolean isValidThreatDistance(float threatDistance){
        return threatDistance > 0;
    }

    /**
     * The communicator used to get the data about the other drones from the overseer
     */
    private final AutopilotCommunicator communicator;

    /**
     * The radius wherein the collision avoidance system scans for potential threats
     */
    private float threatDistance = 100f;
}
