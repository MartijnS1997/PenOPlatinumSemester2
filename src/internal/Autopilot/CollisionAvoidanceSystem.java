package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.AutopilotInfo;
import internal.Helper.Vector;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 30/04/2018.
 * A class responsible for avoiding collisions with other drones during flight
 * --> should be active only when the drone is crossing the airspace of other drones, namely during descend and takeoff
 */
//TODO implement negotiation, this is the case where two drones are not in their airspace and are about to crash,
//TODO the drones can broadcast their intent (with timestamp/seq nb) about ascending or descending to avoid collision
public class CollisionAvoidanceSystem {

    public CollisionAvoidanceSystem(AutoPilot autopilot){
        this.autopilot = autopilot;
    }

    public CasCommands getCollisionActions(Map<String, AutopilotInfo> autopilotInfoMap, AutopilotInputs_v2 currentInputs){
        Set<AutopilotInfo> closeDrones = getDronesToMonitor(autopilotInfoMap, currentInputs);

        return null;
    }

    /**
     * Getter for the drones that the collision detection (and avoidance) system should monitor
     * @param autopilotInfoMap the map containing all the information needed about all the drones in the world
     * @param currentInputs the current inputs for the drone (we do not look for ourselves in the info, this info
     *                      is one step behind on the real simulation)
     * @return a set containing all the drones that are closer than scan distance from the drone
     */
    private Set<AutopilotInfo> getDronesToMonitor(Map<String, AutopilotInfo> autopilotInfoMap, AutopilotInputs_v2 currentInputs) {
        //get the id of our own drone
        AutoPilot autopilot = this.getAutopilot();
        String ownID = autopilot.getID();
        Vector position = Controller.extractPosition(currentInputs);
        float scanDistance = this.getCollisionScanDistance();

        //first filter the entries for drones that are not within collision range
        Set<AutopilotInfo> autopilotSet = new HashSet<>(autopilotInfoMap.values());
        //filter for all the drones that are nearby enough to be monitored (and are not ourselves)
        Set<AutopilotInfo> nearAutopilots = autopilotSet.stream()
                .filter(info -> info.getCurrentPosition().distanceBetween(position) < scanDistance && info.droneID() != ownID)
                .collect(Collectors.toSet());

        return nearAutopilots;
    }

    /**
     * Getter for the collision scan disatnce, this is the distance for which the drone monitors the other drones
     * if the drones are outside of this range, our drone doesn't care
     * @return a float containing the collision alert distance in meters
     */
    public float getCollisionScanDistance() {
        return collisionScanDistance;
    }

    /**
     * Setter for the collision alert distance, this is the distance in which the collision avoidance system monitors
     * the behavior of other drones
     * @param collisionScanDistance the distance for which to scan for other drones
     */
    public void setCollisionScanDistance(float collisionScanDistance) {
        this.collisionScanDistance = collisionScanDistance;
    }

    public AutoPilot getAutopilot() {
        return autopilot;
    }


    /**
     * The scanning area of the collision detection system, all the drones within this radius are monitored by
     * CAS
     */
    private float collisionScanDistance = 100f;

    /**
     * The autopilot associated with the collision avoidance system
     */
    private final AutoPilot autopilot;

    /**
     * The commands that can be issued by the Collision avoidance system, the three possible commands are
     * ASCEND: the drone should immediately ascend aggressively to avoid the collision
     * DESCEND: the drone should immediately descend aggressively to avoid collision
     * NO_COLLISION: the drone is in no immediate danger and may remain on its course
     */
    enum CasCommands{
        ASCEND, DESCEND, NO_COLLISION
    }


}
