package TestbedAutopilotInterface.Overseer;

import internal.Autopilot.AutoPilot;
import internal.Autopilot.AutopilotState;
import internal.Helper.Vector;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 6/05/2018.
 * a class used to communicate the autopilot information from the overseer to the communicator
 */
public class AutopilotInfoCenter {

    public AutopilotInfoCenter() {

    }

    public synchronized void updateEntry(AutoPilot autopilot, AutopilotInfo info){
        Map<String, AutopilotInfo> infoMap = this.getAutopilotInfoMap();
        String droneID = autopilot.getID();
        infoMap.put(droneID, info);
    }

    /**
     * Removes the entry belonging to the provided autopilot
     * @param autopilot the autopilot to remove the entry for
     */
    public synchronized void removeEntry(AutoPilot autopilot){
        String autopilotID = autopilot.getID();
        this.getAutopilotInfoMap().remove(autopilotID);
    }

    /**
     * Checks if the provided autopilot already has an entry in the info center
     * @param autopilot the autopilot to check for
     * @return true if the autopilot info center already has an entry for the given autopilot
     */
    protected boolean hasEntry(AutoPilot autopilot){
        //get the entry for the autopilot in the map
        String droneID = autopilot.getID();
        //if the map returns null, this means there is no entry present yet
        AutopilotInfo info = this.getEntry(autopilot);
        return info != null;
    }

    /**
     * Getter for the info about one particular entry in the info center
     * @param autopilot the autopilot to get the entry for
     * @return the autopilot info associated with the autopilot provided
     */
    protected synchronized AutopilotInfo getEntry(AutoPilot autopilot){
        String droneID = autopilot.getID();
        return this.getAutopilotInfoMap().get(droneID);
    }


    /**
     * Gets the number of entries (or the size) in the info center, this is equal to the number of autopilots
     * that have already filed their first report to the info center
     * @return the number of autopilots that already have an entry in the info center
     */
    protected int getSize(){
        Set<String> keySet = this.getAutopilotInfoMap().keySet();
        return keySet.size();
    }


    /**
     * Creates a map that contains all the autopilots and their positions
     * @return a map with as keys the id's of the drones and values the current position of the autopilot
     */
    protected synchronized Map<String, Vector> getAutopilotPositions(){
        Map<String, AutopilotInfo> infoMap = this.getAutopilotInfoMap();
        Set<String> keySet = infoMap.keySet();
        Map<String, Vector> dronePositions = new HashMap<>();
        for(String droneID : keySet){
            AutopilotInfo entry = infoMap.get(droneID);
            Vector position = entry.getCurrentPosition();
            dronePositions.put(droneID, position);
        }

        return dronePositions;
    }

    /**
     * Gets the positions of all the autopilots except for the autopilot that was specified in the call
     * @param autopilot the autopilot that is calling the method
     * @return a map with as keys the id's of the drones and as values the positions
     */
    public synchronized Map<String, Vector> getOtherAutopilotPositions(AutoPilot autopilot){
        String droneID = autopilot.getID();
        Map<String, Vector> allPositions = this.getAutopilotPositions();
        allPositions.remove(droneID);
        return allPositions;
    }

    /**
     * Getter for the set of autopilot info's containing all the information about the other autopilots
     * besides the caller of the method
     * @param autopilot the autopilot that calls the method
     * @return a set with all autopilot info's with a drone id different from that of the provided autopilot
     */
    public synchronized Set<AutopilotInfo> getInfoOtherAutopilots(AutoPilot autopilot){
        Set<AutopilotInfo> infoSet = new HashSet<>(this.getAutopilotInfoMap().values());
        String droneID = autopilot.getID();
        //filter for the entries that are not the provided autopilot
        return infoSet.stream().filter(info -> !info.droneID().equals(droneID)).collect(Collectors.toSet());
    }

    /**
     * Returns the number of autopilots that have a state different from the initialized state
     */
    protected boolean allAutopilotsInitialized(){
        //get all the entries & filter them based on the state that they have
        Set<AutopilotInfo> infoSet = new HashSet<>(this.getAutopilotInfoMap().values());
        Set<AutopilotInfo> initSet = infoSet.stream().filter(info -> info.getAutopilotState() == AutopilotState.INIT_FLIGHT).collect(Collectors.toSet());
        return initSet.size() == 0;
    }

    /**
     * Getter for the map that contains all the information about the autopilots needed to do collision detection
     * @return the map containing all the data about the autopilots, keys are the ID's of the autopilots and
     *         values the corresponding autopilot info objects
     */
    private Map<String, AutopilotInfo> getAutopilotInfoMap() {
        return autopilotInfoMap;
    }

    /**
     * The map containing all the information about the autopilots
     */
    private Map<String, AutopilotInfo> autopilotInfoMap= new ConcurrentHashMap<>();
}
