package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.AutopilotInfo;
import internal.Helper.Vector;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Martijn on 6/05/2018.
 * Extension of the Collision avoidance system used while descending
 * (is used by the landing and the descending controller)
 * TODO make more fine grained CAS commands (make the command enum a class?)
 */
public abstract class DescendCas extends CollisionAvoidanceSystem{

    public DescendCas(AutopilotCommunicator communicator) {
        super(communicator);
    }

    /**
     * Used to configure the CAS for different needs (a landing controller may want to take more risks during
     * the descend because its landing deadline is tight, while the descending controller has a lot of time to spend
     * to configure the descending
     * @param descendRate the rate of descend employed by the descend controller
     */
    protected abstract void configureDescendCas(float descendRate);


    @Override
    protected CasCommand getCASCommand(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        Vector currentPosition = Controller.extractPosition(currentInputs);
        List<AutopilotInfo> threats = new ArrayList<>(this.getThreats(currentPosition));
        //sort the threats --> closest to farthest threats
        threats.sort(getThreatComparator(currentPosition));
        //cycle trough all the threats sorted by distance, we handle the closest one first
        //if the threat doesn't require an action (is not an active threat) handle the next one
        CasCommand command = CasCommand.NOP;
        for(AutopilotInfo threatInfo: threats){
            //handle the threat
            command = handleThreat(currentInputs, threatInfo);
            //if the drone may continue its flight, handle the next threat
            if(command != CasCommand.NOP){
                break;
            }
        }

        //return the command to handle the closest threat that doesn't require a NOP

        return command;
    }

    /**
     * Threat handler
     * A threat is handled by ascending or descending depending on the type of the threat
     * @param currentInputs the inputs most recently received from the testbed
     * @param threatInfo the info about the threat (needed to handle it)
     * @return the cas command needed to avoid collision
     *         --> if the threat has a higher altitude, the drone descends
     *         --> if the threat has a lower altitude, the drone ascends
     */
    private CasCommand handleThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){

        //first check if the drone is ascending and descending
        if(isAscending(currentInputs)){
            return handleAscendingThreat(currentInputs, threatInfo);

        }

        //if not, the drone is descending
        else{
            //check if the threat is at a lower altiude, if not, we dc
            if(threatHasHigherAltitude(currentInputs, threatInfo)){
                return CasCommand.NOP;
            }

            //check if the threat has the same direction
            Vector ownDirection = getHeadingVector(currentInputs);
            if(isFlyingInSameDirection(ownDirection, threatInfo)){
                //if so, the drone always needs to ascend
                return CasCommand.ASCEND;
            }

            //if the drone is oriented in a different direction
            //we need to check if we've already flown by
            if(isBehindThreat(currentInputs, threatInfo)){
                //if we're behind NOP(E)
                return CasCommand.NOP;
            }

            //resting case: descend
            return CasCommand.ASCEND;
        }

    }

    private CasCommand handleAscendingThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo) {
        //check if the threat is at a higher altitude than the drone, if not no threat
        if(!threatHasHigherAltitude(currentInputs, threatInfo)){
            return CasCommand.NOP;
        }

        Vector ownDirection = getHeadingVector(currentInputs);
        //if so, check if the threat is headed in the same direction
        if(isFlyingInSameDirection(ownDirection, threatInfo)){
            return CasCommand.DESCEND;
        }

        //if not, check if we're behind the threat or not
        if(isBehindThreat(currentInputs, threatInfo)){
            return CasCommand.NOP;
        }

        //resting case: in front and diff direction: descend
        return CasCommand.DESCEND;
    }


}

//    /**
//     * Dispatcher for threat handlers, handles the threat in such a way that collision is avoided
//     * @param currentInputs the inputs most recently received by the drone
//     * @param threatInfo the info about the threat
//     * @return the command needed to avoid a crash
//     */
//    private CasCommand handleThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){
//        //check first if the threat is in a fixed altitude state (other threats shouldn't be considered since they do
//        //not appear --> a taxiing drone beneath us won't affect our trajectory)
//        if(!isDescendThreat(threatInfo.getAutopilotState())){
//            return CasCommand.NOP;
//        }
//
//        //check if the threat is above the drone, if so, we may ignore it except we're pitching upwards
//        if(threatHasHigherAltitude(currentInputs, threatInfo)){
//            return handleHigherAltitudeThreat(currentInputs, threatInfo);
//        }
//
//        //the case where the threat is below us we have two cases, one where the threat is behind us
//        //and one where the threat is in front of us
//        if(!isInFrontOfDrone(currentInputs, threatInfo)){
//            return handleInBackThreat(currentInputs, threatInfo);
//        }
//
//        else{
//            return handleInFrontThreat(currentInputs, threatInfo);
//        }
//    }
//
//    /**
//     * Handler for the threat where the threatening drone is flying higher than the current drone
//     * @param currentInputs the inputs most recently received from the testbe
//     * @param threatInfo the info about the threat
//     * @return
//     */
//    private CasCommand handleHigherAltitudeThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){
//        //get the pitch of the drone
//        float pitch = currentInputs.getPitch();
//        //if the drone is heading upwards it must descend to avoid the threat, if not, NOP
//        return pitch > 0 ? CasCommand.DESCEND : CasCommand.NOP;
//    }
//
//    /**
//     * Handler for a threat that is in the back of the drone (behind the drone)
//     * @param currentInputs the inputs most recently received from the testbed
//     * @param threatInfo the info about the threat
//     * @return the Cas command needed to avoid collision with the threat
//     */
//    private CasCommand handleInBackThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){
//        //check if the threat is heading in the same direction as the drone
//        Vector headingVector = getHeadingVector(currentInputs);
//        return isFlyingInSameDirection(headingVector, threatInfo) ? CasCommand.MAINTAIN_ALTITUDE : CasCommand.NOP;
//
//    }
//
//    /**
//     * Handler for the threats where the threat is in front of the drone
//     * @param currentInputs the inputs most recently received from the testbed
//     * @param threatInfo the info about the threat
//     * @return the cas command needed to avoid a collision
//     */
//    private CasCommand handleInFrontThreat(AutopilotInputs_v2 currentInputs, AutopilotInfo threatInfo){
//        Vector headingVector = getHeadingVector(currentInputs);
//
//        //first check if the threat is going in the same direction
//        if(isFlyingInSameDirection(headingVector, threatInfo)){
//            return CasCommand.MAINTAIN_ALTITUDE;
//        }
//
//        //secondly check if the drone is close to the threat, if the drone is flying head-on we need
//        //extra precaution
//        float nearCrashDistance = this.getThreatDistance();
//        Vector threatPosition = threatInfo.getCurrentPosition();
//        Vector currentPosition = Controller.extractPosition(currentInputs);
//        return threatPosition.distanceBetween(currentPosition) <= nearCrashDistance ? CasCommand.DESCEND : CasCommand.MAINTAIN_ALTITUDE;
//
//    }
//
//    /**
//     * Checks if the provided state poses a threat to the descending controllers
//     * @param state the state to check
//     * @return true if the state is one that is airborne and at a constant altitude
//     */
//    private static boolean isDescendThreat(AutopilotState state){
//        switch(state){
//            case FLIGHT:
//                return true;
//            case DESCEND_WAIT:
//                return true;
//            default:
//                return false;
//        }
//    }
//
//    /**
//     * Getter for the near crash distance, this is the distance wherefore the drone must take extra
//     * CAS actions to avoid a crash
//     * @return the distance wherefore the extra measures must be taken in meters
//     */
//    private static float getNearCrashDistance() {
//        return nearCrashDistance;
//    }
//
//    /**
//     * Getter for the distance where the drone needs to take radical action to avoid a crash
//     * --> in the case where the drone is flying head-on with the threat
//     */
//    private final static float nearCrashDistance = 20f;
