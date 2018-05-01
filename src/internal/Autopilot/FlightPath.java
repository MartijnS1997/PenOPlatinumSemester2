package internal.Autopilot;

import internal.Helper.Vector;

/**
 * Created by Martijn on 30/04/2018.
 * A class to save the flight path of the drone for collision detection and avoidance
 */
public class FlightPath {

    public FlightPath(AutopilotTurn firstTurn, AutopilotTurn secondTurn) {
        this.firstTurn = firstTurn;
        this.secondTurn = secondTurn;
        this.connectionVector = calcConnectionVector(firstTurn, secondTurn);
    }

    /**
     * Calculates the vector that connects the exit point of the first turn with the entry point of the second turn
     * @param firstTurn the first turn, where the exit point is extracted from
     * @param secondTurn the second turn, where the entry point is extracted from
     * @return a vector that points from the exit point of the first turn to the entry point of the second turn
     */
    private Vector calcConnectionVector(AutopilotTurn firstTurn, AutopilotTurn secondTurn){
        Vector exitTurn1Rel = firstTurn.getExitPoint();
        Vector entryTurn2Rel = secondTurn.getEntryPoint();

        Vector centerTurn1 = firstTurn.getTurnCenter();
        Vector centerTurn2 = secondTurn.getTurnCenter();

        Vector exitTurn1 = centerTurn1.vectorSum(exitTurn1Rel);
        Vector entryTurn2 = centerTurn2.vectorSum(entryTurn2Rel);

        return entryTurn2.vectorDifference(exitTurn1);
    }

    /**
     * Getter for the first turn in the flight path, this is the turn originating from the drone
     * @return the first turn in the flight path
     */
    public AutopilotTurn getFirstTurn() {
        return firstTurn;
    }

    /**
     * Getter for the second turn in the flight path, this is the turn that will end parallel to the runway
     * of the airport
     * @return the second turn of the flight path
     */
    public AutopilotTurn getSecondTurn() {
        return secondTurn;
    }

    /**
     * Getter for the connection vector, this is the vector that connects the exit and the entry point
     * of the first and second turn respectively
     * @return the vector that points from the first turn to the entry of the second turn
     */
    public Vector getConnectionVector() {
        return connectionVector;
    }

    private final AutopilotTurn firstTurn;

    private final AutopilotTurn secondTurn;

    private final Vector connectionVector;
}
