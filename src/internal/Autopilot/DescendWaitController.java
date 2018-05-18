package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 4/05/2018.
 * Controller used to wait in the drone's airspace if an airport is locked by another drone
 * The drone keeps turning until the airport is released by the other drone
 * TODO now we're testing a single wait turn, configure controller that we can wait for multiple turns
 * TODO remove request for airport when taking off
 */
public class DescendWaitController extends TurnBasedController {

    public DescendWaitController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public void reset() {
        //we must remove all the state from the controller
        this.setTurn(null);
        this.setCruisingAltitude(0);
        this.setAcquiredLock(false);
        this.setTurnControl(null);
        this.setAirportToLock(0);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        TurnControl turnControl = this.getTurnControl();
        AutopilotTurn turn = turnControl.getTurn();
        //check if the turn control has finished making its turn, if so reset the turn controls
        if(hasFinishedTurn(turn, currentInputs, previousInputs)/*turnControl.hasFinishedTurn(*/){
            //normally this shouldn't cause any harm because the has reached objective is called before the
            //getControl actions, thus we may reset
            turnControl.resetAngleToGo();
        }
        //all the controls are done by this call, we don't need to do anything else to the outputs
        AutopilotOutputs outputs = turnControl.getControlActions(currentInputs, previousInputs);
        return outputs;
    }

    /**
     * Checks if the drone has reached the objective
     * @param currentInputs the inputs most recently received from the testbed (this is the base of the check)
     * @param previousInputs the inputs previously received from the testbed
     * @return true if the drone has acquired a lock and a. has not yet started the turn, b. has finished the waiting turn
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //try to lock every time we call the has reached objective
        boolean airportReserved = tryLock();

        //there are two cases when the drone has reached its objective, for both is needed: a lock on the desired airport
        if(airportReserved){
            TurnControl turnController = this.getTurnControl();

            //first case, the turn has not yet started (and thus the angle to go is equal to the turning angle)
            if(!turnController.hasStartedTurn()){
                return true;
            }
            else{
                //second case, the turn has finished
                //we may use the has finished turn because the has reached objective is invoked before
                //the call to the controller so the state of has finished turn is maintained for exactly one iteration
                AutopilotTurn turn = turnController.getTurn();
                return hasFinishedTurn(turn, currentInputs, previousInputs);
            }
        }

        return false;
    }

    /**
     * Configures the descend wait controller
     * --> sets the turn to be made based on the current position
     * --> assigns the cruising altitude of the drone
     * this method should be called every time the controller is invoked after a state transition in the
     * autopilot finite state machine
     * @param currentInputs the inputs most recently received from the testbed
     * @param cruisingAltitude the cruising altitude of the drone
     */
    public void configureWaitController(AutopilotInputs_v2 currentInputs, float cruisingAltitude, int airportToLock){
        this.setCruisingAltitude(cruisingAltitude);
        this.setAirportToLock(airportToLock);
        //generate the turn
        Vector droneGroundPos = extractGroundPosition(currentInputs);
        Vector droneOrientation = extractOrientation(currentInputs);
        Vector droneNegXHa = new Vector(-1,0,0); //the negative X axis of the drone in the heading axis system
        float turnRadius = getTurnRadius();
        //first get the center of the turn
        //get the -x axis of the heading axis system in the world axis system
        Vector droneX = PhysXEngine.headingOnWorld(droneNegXHa, droneOrientation);
        //scale the x-axis to the turn radius to get the center of the turn relative to the position of the drone
        Vector turnCenterRel = droneX.normalizeToLength(turnRadius);
        //add the ground position of the drone
        Vector turnCenter = droneGroundPos.vectorSum(turnCenterRel);
        //get the entry position relative to the turn center
        Vector turnEntry = droneGroundPos.vectorDifference(turnCenter);
        //get the exit point, this is the same point as the entry point
        Vector turnExit = turnEntry;
        //and we will turn anticlockwise for a full 360 degrees
        float turnAngle = getTurnAngle();

        this.setTurn(createTurn(turnCenter, turnEntry, turnExit, turnAngle, turnRadius));
        PhysXEngine.TurnPhysX turnPhysX = this.getAutopilot().getPhysXEngine().createTurnPhysics();
        this.setTurnPhysX(turnPhysX);
        configureTurnController();
    }

    /**
     * Method for configuring the turn controller itself, this is the controller used to make the turn
     * this method is called every time a new waiting turn is made (it is possible that we have to make several
     * turns before we can actually land)
     */
    private void configureTurnController(){

        AutopilotTurn turn = this.getTurn();
        PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();
        StandardOutputs standardOutputs = this.getStandardOutputs();
        float cruisingAltitude = this.getCruisingAltitude();
        TurnControl turnControl = new TurnControl(turn, turnPhysX, standardOutputs, cruisingAltitude);
        this.setTurnControl(turnControl);
    }

    /**
     * tries to acquire a lock on the airport specified in getAirportToLock
     * if the lock is already acquired, the method has no effect
     * if the lock is not yet acquired, the method tries to establish a lock
     * @return true if the drone has a lock on the airport
     */
    private boolean tryLock(){
        //check if the drone has already a lock on the airport, so, this method will have no effect
        if(this.hasAcquiredLock()){
            return true;
        }
        //if not, call the communicator and try to get the lock
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        int landingAirport = this.getAirportToLock();
        boolean couldLock = communicator.requestLanding(landingAirport);
        //save if the drone could acquire the lock for the next iteration
        this.setAcquiredLock(couldLock);
        return couldLock;
    }

    /**
     * Getter for the turn controller, this is the controller used to make the turn while waiting for a lock
     * @return the turn controller used for a single turn
     */
    private TurnControl getTurnControl() {
        return turnControl;
    }

    /**
     * Setter for the turn controller (see getter for more info)
     * @param turnControl the turn controller used for a single turn
     *                    note that the controller must be rebuilt (or reset) every time we need to do another turn
     *                    we may keep the old specified turn but we must reset the controller to avoid mishaps
     */
    private void setTurnControl(TurnControl turnControl) {
        this.turnControl = turnControl;
    }

    /**
     * Getter for the turn radius, the radius of the turn that the drone makes while it is waiting to descend
     * @return the turn radius in meters
     */
    private static float getTurnRadius() {
        return turnRadius;
    }

    /**
     * Getter for the turn angle, this is the nb of radians the drone has to turn before it has finished the turn
     * @return the turn angle in radians
     */
    private static float getTurnAngle() {
        return turnAngle;
    }

    /**
     * Getter for the turn specified for the descend waiting controller, the drone keeps executing this turn until
     * it has gotten a lock on the airport
     * @return the turn generated for circling around the airport
     */
    private AutopilotTurn getTurn(){
        return this.turn;
    }

    /**
     * Setter for the turn that the drone makes while waiting for a lock on the airport
     * @param turn the turn to make
     */
    private void setTurn(AutopilotTurn turn) {
        this.turn = turn;
    }

    /**
     * Getter for the airport that must be locked by the drone
     * @return the id of the airport that must be locked by the drone
     */
    private int getAirportToLock() {
        return airportToLock;
    }

    /**
     * Setter for the airport to lock, this is the airport where the drone wants to land
     * and only may do so if it has gained permission to land
     * @param airportToLock the airport the controller needs to acquire a lock for
     */
    private void setAirportToLock(int airportToLock) {
        this.airportToLock = airportToLock;
    }

    /**
     * Returns the value for the lock flag, if the value is true, the drone has acquired a lock on the airport or landing
     */
    private boolean hasAcquiredLock() {
        return acquiredLock;
    }

    /**
     * Sets the lock flag status to the provided one
     * @param acquiredLock the lock status to set
     */
    private void setAcquiredLock(boolean acquiredLock) {
        this.acquiredLock = acquiredLock;
    }

    /**
     * Getter for the turn physics, these are the physics used to calculate the flight parameters for making the turn
     * (must be put in the turn controller)
     * @return the turn physics
     */
    private PhysXEngine.TurnPhysX getTurnPhysX() {
        return turnPhysX;
    }

    /**
     * Setter for the turn physics (see getter for more info)
     * @param turnPhysX the turn physics to be set
     */
    private void setTurnPhysX(PhysXEngine.TurnPhysX turnPhysX) {
        this.turnPhysX = turnPhysX;
    }

    /**
     * Getter for the cruising altitude, this is the altitude the drone has to maintain while turning
     * @return the cruising altitude of the drone in meters
     */
    private float getCruisingAltitude() {
        return cruisingAltitude;
    }

    /**
     * Setter for the cruising altitude of the drone
     * @param cruisingAltitude the altitude to maintain during the turn in meters > 0
     */
    private void setCruisingAltitude(float cruisingAltitude) {
        this.cruisingAltitude = cruisingAltitude;
    }

    /**
     * Getter for the standard outputs, needed to generate the turn controller, these are the default values
     * for the inclinations and brakes
     * @return the standard outputs for the controller
     */
    private StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

    /**
     * Getter for the turn controller responsible for making the turn while waiting for acquiring a lock
     */
    private TurnControl turnControl;

    /**
     * The turn to execute while waiting for the descend
     */
    private AutopilotTurn turn;

    /**
     * The turn physics used for calculating the parameters needed to make the turn
     */
    private PhysXEngine.TurnPhysX turnPhysX;

    /**
     * The airport id of the airport that the drone has to acquire a lock for
     */
    private int airportToLock;

    /**
     * Flag to indicate if the lock on the airport was acquired or not (we don't need to lock once we have acquired a lock
     */
    private boolean acquiredLock = false;

    /**
     * The cruising altitude for the drone, this is the altitude the drone has to maintain during the turn
     */
    private float cruisingAltitude;

    /**
     * The radius of the turn to execute
     */
    private final static float turnRadius = 1000f;

    /**
     * The turn angle used for making the turn while waiting
     */
    private final static float turnAngle = (float) (2*PI);


    /**
     * The standard outputs used to construct the control outputs used by the controller
     * use of the control outputs class prevents that we need to specify a full fledged control output on every single
     * iteration (use of anonymous classes) and lets us focus on only the needed control actions that are of intrest
     * for this specific controller
     */
    private StandardOutputs standardOutputs = new StandardOutputs() {
        @Override
        public float getStandardRightMainInclination() {
            return MAIN_STABLE;
        }

        @Override
        public float getStandardLeftMainInclination() {
            return MAIN_STABLE;
        }

        @Override
        public float getStandardHorizontalStabilizerInclination() {
            return HORIZONTAL_STABLE;
        }

        @Override
        public float getStandardVerticalStabilizerInclination() {
            return VERTICAL_STABLE;
        }

        @Override
        public float getStandardThrust() {
            return 0;
        }
    };

    /**
     * Parameters for the standard outputs
     */
    private final static float MAIN_STABLE = (float) (5*PI/180);
    private final static float HORIZONTAL_STABLE = 0f;
    private final static float VERTICAL_STABLE = 0f;

}
