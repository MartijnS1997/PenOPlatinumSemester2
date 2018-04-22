package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

/**
 * Created by Martijn on 3/04/2018.
 * A class of controllers used to navigate between the airports present in the world
 * TODO implement the methods to handle multiple turns in a row and to fly to the next one
 * TODO split the turn control and in between control into two separate private classes
 * matlab commands to plot the flight path:
 * M = dlmread('trajectoryLog.txt',';')
 * plot3(M(:,1), M(:,2), M(:,3))
 * with the current folder the project folder of this project
 *
 * note: all vectors are in the world axis system unless specified differently
 */
public class AirportNavigationController extends AutopilotFlightController {


    public AirportNavigationController(AutoPilot autopilot) {
        super(autopilot);
    }

    /**
     * Generates the control actions of the navigator for the given inputs and the currently active state
     * (see the navigator state interface)
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return the control outputs needed to steer the drone to the next airport
     */
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the current state of the controller
        NavigatorState currentState = this.getNavigatorState();
        //get the next state to generate outputs for
        NavigatorState nextState = this.getNextState(currentInputs, previousInputs);
        //check if the same state as previous one
        if(nextState != currentState){
            System.out.println();
            //if not configure the controller
            configureStateController(nextState, currentInputs, previousInputs);
            //and save the state
            this.setNavigatorState(nextState);
            //note: we only have to save the next state if the state is indeed changed, if not we would be doing
            //duplicate work
            System.out.println("switched state from: " + currentState +  ", to: " + nextState);
        }

        //TODO remove once done debugging
        trajectoryLog(currentInputs);
        //errorLog(currentInputs.getRoll());

//        trajectoryLog(currentInputs);
        //get the control actions needed for the next state (we always use the next state in case it is changed)
        return getStateControls(nextState, currentInputs, previousInputs);

    }

    /**
     * Initializes the airport navigation controller
     * --> creates the turn physX object needed to do the physics calculations
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void initializeNavigation(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        PhysXEngine.TurnPhysX turnPhysX = this.getAutopilot().getPhysXEngine().createTurnPhysics();
        //save the turn physX
        this.setTurnPhysX(turnPhysX);
    }

    /**
     * Getter for the next turn on the trajectory of the drone to navigate to the airport
     * TODO now it is just configured for testing but once finished, hook it up to the path generator
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return an AutopilotTurn object containing all the specifications to do the next turn
     */
    private AutopilotTurn getNextTurn(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //TODO implement
        //TODO make call to the path generator & delete the dummy

        if(firstCall) {
            generateCurcuit(currentInputs);
        }


        AutopilotTurn currentTurn = turns.get(floorMod(turnCount++,2));
        System.out.println("current turn center: " + currentTurn.getTurnCenter() + "entry point: " + currentTurn.getEntryPoint());
        return currentTurn;

    }

    /**
     * debugging method, remove if ready hook == first turning angle is zero, causing the drone to fly straight
     * after takeoff
     */
    private void generateHook(AutopilotInputs_v2 currentInputs){
        //to getNextTurn if the second controller can fly level
        float turnRadius = 1000;
        float turnDistance = 4000; //distance between both turn centers
        float turnAngle = -(float) PI; //both turns go 180 degrees clockwise
        Vector xHeading = new Vector(1, 0, 0);
        Vector zHeading = new Vector(0,0,-1);
        Vector currentGroundPos = extractGroundPosition(currentInputs);
        Vector currentOrientation = extractOrientation(currentInputs);
        //calculate the turn centers
        //for the first turn, place it to the right of the drone at 1000m distance
        Vector relCenter1GroundHa = xHeading.scalarMult(turnRadius); // location of first turn in heading axis on ground
        // relative in heading axis in the air
        Vector relCenter1 = PhysXEngine.headingOnWorld(relCenter1GroundHa, currentOrientation);
        Vector center1 = relCenter1.vectorSum(currentGroundPos);
        Vector entry1 = currentGroundPos.vectorDifference(center1); //the entry point

        // the translation of the center of the first turn along the negative z-axis in the heading axis system
        Vector center1ZTransHa = zHeading.scalarMult(turnDistance);
        Vector center1ZTrans = PhysXEngine.headingOnWorld(center1ZTransHa, currentOrientation);
        Vector center2 = center1.vectorSum(center1ZTrans);

        //now translate the exit point along center1ZTrans to get the entry point of the first turn
        Vector entry2abs = currentGroundPos.vectorSum(center1ZTrans);
        Vector entry2rel = entry2abs.vectorDifference(center2);

        //generate the turns
        AutopilotTurn turn1 = new AutopilotTurn() {
            @Override
            public Vector getTurnCenter() {
                return center1;
            }

            @Override
            public Vector getEntryPoint() {
                return entry1;
            }

            @Override
            public float getTurnRadius() {
                return turnRadius;
            }

            @Override
            public float getTurnAngle() {
                return 0;
            }
        };

        AutopilotTurn turn2 = new AutopilotTurn() {
            @Override
            public Vector getTurnCenter() {
                return center2;
            }

            @Override
            public Vector getEntryPoint() {
                return entry2rel;
            }

            @Override
            public float getTurnRadius() {
                return turnRadius;
            }

            @Override
            public float getTurnAngle() {
                return turnAngle;
            }
        };

        turns.add(turn1);
        turns.add(turn2);

        firstCall = false;
        System.out.println("Turn data, turn 1 center: " + turn1.getTurnCenter() + ", entry: " + turn1.getEntryPoint());
        System.out.println("Turn data, turn 2 center: " + turn2.getTurnCenter() + ", entry: " + turn2.getEntryPoint());
    }

    /**
     * for debugging, remove once ready --> the shape of the generated path is like a running track
     */
    private void generateCurcuit(AutopilotInputs_v2 currentInputs){
        //to getNextTurn if the second controller can fly level
        float turnRadius = 1000;
        float turnDistance = 4000; //distance between both turn centers
        float turnAngle = -(float) PI; //both turns go 180 degrees clockwise
        Vector xHeading = new Vector(1, 0, 0);
        Vector zHeading = new Vector(0,0,1);
        Vector altitude = new Vector(0, this.getCruisingAltitude(),0);
        Vector currentGroundPos = extractGroundPosition(currentInputs);
        Vector currentOrientation = extractOrientation(currentInputs);
        //calculate the turn centers
        //for the first turn, place it to the right of the drone at 1000m distance
        Vector relCenter1GroundHa = xHeading.scalarMult(turnRadius); // location of first turn in heading axis on ground
        // relative in heading axis in the air
        Vector relCenter1 = PhysXEngine.headingOnWorld(relCenter1GroundHa, currentOrientation);
        Vector center1 = relCenter1.vectorSum(currentGroundPos);
        Vector entry1 = currentGroundPos.vectorDifference(center1); //the entry point

        //for the second turn we also need to go backwards (along pos z-axis of heading axis) to place the center
        //we can use the first center and sum the relative displacement

        // the translation of the center of the first turn along the z-axis in the heading axis system
        Vector center1ZTransHa = zHeading.scalarMult(turnDistance);
        Vector center1ZTrans = PhysXEngine.headingOnWorld(center1ZTransHa, currentOrientation);
        Vector center2 = center1.vectorSum(center1ZTrans);

        //calculate the entry point of the second turn
        //first get the exit point of the first turn
        Vector relExit1GroundHa = xHeading.scalarMult(turnRadius*2); //the exit at ground level in heading axis
        Vector relExit1 = PhysXEngine.headingOnWorld(relExit1GroundHa, currentOrientation); //exit relative to drone in world
        Vector absExit1 = currentGroundPos.vectorSum(relExit1); //absolute exit point

        //now translate the exit point along center1ZTrans to get the entry point of the first turn
        Vector entry2abs = absExit1.vectorSum(center1ZTrans);
        Vector entry2rel = entry2abs.vectorDifference(center2);

        //generate the turns
        AutopilotTurn turn1 = new AutopilotTurn() {
            @Override
            public Vector getTurnCenter() {
                return center1;
            }

            @Override
            public Vector getEntryPoint() {
                return entry1;
            }

            @Override
            public float getTurnRadius() {
                return turnRadius;
            }

            @Override
            public float getTurnAngle() {
                return turnAngle;
            }
        };

        AutopilotTurn turn2 = new AutopilotTurn() {
            @Override
            public Vector getTurnCenter() {
                return center2;
            }

            @Override
            public Vector getEntryPoint() {
                return entry2rel;
            }

            @Override
            public float getTurnRadius() {
                return turnRadius;
            }

            @Override
            public float getTurnAngle() {
                return turnAngle;
            }
        };

        turns.add(turn1);
        turns.add(turn2);

        firstCall = false;
        System.out.println("Turn data, turn 1 center: " + turn1.getTurnCenter() + ", entry: " + turn1.getEntryPoint());
        System.out.println("Turn data, turn 2 center: " + turn2.getTurnCenter() + ", entry: " + turn2.getEntryPoint());
    }

    //TODO remove after path generator is ready
    private int turnCount = 0;
    private boolean firstCall = true;
    private List<AutopilotTurn> turns = new ArrayList<>();



    /**
     * Getter for the state that will be active during the next iteration of control generation
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return the navigator state that will be active on the next iteration
     */
    private NavigatorState getNextState(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the current state
        NavigatorState currentState = this.getNavigatorState();
        //open a switch to check which controls are next
        switch(currentState){
            case INIT:
                //when we're initializing we'll only need to return that we'll be turning next
                //but first initialize the controller
                this.initializeNavigation(currentInputs, previousInputs);
                return NavigatorState.TURNING;
            case TURNING:
                //check if we've finished the turn, if so we have to navigate to the next turn, if not keep turning
                TurnControl turnController = this.getTurnControl();
                return turnController.hasFinishedTurn(currentInputs, previousInputs) ? NavigatorState.TO_NEXT_TURN : NavigatorState.TURNING;
            case TO_NEXT_TURN:
                //check if we've reached the entry point of the next point, if so start next turn, if not keep flying
                ToNextTurnControl toNextTurnControl = this.getToNextTurnControl();
                return toNextTurnControl.hasReachedEntryPoint(currentInputs) ? NavigatorState.TURNING : NavigatorState.TO_NEXT_TURN;
            default:
                //gets called if the next state is null...this is the case when the controller is initializing
                return NavigatorState.TURNING;
        }
    }

    /**
     * Generates the control actions for the drone based on the currently active state based on the
     * current inputs and the previous inputs
     * @param navigatorState the state of the airport navigator, used to select the controller and its actions
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return a control outputs object containing the control actions for the drone for the current state
     */
    private AutopilotOutputs getStateControls(NavigatorState navigatorState, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //open a switch to pass the inputs to the right controller
        switch(navigatorState){
            case TURNING:
                TurnControl turnControl = this.getTurnControl();
                return turnControl.getControlActions(currentInputs, previousInputs);
            case TO_NEXT_TURN:
                ToNextTurnControl toNextTurnControl = this.getToNextTurnControl();
                return toNextTurnControl.getControlActions(currentInputs, previousInputs);
            default:
                throw new IllegalStateException("An illegal state was entered, please check the possible states to select from");
        }
    }

    /**
     * Configures the controller responsible for the provided state
     * this method should be called every time the finite state machine switches state
     * @param navigatorState the state to configure the corresponding controller for
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void configureStateController(NavigatorState navigatorState, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //open a switch to call the configurators
        switch(navigatorState){
            case TURNING:
                //call the configuration method
                configureTurningController(currentInputs, previousInputs);
                break;
            case TO_NEXT_TURN:
                //call the configuration method
                configureToNextTurnController(currentInputs, previousInputs);
                break;
        }
    }

    /**
     * Configures the turn controller for the next turn that has to be made
     * method should be called every time the controller is switched to a turn controller
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void configureTurningController(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the turn physics
        PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();
        //generate the next turn
        NavigatorState state = this.getNavigatorState();
        AutopilotTurn nextTurn;
        //the controller needs a turn to do, in the case that the navigator was initializing
        //we need to generate a new turn, otherwise the next turn can be extracted from the to next turn controller
        //that was responsible for flying to the turn
        switch(state){
            case INIT:
                nextTurn = this.getNextTurn(currentInputs, previousInputs);
                break;
            default:
                ToNextTurnControl toNextTurnControl = this.getToNextTurnControl();
                nextTurn = toNextTurnControl.getNextTurn();
        }

        //then generate the next turn controller
        TurnControl nextTurnControl = new TurnControl(nextTurn, turnPhysX);
        //after generation save the newly generated controller
        this.setTurnControl(nextTurnControl);
    }

    /**
     * Configures the to next turn controller
     * this method should be called every time the state is switched to the to next turn controller
     * --> generates a new controller configured to fly to the next turn
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * note: this method relies on the previously active controller being the turning controller
     *       if we decide to start with the to next turn controller this method should be changed & make use
     *       of a dummy turn to begin with
     */
    private void configureToNextTurnController(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the turn physics
        PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();
        //get the previous turn from the previous turning controller
        TurnControl previousTurnControl = this.getTurnControl();
        AutopilotTurn previousTurn = previousTurnControl.getTurn();

        //generate the next turn
        AutopilotTurn nextTurn = this.getNextTurn(currentInputs, previousInputs);

        //generate and set the controller
        ToNextTurnControl toNextTurnControl = new ToNextTurnControl(nextTurn, previousTurn, turnPhysX);
        this.setToNextTurnControl(toNextTurnControl);

    }

    /*
    Controller instances
     */


    /**
     * Getter for the state the airport navigation controller is currently in
     * @return the current state of the navigation controller
     */
    private NavigatorState getNavigatorState() {
        return navigatorState;
    }

    /**
     * Setter for the state of the airport navigation controller
     * @param navigatorState the state to set
     */
    private void setNavigatorState(NavigatorState navigatorState) {
        this.navigatorState = navigatorState;
    }

    /**
     * Getter for the cruising altitude of the drone, the altitude the drone has to maintain during its flight
     * @return the cruising altitude assigned to the drone by the overseer (in meters)
     */
    private float getCruisingAltitude() {
        return cruisingAltitude;
    }

    /**
     * Setter for the cruising altitude of the drone, this is the reference altitude for the controllers and must be
     * maintained during the flight
     * @param cruisingAltitude the (positive) cruising altitude in meters
     */
    public void setCruisingAltitude(float cruisingAltitude) {
        if(!isValidCruisingAltitude(cruisingAltitude)){
            throw new IllegalArgumentException("the provided cruising altitude was not strictly positive");
        }
        this.cruisingAltitude = cruisingAltitude;
    }

    /**
     * Checks if the cruising altitude is valid
     * @param cruisingAltitude the cruising altitude to check
     * @return true if and only if the cruising altitude > 0
     */
    private boolean isValidCruisingAltitude(float cruisingAltitude){
        return cruisingAltitude > 0;
    }


    /**
     * Getter for the control turn object(containing all the info needed to make the currently specified turn)
     * @return the control turn specified for the current turn
     */
    private TurnControl getTurnControl() {
        return turnControl;
    }

    /**
     * Setter for the current control turn, must be invoked once the drone is finished making its previous turn
     * @param turnControl the next turn to make with the navigation controller
     */
    private void setTurnControl(TurnControl turnControl) {
        this.turnControl = turnControl;
    }

    /**
     * Getter for the to next turn control needed to navigate to the next turn
     * --> controller is used during the to next turn state to guide the drone to the next turn
     * @return the between turn control currently used
     */
    private ToNextTurnControl getToNextTurnControl() {
        return toNextTurnControl;
    }

    /**
     * Setter for the next turn control, the controller used to navigate to the next turn (see getter for more info)
     * @param toNextTurnControl the next turn control controller to use by the finite state machine
     */
    private void setToNextTurnControl(ToNextTurnControl toNextTurnControl) {
        this.toNextTurnControl = toNextTurnControl;
    }

    /**
     * Getter for the main wing stable inclination, used to steer the drone
     * @return a float between 0 and PI/2 indicating the stable/reference inclination for the main wing
     */
    private static float getMainStable() {
        return MAIN_STABLE;
    }

    /**
     * Getter for the main wing inclination deviation, this is the maximum deviation from the stable inclination
     * allowed to be assigned to the main wings. A main wing thus always has an inclination in the range
     * [MAIN_STABLE - MAIN_DELTA_INCL, MAIN_STABLE + MAIN_DELTA_INCL]
     * @return the main wing inclination deviation
     */
    private static float getMainDeltaIncl() {
        return MAIN_DELTA_INCL;
    }

    /**
     * Getter for the stable horizontal stabilizer inclination, the reference inclination to add the
     * inclination to
     * @return the stable horizontal stabilizer inclination in radians
     */
    private static float getHorizontalStable() {
        return HORIZONTAL_STABLE;
    }

    /**
     * Getter for the maximum horizontal delta inclination (same semantics as the main delta inclination)
     * the available range for the horizontal stabilizer is: [HORIZONTAL_STABLE -HOR_MAX, HORIZONTAL_STABLE + HOR_MAX]
     * @return the maximum deviation in the horizontal inclination from the stable value
     */
    private static float getHorizontalDeltaIncl() {
        return HORIZONTAL_DELTA_INCL;
    }

    /**
     * Getter for the stable inclination of the vertical stabilizer, this is the reference inclination that is used
     * as a default if the controls do not specify the vertical stabilizer inclination
     * @return the stable vertical stabilizer inclination
     */
    private static float getVerticalStable() {
        return VERTICAL_STABLE;
    }

    /**
     * Getter for the angle of attack margin of error during the calculations for adjusting the
     * inclinations to fix the aoa
     * @return the error margin used during the aoa calculations (in radians)
     */
    private float getAoaErrorMargin() {
        return aoaErrorMargin;
    }

    /**
     * Setter for the error margins used for the AOA calculations
     * @param aoaErrorMargin the margin in radians
     */
    public void setAoaErrorMargin(float aoaErrorMargin) {
        this.aoaErrorMargin = aoaErrorMargin;
    }

    /**
     * Getter for the standard outputs for the controller, used to initialize the control outputs
     * @return an object containing the standard outputs for the autopilot
     */
    private StandardOutputs getStandardOutputs(){
        return this.standardOutputs;
    }

    /**
     * Getter for the turn physics used for calculating the banking angle for a turn and the reference velocity
     * @return the turn physics used by the controller for calculating the parameters for making the turn
     */
    private PhysXEngine.TurnPhysX getTurnPhysX() {
        return turnPhysX;
    }

    /**
     * Setter for the turn physics of the drone
     * @param turnPhysX the turn physics to configure the controller with
     */
    private void setTurnPhysX(PhysXEngine.TurnPhysX turnPhysX){
        if(!canHaveAsTurnPhysX(turnPhysX)){
            throw new IllegalArgumentException("The turn physics currently provided is a null reference or the " +
                    "controller already has a turn physics configured");
        }
        this.turnPhysX = turnPhysX;
    }

    /**
     * Checks if the controller can have the provided turn physics as its physics engine for the turns
     * @param turnPhysX the turn physics object to check
     * @return true if and only if the controller already has a turn physics and the provided turn physics is not
     *         a null reference
     */
    private boolean canHaveAsTurnPhysX(PhysXEngine.TurnPhysX turnPhysX){
        return turnPhysX!=null && this.turnPhysX == null;
    }

    /**
     * The state of the airport navigator, used to configure the finite state machine
     * describing the behavior of the airport navigation
     * --> 1. start turning
     * --> 2. fly to next turn
     * --> goto 1. until we're landing
     */
    private NavigatorState navigatorState = NavigatorState.INIT;

    /**
     * The cruising altitude assigned to the drone by the overseer, this is the reference altitude for the
     * altitude PID controllers ( there are different controllers for different scenario's)
     */
    private float cruisingAltitude;

    /**
     * The physics engine turn physics used for calculating the reference velocity and the banking roll
     */
    private PhysXEngine.TurnPhysX turnPhysX;


    /**
     * The control turn object used to manage all the turn related state and checks is used to
     * --> calculate and save the reference roll needed to make the turn
     * --> calculate and save the reference velocity needed to keep the drone level
     * --> check if the turn has finished
     * --> save the specifications of the turn we're currently making
     * --> calculates the control actions needed to make the turn
     */
    private TurnControl turnControl;

    /**
     * The control object used to manage all the in between turn state and checks used
     * also generates the control actions needed to fly between the turns
     */
    private ToNextTurnControl toNextTurnControl;



    /*
    Configuration of the steering angles of the controller
     */
    //the main wings
    /**
     * Main wing configuration, we cap the main wing inclinations to prevent excessive control actions
     * --> Main stable: the stable inclination of the main wings, used as a reference
     * --> Main delta incl: de maximum inclination deviation from the stable value, the range of possible main wing
     *                      inclinations= range[MAIN_STABLE - MAIN_DELTA_INCL, MAIN_STABLE + MAIN_DELTA_INCL]
     */
    private final static float MAIN_STABLE = (float) (5*PI/180);
    private final static float MAIN_DELTA_INCL = (float) (2*PI/180);

    /**
     * Configuration for the horizontal stabilizer, we cap the horizontal stabilizer inclinations to prevent
     * excessive control actions
     * --> horizontal stable: the stable inclination of the horizontal stabilizer, used for reference
     * --> horizontal delta: the maximum stabilizer inclination deviation, the range of possible inclinations
     *                       [HORIZONTAL_STABLE -HOR_MAX, HORIZONTAL_STABLE + HOR_MAX]
     */
    private final static float HORIZONTAL_STABLE = 0f;
    private final static float HORIZONTAL_DELTA_INCL = (float) (8*PI/180);

    /**
     * Configuration of the vertical stabilizer, these wings should stay fixed during the flight
     */
    private final static float VERTICAL_STABLE = 0f;
    private final static float VERTICAL_MAX = 0f;

    /**
     * The error margin used for the AOA calculations, eg if the inclination is set back to 15° in theory
     * the real inclination will be 13° to account for errors in the velocity approx
     */
    private float aoaErrorMargin = (float) (2*PI/180);

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
     * The states for the airport navigator finite state machine
     * --> sates may be added in the future if needed
     */
    //TODO ADD WRAP UP CONTROLS FOR WHEN THE TOTAL CONTROLS ARE FINISHED FOR THIS NAVIGATION
    private enum NavigatorState{
        INIT, TURNING, TO_NEXT_TURN
    }


    /**
     * A class for configuring the controller for a turn and generating the control actions needed
     * to make the specified turn given the current and previous inputs of the autopilot
     * note: on every new turn a new turn controller has to be initialized
     */
    //TODO improve the performance of the altitude controller by using a pitch control system based on targets
    //TODO instead of altitude
    private class TurnControl {

        public TurnControl(AutopilotTurn turn, PhysXEngine.TurnPhysX turnPhysX) {
            this.turn = turn;
            this.turnPhysX = turnPhysX;
            this.angleToGo = turn.getTurnAngle();
            configureTurn(turn);
        }

        /**
         * Configures the controller for making the current turn
         * --> sets the roll needed to make the specified turn
         * --> sets the velocity needed to make the specified turn
         * @param turn the turn to configure the controller for
         */
        private void configureTurn(AutopilotTurn turn){
            //calculate the reference roll and velocity for making the turn
            float roll = calculateBankingRoll(turn);
            float velocity = calculateTurnVelocity(roll);

            //save the results
            this.setBankingTurnRoll(roll);
            this.setTurnVelocity(velocity);
        }

        /**
         * Getter for the control actions for the turn controller
         * the control outputs contain the commands for the drone to make the specified turn
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         * @return a control outputs object containing the outputs for the drone to make the turn
         */
        private AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //create the control outputs object to store the control actions for the drone
            ControlOutputs outputs = new ControlOutputs(AirportNavigationController.this.getStandardOutputs());

            //get all the control actions needed to make the turn
            rollControl(outputs, currentInputs, previousInputs);
            pitchControl(outputs, currentInputs, previousInputs);
            angleOfAttackControl(getAoaErrorMargin(), outputs, currentInputs, previousInputs);
            thrustControl(outputs, currentInputs, previousInputs);

            outputs.capInclinations(getMainDeltaIncl(), getMainDeltaIncl(), getHorizontalDeltaIncl(), 0);
            return outputs;
        }


        /**
         * Calculates the thrust outputs for maintaining the reference velocity
         * needs to be called after the inclinations are known
         * @param outputs the outputs to write the control actions to and to read the wing inclinations from to determine
         *                the thrust needed to maintain the reference velocity
         * @param currentInputs the latest inputs from the testbed
         * @param previousInputs the previous inputs from the testbed
         */
        private void thrustControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the parameters needed to call the cruise control
            float referenceVelocity = this.getTurnVelocity();
            PIDController thrustPID = this.getThrustController();
            AirportNavigationController.this.flightCruiseControl(outputs, currentInputs, previousInputs, thrustPID, referenceVelocity);
        }

        /**
         * Generates the control actions to keep the roll in check during a banking turn with turning radius
         * ||dronePos - turnCenter|| and with center turnCenter
         * @param outputs the outputs to write the calculated results to
         * @param currentInputs the latest inputs received by the autopilot from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            //calculate the angle needed to make the turn (may be done only once in the future)
            //get the parameters first:

            //calculate the roll needed to make the turn
            float bankingRoll = this.getBankingTurnRoll();
            //correct the banking roll for the error on the turn radius
            float correctedBanking = correctReferenceRoll(bankingRoll, currentInputs);
            //get the PID and the parameters needed to get the errorOutput
            float roll = extractRoll(currentInputs);
            float rollError = correctedBanking - roll; // the error on the roll of the drone
            float deltaTime = getDeltaTime(currentInputs, previousInputs);

//            System.out.println("banking roll needed: " + bankingRoll + ", actual roll: " + roll + ", error: " + rollError);

            PIDController rollController = this.getRollController();
            float pidRollError = rollController.getPIDOutput(rollError, deltaTime);

            //adjust the wings accordingly
            //if the roll error was negative we need to steer to the right, if the roll error was positive
            //we need to steer to the left (draw it yourself if you don't believe me) because the PID inverts
            //the error value we need to steer to the right if we have a positive pidRollError and to the left
            //if we have a positive pidRollError
            float mainStableInclination = getMainStable();
            float mainDeltaIncl = getMainDeltaIncl();
            float requiredRightMain = mainStableInclination - pidRollError;
            float requiredLeftMain = mainStableInclination + pidRollError;

            float cappedRightMain = capInclination(requiredRightMain, mainStableInclination, mainDeltaIncl);
            float cappedLeftMain = capInclination(requiredLeftMain, mainStableInclination, mainDeltaIncl);

            outputs.setRightWingInclination(cappedRightMain);
            outputs.setLeftWingInclination(cappedLeftMain);
        }

        /**
         * Method to correct the reference roll of the drone to account for deviation in the current radius of
         * the circle the drone is flying in and the desired radius
         * @param referenceRoll the reference roll to correct, this is the roll that is used as a reference for the
         *                      roll controller
         * @param currentInputs the inputs most recently received from the testbed
         * @return the corrected roll
         */
        private float correctReferenceRoll(float referenceRoll, AutopilotInputs_v2 currentInputs){
            //get the current turn
            AutopilotTurn currentTurn = this.getTurn();
            float turnRadius = currentTurn.getTurnRadius();

            //get the current distance from the center (measured in ground coordinates)
            Vector droneGroundPos = extractGroundPosition(currentInputs);
            Vector turnCenter = currentTurn.getTurnCenter();
            Vector radiusVector = droneGroundPos.vectorDifference(turnCenter);
            float actualRadius =  radiusVector.getSize();
            float error = actualRadius-turnRadius;

            //get the error ratio, if the the drone is currently to far from the center we have to roll harder
            //if we're to close we may loosen the roll a bit
            return referenceRoll*((turnRadius + error *0.1f)/turnRadius);
        }

        /**
         * Getter for the control actions needed to keep the drone at the cruising altitude during the turn
         * @param outputs the outputs of the drone to write the result to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void pitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the reference value
            Vector pitchReferencePoint = this.getPitchReferencePoint(currentInputs);
            float pitchError = this.getPitchError(currentInputs, pitchReferencePoint);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            //feed the results into the pid controller
            PIDController pitchPid = this.getPitchController();
            float pidResult = pitchPid.getPIDOutput(pitchError, deltaTime);
            //if the pid result is positive: the set point is larger than the input -> lower the pitch (pos inclination)
            //if the pid result is negative: the set point is smaller than the input -> increase the pitch (neg inclination)
            float horizontalInclination = getHorizontalStable() + pidResult;
            horizontalInclination = capInclination(horizontalInclination, getHorizontalStable(), getHorizontalDeltaIncl());
            outputs.setHorStabInclination(horizontalInclination);
        }

        /**
         * Calculates the reference point used for the pitch controller, this controller is used to maintain the cruising altitude
         * the reference point is generated in front of the drone with coordinates (0, cruisingAltitude, - lookaheadDistance)
         * (coordinates are specified in the heading axis system, the axis system where roll and pitch are zero)
         * @param currentInputs the latest inputs received by the autopilot from the testbed
         * @return the reference point for the pitch in the world axis system
         * note: this method must be called every time the pitch controls are calculated because the reference point
         *       changes on every iteration
         */
        private Vector getPitchReferencePoint(AutopilotInputs_v2 currentInputs){
            //grab the parameters needed to calculate the reference point
            Vector dronePositionGround = extractGroundPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);
            Vector headingDroneHa = new Vector(0,0,-1); // the heading vector of the drone is heading axis
            float cruisingAltitude = AirportNavigationController.this.getCruisingAltitude();
            Vector altitudeVector = new Vector(0,cruisingAltitude, 0);
            float lookaheadDistance = this.getLookaheadDistance();

            //first calculate the position of the reference point in the heading axis system
            Vector referencePitchGroundHa = headingDroneHa.scalarMult(lookaheadDistance); //reference in heading axis at ground level
            //also add the cruising altitude so that we factor the reference altitude in
            Vector referencePitchHa = referencePitchGroundHa.vectorSum(altitudeVector);
            //transform the reference point to the world axis system, this vector is now relative to the position
            //of the drone in the world, so we have to add the ground coordinates (since the altitude is our target)
            Vector referenceWorldRel = PhysXEngine.headingOnWorld(referencePitchHa, droneOrientation);

            //sum the ground coordinates to make the reference point absolute
            return dronePositionGround.vectorSum(referenceWorldRel);

        }

        /**
         * Calculates the error on the pitch of the drone against the reference point of the flight
         * the pitch error is the angle between the heading vector (zero roll) in the pitch axis system
         * and the vector pointing from the drone to the reference point ( =difference vector) projected onto
         * the yz-plane of the pitch axis system
         * @param currentInputs the inputs most recently received from the testbed
         * @param referencePoint the reference point to calculate the pitch error for
         * @return the angle as described above with the sign indicating the direction of the error
         *         if the pitch error is positive, extra pitch is required (steering upwards)
         *         if the pitch error is negative, lesser pitch is required (steering downwards)
         */
        private float getPitchError(AutopilotInputs_v2 currentInputs, Vector referencePoint){
            //grab the parameters needed fo the calculation
            Vector droneHeadingPa = new Vector(0,0,-1); //the heading vector of the drone in pitch axis
            Vector yzNormalPa = new Vector(1,0,0); // normal of the yz-plane in pitch axis
            Vector dronePosition = extractPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);

            //calculate the difference vector and transform it to the pitch axis system
            //difference vector points from the drone position to the reference point
            Vector diffVector =  referencePoint.vectorDifference(dronePosition);
            Vector diffPa = PhysXEngine.worldOnPitch(diffVector, droneOrientation);

            //project the diff vector in pitch axis system onto the yz plane of the pitch axis
            Vector projDiffPa = diffPa.orthogonalProjection(yzNormalPa);

            //calculate the angle between the heading vector and the projected diff vector
            float pitchErrorAngle = projDiffPa.getAngleBetween(droneHeadingPa);

            //also calculate the direction of the error, take the vector product of the heading and the diff vector
            //if the x-component is positive the drone has to pitch upwards, if it is negative the drone has
            //to pitch downwards to reach the desired altitude/pitch
            float direction = signum(droneHeadingPa.crossProduct(projDiffPa).getxValue());

            //multiply to get the error
            float pitchError = direction*abs(pitchErrorAngle);

            //return the result while checking for NaN, if so return harmless (?) value
            return Float.isNaN(pitchError) ? 0 : pitchError;
        }

        /**
         * Checks if the drone has finished making the current specified turn
         * @param currentInputs the inputs most recently received by the autopilot from the testbed
         * @param previousInputs the previous inputs received from the testbed
         * @return true if and only if the signs of getAngleToGo() and turn.getTurnAngle() differ
         * note: the working is based on the rotation that the drone covers during one iteration by subtracting
         * the delta angle from the angle that the drone still has to do. For counterclockwise rotation both
         * angle to go and the delta angle are positive such that the angle to go will slowly decrease to zero
         * and will finally pass it, causing a negative angle to go and a positive turn angle (because counterclockwise)
         *
         * for a clockwise rotation both the turn angle and the delta angle are negative such that the angle to
         * go will increase to zero and ultimately pass it, causing (again) a difference in sign of the angles
         *
         * extra: the sign of the delta angle is determined by the vector product
         * of the vector pointing from the center to the previous position and the vector pointing from the center
         * to the current position (all coordinates in the xz plane)
         *
         * for more info see the notes --> contact Martijn
         */
        private boolean hasFinishedTurn(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){

            //get the specifications for the turn we're currently making
            AutopilotTurn turn = this.getTurn();
            float turnAngle = turn.getTurnAngle();

            //get the current and previous location of the drone in the xz-plane
            Vector currentPos = extractGroundPosition(currentInputs);
            Vector previousPos = extractGroundPosition(previousInputs);

            //get the location of the center of the current turn
            Vector turnCenter = turn.getTurnCenter();

            //calculate the (unit) difference vectors between the inputs and the turn center
            Vector previousTurnVector = previousPos.vectorDifference(turnCenter).normalizeVector();
            Vector currentTurnVector = currentPos.vectorDifference(turnCenter).normalizeVector();

            //get the angle difference between them both (this is the part of the turn that we've made
            //during the last iteration (we still have to determine the sign)
            float turnDiffAngle = previousTurnVector.getAngleBetween(currentTurnVector);

            //determine the sign, take the vector product of the previous and the current turn vector
            //take the y-component of the product and take the sign
            float sign = signum(previousTurnVector.crossProduct(currentTurnVector).getyValue());

            //update the angleToGo
            float deltaAngle = sign*turnDiffAngle;
            updateAngleToGo(deltaAngle);
            //check if we've made the whole turn this can be checked easily by comparing the signs of the
            //angle to go and the turn angle, if they differ the angle is finished
            float angleToGo = this.getAngleToGo();
            return signum(angleToGo) != signum(turnAngle);
        }

        /**
         * Calculates the roll needed to make the given turn
         * @param currentTurn the turn to calculate the banking roll for
         * @return the roll needed to make the banking turn as specified above
         * note: negative roll means turn to right and positive roll means turn to left
         */
        private float calculateBankingRoll(AutopilotTurn currentTurn){
            //first calculate the turning radius & calculate the angle needed to bank
            float bankingAngle = calculateBankingAngle(currentTurn);

            //the banking angle will always be positive so we also need to get the direction needed
            // if the turn is counterclockwise (positive turn angle) we need to turn to the left --> pos roll
            // if the turn is clockwise (negative turn angle) we need to turn to the right --> neg roll
            // it is nice to know that the counter clockwise rotation is a positive angle and the clockwise rotation
            // is a negative angle

            //so if clockwise rotation --> negative angle and negative roll
            //and for counter clockwise rotation --> positive angle and positive roll
            return signum(currentTurn.getTurnAngle()) * abs(bankingAngle);
        }

        /**
         * Calculates the banking angle to make the given turn
         * @param currentTurn  the turn to calculate the banking angle for
         * @return the banking angle the drone needs to maintain for turning with the given radius (in radians)
         */
        private float calculateBankingAngle(AutopilotTurn currentTurn){
            //get the turn physics responsible
            PhysXEngine.TurnPhysX turnPhysX = AirportNavigationController.this.getTurnPhysX();

            return turnPhysX.getBankingAngle(currentTurn, getMainStable());
            //return the roll
        }

        /**
         * Calculates the reference velocity needed to keep the drone at the same altitude during the turn
         * this is the reference velocity passed to the thrust controller
         * @param roll the roll wherefore the altitude must remain the same
         * @return the velocity needed to fly level in m/s
         */
        private float calculateTurnVelocity(float roll){
            //get the turn physics
            PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();
            //get the other parameters needed
            float mainStable = getMainStable();
            //calculate
            return turnPhysX.calcTurnVelocity(roll, mainStable);

        }

        /**
         * Getter for the turn wherefore this instance was configured
         * @return the autopilot turn object wherefore this instance was configured
         */
        private AutopilotTurn getTurn() {
            return turn;
        }

        /**
         * Getter for the turn physics engine used to calculate the physics related stuff for making the turn
         * @return the turn physics engine
         */
        private PhysXEngine.TurnPhysX getTurnPhysX() {
            return turnPhysX;
        }

        /**
         * Getter for the rotation around the center point of the turn that the drone has yet to do
         * before it has finished the turn
         * @return the rotation that the drone has to do before it has finished the turn in radians
         *         note: angleToGo is negative for clockwise rotations and positive for counter clockwise rotations
         */
        private float getAngleToGo() {
            return angleToGo;
        }

        /**
         * Decrements the angle to go with the given delta angle
         * @param deltaAngle the rotation around the center that was achieved during the last iteration
         *                   in radians
         *                   note: may me negative in the case that we need to do a clockwise turn, needs to be
         *                   positive if we're doing a counter clockwise turn
         */
        private void updateAngleToGo(float deltaAngle) {
            this.angleToGo -= deltaAngle;
        }

        /**
         * Getter for te roll that the drone needs to have to make the current turn
         * @return the reference roll for the drone to make the turn in radians.
         */
        private float getBankingTurnRoll() {
            return bankingTurnRoll;
        }

        /**
         * Setter for the roll required to make the specified turn
         * @param bankingTurnRoll the reference roll in radians
         */
        private void setBankingTurnRoll(float bankingTurnRoll) {
            this.bankingTurnRoll = bankingTurnRoll;
        }

        /**
         * Getter for the velocity needed for the drone to maintain a stable altitude during the turn
         * this is the reference velocity to be fed into the thrust controller
         * @return the reference velocity needed to be maintained during the turn in meters
         */
        private float getTurnVelocity() {
            return turnVelocity;
        }

        /**
         * Setter for the reference velocity needed to make the turn with a stable altitude
         * @param turnVelocity the reference velocity for the thrust controller in m/s
         */
        private void setTurnVelocity(float turnVelocity) {
            this.turnVelocity = turnVelocity;
        }

        /**
         * Getter for the lookahead distance, the distance between the drone and the pitch reference point
         * expressed in ground coordinates (the distance in the xz-plane)
         * @return the lookahead distance used to generate the pitch reference point in meters
         */
        private float getLookaheadDistance() {
            return lookaheadDistance;
        }

        /**
         * Setter for the lookahead distance of the drone (see getter for more info)
         * @param lookaheadDistance the distance used to generate the reference point in meters
         */
        private void setLookaheadDistance(float lookaheadDistance) {
            this.lookaheadDistance = lookaheadDistance;
        }

        /**
         * The turn that the turn state is specifying
         */
        private final AutopilotTurn turn;

        /**
         * The rotation angle that the drone has still to make before the turn is finished
         */
        private float angleToGo;

        /**
         * The roll needed to make the turn in the specified radius
         */
        private float bankingTurnRoll;

        /**
         * The velocity needed to fly in the same xz-plane during the turn (maintain a stable y-coordinate)
         */
        private float turnVelocity;

        /**
         * The distance between the drone and the pitch reference point measured in ground coordinates
         * this distance is used to generate the pitch reference point for the drone
         */
        private float lookaheadDistance = 100f;

        /**
         * The turn PhysX object used by the autopilot to make turns
         * --> calculates banking angles and such
         */
        private final PhysXEngine.TurnPhysX turnPhysX;

        /*
        Controllers for the flight
         */

        /**
         * Controllers needed for banking the drone for a given radius
         * --> thrust controller: needed for the cruise control maintaining the velocity
         * --> roll controller: needed to keep the roll stable to maintain a good bank
         * --> altitude controller: needed to keep the drone at the same level (constant height)
         * note: the roll and the altitude controllers need to react slowly to changes (so the control actions
         * do not go in overdrive, maybe use low pass filter?)
         */

        /**
         * Getter for the banking thrust PID controller, this PID controller needs to be passed
         * to the cruise control method implemented in the main controller class
         * @return a pid controller tuned to maintain the reference velocity of the autopilot
         */
        private PIDController getThrustController() {
            return thrustController;
        }

        /**
         * Getter for the roll controller, this controller is responsible for keeping the right rolling angle for
         * the drone to make a banking turn in the provided airspace
         * @return the PID controller tuned for reaching and maintaining the roll needed for banked turn
         */
        private PIDController getRollController() {
            return bankingRollController;
        }


        /**
         * Getter for the pitch controller of the turning controller, this is the controller for maintaining
         * the reference altitude during the turn (the reference altitude is defined as the cruising altitude, assigned
         * by the autopilot overseer)
         * @return the PID controller tuned for controlling the pitch
         */
        private PIDController getPitchController() {
            return pitchController;
        }

        private final static float THRUST_GAIN = 1.0f;
        private final static float THRUST_INTEGRAL = 0.2f ;
        private final static float THRUST_DERIVATIVE = 0.1f;
        private final PIDController thrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

        private final static float ROLL_GAIN = 0.04f;
        private final static float ROLL_INTEGRAL = 0.01f;
        private final static float ROLL_DERIVATIVE = 0.02f;
        private final PIDController bankingRollController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

        private final static float PITCH_GAIN = 1.0f;
        private final static float PITCH_INTEGRAL = 0.6f;
        private final static float PITCH_DERIVATIVE = 0.7f;
        private final PIDController pitchController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);
    }

    /**
     * A class responsible for specifying the control constants used for the flight between the two turns
     * also provides methods for controlling the drone during the flight between two turns
     * note: every time the drone has to navigate to a new turn, a new instance should be created
     */
    private class ToNextTurnControl {

        /**
         * Constructor for the control specifications for a flight between two
         * @param nextTurn the next turn to make, the drone needs the entry point such that it can correctly position
         *                 itself for entering the next turn
         */
        public ToNextTurnControl(AutopilotTurn nextTurn, AutopilotTurn previousTurn, PhysXEngine.TurnPhysX turnPhysX) {
            this.nextTurn = nextTurn;
            this.previousTurn = previousTurn;
            this.turnPhysX = turnPhysX;
            configureToNextTurn(nextTurn, previousTurn, turnPhysX);
        }

        /**
         * Configures the parameters needed to guide the flight to the next turn, this method is invoked to avoid
         * unnecessary calculations when determining the controls (since the results won't change during the flight
         * in between two turns)
         * @param nextTurn the next turn, this is the target of the drone for now
         * @param previousTurn the previous turn that the drone made (needed to calculate the proper exit point)
         * @param turnPhysX the turn physics used to calculate the parameters
         */
        private void configureToNextTurn(AutopilotTurn nextTurn, AutopilotTurn previousTurn, PhysXEngine.TurnPhysX turnPhysX){
            //get the parameters needed to calculate the velocity needed for a flight with constant altitude
            float mainWingStable = getMainStable();
            //calculate the reference velocity
            float referenceVelocity = turnPhysX.calcStableZeroPitchVelocity(mainWingStable);
            setReferenceVelocity(referenceVelocity);

            //calculate the connection vector for this flight, used to calculate a reference
            //first calculate the exit point relative to the turn center of the previous turn
            Vector prevEntryPointRel = previousTurn.getEntryPoint();
            float prevTurnAngle = previousTurn.getTurnAngle();
            Vector prevExitPointRel = turnPhysX.rotateTurnVector(prevEntryPointRel, prevTurnAngle);
            //now get the absolute exit point
            Vector prevTurnCenter = previousTurn.getTurnCenter();
            Vector prevExitPointAbs = prevExitPointRel.vectorSum(prevTurnCenter);

            //get the entry point for the next turn (relative to the center of the turn)
            Vector nextEntryPointRel = nextTurn.getEntryPoint();
            Vector nextPointCenter = nextTurn.getTurnCenter();
            Vector nextEntryPointAbs = nextEntryPointRel.vectorSum(nextPointCenter);
            //calculate the connection vector
            Vector connectionVector = nextEntryPointAbs.vectorDifference(prevExitPointAbs);
            //and save the result
            setPreviousExitPoint(prevExitPointRel);
            setConnectionVector(connectionVector);

            //must we add other features?
        }

        /**
         * Generates the control actions for the current flight in between the two specified turns
         * @param currentInputs the latest inputs received from the testbed
         * @param previousInputs the previous inputs received from the testbed
         * @return the control actions necessary to steer the drone to the entry point of the next turn
         */
        private AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //create the control outputs object to write the control actions to
            ControlOutputs outputs = new ControlOutputs(AirportNavigationController.this.getStandardOutputs());
            //get the reference point needed to calculate the control actions
            Vector referencePoint = this.calculateReferencePoint(currentInputs);
//            System.out.println("Reference point" + referencePoint);
            //get the control actions on the heading pitch and roll
            this.headingControl(outputs, currentInputs, previousInputs, referencePoint);
            this.pitchControl(outputs, currentInputs, previousInputs, referencePoint);
            //this.rollControl(outputs, currentInputs);
            //this.headingControlVertical(outputs, currentInputs, previousInputs, referencePoint);
            this.rollControl(outputs, currentInputs, previousInputs);

            //adjust such that there won't appear any AOA errors
            float aoaResMargin = AirportNavigationController.this.getAoaErrorMargin();
            AirportNavigationController.this.angleOfAttackControl(aoaResMargin, outputs, currentInputs, previousInputs);

            //get the thrust control actions
            this.thrustControl(outputs, currentInputs, previousInputs);

            outputs.capInclinations(getMainDeltaIncl(), getMainDeltaIncl(), getHorizontalDeltaIncl(), 0);
            return outputs;
        }

        /**
         * Checks if the controller is finished guiding the drone to the next turn
         * @param currentInputs the inputs most recently received from the testbed
         * @return true if the entry point of the next turn is not in front of the drone
         * TODO it could be that the drone is facing away from the target at a great angle and thus
         * TODO causing the reached objective to trigger even if we've not passed the radius (but good enough for now)
         */
        private boolean hasReachedEntryPoint(AutopilotInputs_v2 currentInputs){
            AutopilotTurn nextTurn = this.getNextTurn();
            //determine the target
            Vector turnCenter = nextTurn.getTurnCenter();
            Vector turnEntry = nextTurn.getEntryPoint();

            Vector target = turnCenter.vectorSum(turnEntry);
            return !targetInFrontOfDrone(currentInputs, target);
        }

        /**
         * Checks if the given target is in front of the drone
         * @param currentInputs the inputs most recently received from the testbed
         * @param target the target to check if it is in front of the drone (in world axis system)
         * @return true if and only if the vector pointing from the drone to the target
         *         transformed to the drone axis system and projected onto the heading vector of the drone
         *         has a negative sign along the z drone-axis (see implementation)
         *         --> 'in front' can be defined as looking along the (0,0,-1) direction in the
         *         drone axis system with a 180° viewing angle
         */
        private boolean targetInFrontOfDrone(AutopilotInputs_v2 currentInputs, Vector target){
            //grab the parameters needed for the calculation
            Vector dronePosition = extractPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);
            Vector headingDrone = new Vector(0,0,-1); //heading vector in drone axis

            //calc the difference vector, transform it to the drone axis system & project the transformed
            //difference vector onto the heading of the drone
            Vector diffVectorWorld = target.vectorDifference(dronePosition);
            Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, droneOrientation);
            Vector projDiffVector = diffVectorDrone.projectOn(headingDrone);

            //check the z-sign of the projection, if it is negative the target is in front of the drone (aligned along negative z-axis)
            //if the target is not in front the sign will be positive (the target is oriented along the positive z-axis)
            float zCompSign = signum(projDiffVector.getzValue());

            //if smaller than zero the target is still in front
            return zCompSign < 0;
        }

        /**
         * Generates the control actions for the pitch during the flight of the drone such that the drone steers
         * towards the reference point
         * @param outputs the outputs to write the results to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         * @param referencePoint the reference point to steer the drone to
         */
        private void pitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, Vector referencePoint){
            //get the reference value
            float pitchError = this.getPitchError(currentInputs, referencePoint);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            //feed the results into the pid controller
            PIDController pitchPid = this.getPitchPid();
            float pidResult = pitchPid.getPIDOutput(pitchError, deltaTime);
            //if the pid result is positive: the set point is larger than the input -> lower the pitch (pos inclination)
            //if the pid result is negative: the set point is smaller than the input -> increase the pitch (neg inclination)
            float horizontalInclination = getHorizontalStable() + pidResult;
            horizontalInclination = capInclination(horizontalInclination, getHorizontalStable(), getHorizontalDeltaIncl());
            outputs.setHorStabInclination(horizontalInclination);

        }

        /**
         * Generates the control actions for the heading during the flight of the drone. Steers the drone such that the
         * error on heading to the reference point decreases
         * @param outputs the outputs to write the generated control actions to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         * @param referencePoint the reference point to steer the drone to
         */
        private void headingControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, Vector referencePoint){
            //get the error on the heading and the delta time
            float headingError = this.getHeadingError(currentInputs, referencePoint);
            headingError = abs(headingError) < 10E-3 ? 0 : headingError; //to counter oscillating behavior
            float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
            //get the heading PID
            PIDController bankControl = this.getHeadingPid();
            float pidOutput = bankControl.getPIDOutput(headingError, deltaTime);
            errorLog((float) (atan(headingError)*this.getLookaheadDistance()));
            //and inclining the main wings
            float mainLeftWingInclination = getMainStable() + pidOutput;
            float mainRightWingInclination = getMainStable() - pidOutput;

            //get the cap on the inclinations (prevent over steering)
            mainLeftWingInclination = capInclination(mainLeftWingInclination, getMainStable(), getMainDeltaIncl());
            mainRightWingInclination = capInclination(mainRightWingInclination, getMainStable(), getMainDeltaIncl());

            outputs.setLeftWingInclination(mainLeftWingInclination);
            outputs.setRightWingInclination(mainRightWingInclination);
        }



        /**
         * Getter for the control actions needed on the drone to steer to the next target
         * objective of this contoller is to keep the drone as straight as possible
         * @param outputs the outputs to write the control actions to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the current roll (the error) & the other variables needed to calculate the control actions
            float roll = extractRoll(currentInputs);
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            if(abs(roll) < 1*PI/180){
                return;
            }

            //get the PID controller used to steer for the roll
            PIDController rollPid = this.getRollPID();
            //get the outputs based on the error (the roll itself is the error on the wanted situation)
            float pidResult = rollPid.getPIDOutput(roll, deltaTime);
            //if we need to roll to the left the roll is negative & the error generated will be positive
            //thus we need to lift the right main wing a larger inclination than the left one if we want to go to the left
            //the converse for a negative error we need to steer to the right (left wing larger than right)
            float leftWingInclination = getMainStable() - pidResult;
            float rightWingInclination = getMainStable() + pidResult;

            //cap the results
            leftWingInclination = capInclination(outputs.getLeftWingInclination() + leftWingInclination, getMainStable(), getMainDeltaIncl());
            rightWingInclination = capInclination(outputs.getRightWingInclination() + rightWingInclination, getMainStable(), getMainDeltaIncl());

            //save the results
            outputs.setLeftWingInclination(leftWingInclination);
            outputs.setRightWingInclination(rightWingInclination);
        }



        /**
         * Checks if the current output steers right
         * @param outputs the outputs that are generated by the controller
         * @return true if the drone is steering right
         */
        private boolean isSteeringRight(Controller.ControlOutputs outputs){
            return outputs.getRightWingInclination() < getMainStable();
        }

        /**
         * Checks if the current output steers left
         * @param outputs the outputs that are generated by the controller
         * @return true if the drone is steering left
         */
        private boolean isSteeringLeft(Controller.ControlOutputs outputs){
            return outputs.getRightWingInclination() > getMainStable();
        }


        /**
         * Calculates the thrust outputs for maintaining the reference velocity
         * needs to be called after the inclinations are known
         * @param outputs the outputs to write the control actions to and to read the wing inclinations from to determine
         *                the thrust needed to maintain the reference velocity
         * @param currentInputs the latest inputs from the testbed
         * @param previousInputs the previous inputs from the testbed
         */
        private void thrustControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the parameters needed to call the cruise control
            float referenceVelocity = this.getReferenceVelocity();
            PIDController thrustPID = this.getThrustPID();
            //call the controller method used for controlling the thrust for a given reference velocity
            //it changes the thrust in the outputs passed to it so we're finished here
            AirportNavigationController.this.flightCruiseControl(outputs, currentInputs, previousInputs, thrustPID, referenceVelocity);

        }

        /**
         * Calculates the reference point to guide the controller to the start of the next turn
         * @param currentInputs the latest inputs received by the autopilot from the testbed
         * @return a reference point for the controls in the world axis system (absolute)
         * note: the reference point is based on the current location of the drone and the connection vector
         * of the previous and next turn (may be changed in the future)
         */
        private Vector calculateReferencePoint(AutopilotInputs_v2 currentInputs){
            //grab the parameters and other stuff needed to calculate the exit point
            Vector prevExitPoint = this.getPreviousExitPoint();
            Vector connectionVector = this.getConnectionVector();
            Vector previousTurnCenter = this.getPreviousTurn().getTurnCenter();
            Vector dronePosition = extractPosition(currentInputs);
            float lookaheadDistance = this.getLookaheadDistance();
            float refAltitude = AirportNavigationController.this.getCruisingAltitude();
            Vector altitudeVector = new Vector(0,refAltitude,0);

//            System.out.println();
//            System.out.println("Previous turn exit point: " + prevExitPoint);
//            System.out.println("Connection vector: " + connectionVector);
//            System.out.println("next turn entry: " + nextTurn.getEntryPoint());

            //we want to calculate the reference point as the orthogonal projection of the drone position
            //relative to the ideal path start point onto the vector indicating the ideal path

            //first calculate the absolute position of the exit point
            Vector exitPointGround = prevExitPoint.vectorSum(previousTurnCenter);
            Vector exitPoint = exitPointGround.vectorSum(altitudeVector); //the exit point at the given altitude

            //get the drone position relative to the exit point (is relative to the reference altitude)
            Vector exitRelativeDrone = dronePosition.vectorDifference(exitPoint);

            //project the relative drone position onto the ideal path (connection vector) to get the progress relative
            //to the end point (ideal path is of form (x, 0, z)
            Vector idealDronePos = exitRelativeDrone.projectOn(connectionVector);

            //we need a reference point for the ideal path of the drone therefore we place a reference point
            //lookahead distance meters away from the projection along the ideal path
            Vector lookaheadVector = connectionVector.normalizeToLength(lookaheadDistance);

            //then we add the lookahead vector to the ideal drone position (giving a vector of form (x, 0, z)
            Vector referenceRelToExit = idealDronePos.vectorSum(lookaheadVector);

            //finally set the reference point to absolute values by adding the exit point
            //(our calculations were relative against this point)
            return referenceRelToExit.vectorSum(exitPoint);
        }

        /**
         * Calculates the error in the heading of the drone against the given reference point
         * the error contains the magnitude of the error as well as the direction of the error
         * @param currentInputs the inputs most recently received from the testbed
         * @param referencePoint the reference point to calculate the error against
         * @return the error angle between the current heading vector and the vector pointing from the drone to the
         *         reference point ( =difference vector). The angle is positive if the difference vector is to the
         *         left of the heading vector and negative if to the right.
         */
        private float getHeadingError(AutopilotInputs_v2 currentInputs, Vector referencePoint){
            //grab the parameters & other stuff to calculate the heading error for the controller
            Vector headingDroneHa = new Vector(0,0,-1 ); //the heading vector in the heading axis
            Vector xzNormal = new Vector(0,1,0);
            Vector dronePosition = extractPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);

            //transform the heading vector onto the world
            Vector headingWorld = PhysXEngine.headingOnWorld(headingDroneHa, droneOrientation);

            //get the difference vector between the drone and the set point (points from drone to set point)
            Vector diffVector = referencePoint.vectorDifference(dronePosition);
            //project both of them on the xz plane
            Vector headingWorldXZ = headingWorld.orthogonalProjection(xzNormal);
            Vector diffVectorXZ = diffVector.orthogonalProjection(xzNormal);

            //calculate the angle between the heading of the world and the difference vector
            float headingAngle = headingWorldXZ.getAngleBetween(diffVectorXZ);

            //now we also need to get the direction of the angle (vector product between heading and the difference)
            //headingWorld.crossProduct(diffVector) is positive if diff is to the left of the heading
            //and negative if the difference vector is to the right of the heading vector (the sign of the resulting y component)
            float direction = signum(headingWorldXZ.crossProduct(diffVectorXZ).getyValue());

            //the resulting error
            float headingError = direction*abs(headingAngle);

            //for some reason the result can be NaN, better to avert controller spouting errors
            //just return some default value that will cause no harm(?)
            return Float.isNaN(headingError) ? 0 : headingError;
        }

        /**
         * Calculates the error on the pitch of the drone against the reference point of the flight
         * the pitch error is the angle between the heading vector (zero roll) in the pitch axis system
         * and the vector pointing from the drone to the reference point ( =difference vector) projected onto
         * the yz-plane of the pitch axis system
         * @param currentInputs the inputs most recently received from the testbed
         * @param referencePoint the reference point to calculate the pitch error for
         * @return the angle as described above with the sign indicating the direction of the error
         *         if the pitch error is positive, extra pitch is required (steering upwards)
         *         if the pitch error is negative, lesser pitch is required (steering downwards)
         */
        private float getPitchError(AutopilotInputs_v2 currentInputs, Vector referencePoint){
            //grab the parameters needed fo the calculation
            Vector droneHeadingPa = new Vector(0,0,-1); //the heading vector of the drone in pitch axis
            Vector yzNormalPa = new Vector(1,0,0); // normal of the yz-plane in pitch axis
            Vector dronePosition = extractPosition(currentInputs);
            Vector droneOrientation = extractOrientation(currentInputs);

            //calculate the difference vector and transform it to the pitch axis system
            //difference vector points from the drone position to the reference point
            Vector diffVector =  referencePoint.vectorDifference(dronePosition);
            Vector diffPa = PhysXEngine.worldOnPitch(diffVector, droneOrientation);

            //project the diff vector in pitch axis system onto the yz plane of the pitch axis
            Vector projDiffPa = diffPa.orthogonalProjection(yzNormalPa);

            //calculate the angle between the heading vector and the projected diff vector
            float pitchErrorAngle = projDiffPa.getAngleBetween(droneHeadingPa);

            //also calculate the direction of the error, take the vector product of the heading and the diff vector
            //if the x-component is positive the drone has to pitch upwards, if it is negative the drone has
            //to pitch downwards to reach the desired altitude/pitch
            float direction = signum(droneHeadingPa.crossProduct(projDiffPa).getxValue());

            //multiply to get the error
            float pitchError = direction*abs(pitchErrorAngle);

            //return the result while checking for NaN, if so return harmless (?) value
            return Float.isNaN(pitchError) ? 0 : pitchError;

        }


        /**
         * Getter for the next turn that has to be made, will be used to configure the controller for the flight
         * to the next turn
         * @return the autopilot turn object containing the specifications needed to configure the controller
         *         for the flight between the two turns
         */
        private AutopilotTurn getNextTurn() {
            return nextTurn;
        }

        /**
         * Getter for the previous turn, the turn that the drone exited before flying to the next one
         * this turn's exit point is used in determining the set point for the flight
         * @return the specifications of the previous turn
         */
        private AutopilotTurn getPreviousTurn() {
            return previousTurn;
        }

        /**
         * The velocity that is needed to fly at a constant altitude while flying to the next turn
         * this velocity will be used by the cruise control as the setpoint
         * @return the refence velocity in m/s
         */
        private float getReferenceVelocity() {
            return referenceVelocity;
        }

        /**
         * Setter for the reference velocity of the cruise control
         * @param referenceVelocity the reference used for cruise control in meters
         */
        private void setReferenceVelocity(float referenceVelocity) {
            this.referenceVelocity = referenceVelocity;
        }

        /**
         * Getter for the connection vector, this is the vector that points from the exit point of the previous turn
         * to the entry point of the current turn (this variable is saved to prevent unnecessary calculations
         * this vector is always located at ground level
         * @return the vector that points from the previous exit point to the next entry point
         *         with length the distance between the two points in meters
         * note: this variable will ultimately be used to calculate the reference point for the controllers
         * that control the flight in between two turns
         */
        private Vector getConnectionVector() {
            return connectionVector;
        }

        /**
         * Setter for the connection vector, see getter for more info
         * @param connectionVector the vector that connects the previous exit point and the current entry point
         *                         of the turns (in meters)
         */
        private void setConnectionVector(Vector connectionVector) {
            this.connectionVector = connectionVector;
        }

        /**
         * Getter for the exit point of the previous turn, relative to the center of the previous turn
         * this method exists to avoid re-calculation of the same point over and over again
         * @return the exit point of the previous turn in meters (is always located at ground level)
         */
        private Vector getPreviousExitPoint() {
            return previousExitPoint;
        }

        /**
         * Setter for the exit point of the previous turn, relative to the center of the previous turn
         * @param previousExitPoint the exit point of the previous turn (on ground level) specified in meters
         */
        private void setPreviousExitPoint(Vector previousExitPoint) {
            this.previousExitPoint = previousExitPoint;
        }

        /**
         * Getter for the lookahead distance, the distance from the orthogonal projection of the drone position
         * on the connection line, to the reference point for the controller to guide the flight
         * @return the lookahead distance in meters
         */
        private float getLookaheadDistance() {
            return lookaheadDistance;
        }

        /**
         * Setter for the lookahead distance (see getter for more info)
         * @param lookaheadDistance lookahead distance in meters
         */
        private void setLookaheadDistance(float lookaheadDistance) {
            this.lookaheadDistance = lookaheadDistance;
        }

        /**
         * The turn physics engine used to generate the control parameters for the flight in between turns
         * @return the turn physics needed for parameter configuration
         */
        private PhysXEngine.TurnPhysX getTurnPhysX() {
            return turnPhysX;
        }

        /**
         * The next turn that contains the entry point that we have to navigate to
         */
        private final AutopilotTurn nextTurn;

        /**
         * The turn that was previously made by the controller, use the exit point to calculate
         * a reference for the inter turn controller
         */
        private final AutopilotTurn previousTurn;

        /**
         * The turn physics used for calculating the control parameters needed to fly to the next turn
         */
        private final PhysXEngine.TurnPhysX turnPhysX;

        /**
         * The reference velocity for the thrust controls, needs to be maintained for stable flight to the next target
         */
        private float referenceVelocity;

        /**
         * The connection vector, this is the vector that points from the exit point of the previous turn
         * to the entry point of the next turn. This vector is used to calculate a reference for the controllers
         * that manage the flight in between turns (in world axis system)
         */
        private Vector connectionVector;

        /**
         * The exit point of the turn that was previously made, relative to the center point
         * of the previous turn (in the world axis system)
         */
        private Vector previousExitPoint;

        /**
         * The lookahead distance, this is the distance used to place the reference point for the heading
         * and the pitch of the drone. More specifically, for calculating the reference point the current position
         * of the drone is orthogonally projected onto the connection vector, and the reference point is set
         * 'lookaheadDistance' away from that projected point
         */
        private float lookaheadDistance = 100f;

        /*
        Some control variables & getters for them
         */

        /**
         * Getter for the roll threshold used by the controller to cap the maximum roll experienced by the drone
         * during the flight between two turns (to prevent the drone from crashing)
         * @return the threshold on the roll before activating roll control in radians
         */
        private float getRollThreshold() {
            return rollThreshold;
        }

        /**
         * Setter for the roll threshold (see getter for more information)
         * @param rollThreshold the roll threshold to use in radians
         */
        private void setRollThreshold(float rollThreshold) {
            this.rollThreshold = rollThreshold;
        }

        /**
         * Getter for the maximum deviation from the stable inclination for the vertical stabilizer
         * @return the maximum inclination deviance from te stable vertical inclination in radians
         */
        private float getVerticalDelta() {
            return verticalDelta;
        }

        /**
         * Getter for the pid controller that is used to correct the pitch of the drone during the flight between
         * two turns
         * @return the pid controller tuned for the error on the pitch of the drone
         */
        private PIDController getPitchPid() {
            return pitchPid;
        }

        /**
         * Getter for the pid controller that is used to correct the heading of the drone during the flight between
         * two turns
         * @return the pid controller tuned for the error on the heading of the drone
         */
        private PIDController getHeadingPid() {
            return headingPid;
        }

        /**
         * Getter for the pid that controls the thrust of the drone during the flight between two turns
         * @return the pid controller tuned for the flight between two turns
         */
        private PIDController getThrustPID() {
            return thrustPID;
        }


        /**
         * Getter for the roll PID, used to control the roll during the flight to the next point
         * @return a pid controller tuned for keeping the roll to the reference point
         */
        private PIDController getRollPID() {
            return rollPID;
        }

        /**
         * The roll that the drone may maximally experience during the flight to the next turn
         * this parameter is used to 'lock' the drone into a banking angle
         * If the roll threshold is breached the roll control is activated
         */
        private float rollThreshold = (float) (5*PI/180f);

        /**
         * The maximum deviance in the vertical stabilizer inclination from the stable inclination
         * TODO beta usage, verify correct functioning
         */
        private float verticalDelta = (float) (5*PI/180);

        /**
         * The control parameters & the pitch pid for controlling the pitch of the drone
         * during the flight in between two turns
         */
        private final static float PITCH_GAIN = 1.0f;
        private final static float PITCH_INTEGRAL  = 0.4f;
        private final static float PITCH_DERIVATIVE = 0.2f;
        private PIDController pitchPid = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);

        /**
         * The control parameters & the heading pid for controlling the heading of the drone
         * during the flight between two turns
         */
        private final static float HEADING_GAIN = 0.8f;
        private final static float HEADING_INTEGRAL = 0.0f;
        private final static float HEADING_DERIVATIVE = 1.0f ;
        private PIDController headingPid = new PIDController(HEADING_GAIN, HEADING_INTEGRAL, HEADING_DERIVATIVE);

        /**
         * Control parameters & the thrust pid for controlling the thrust of the drone during the flight between
         * two turns
         */
        private final static float THRUST_GAIN = 1.0f;
        private final static float THRUST_INTEGRAL = 0.0f;
        private final static float THRUST_DERIVATIVE = 0.0f;
        private PIDController thrustPID = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);




        /**
         * Control parameters & the roll PID for controlling the roll of the drone (beta function)
         */
        private final static float ROLL_GAIN = 1.0f;
        private final static float ROLL_INTEGRAL = 0.4f;
        private final static float ROLL_DERIVATIVE = 0.3f;
        private final PIDController rollPID = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);
    }
}

/*
Code graveyard for the turning controller
 */


//    /**
//     * Generates the control actions for maintaining the altitude of the drone
//     * @param outputs the outputs to write the altitude controls to
//     * @param currentInputs the inputs the most recent received by the autopilot from the testbed
//     * @param previousInputs the inputs previously received by the drone
//     */
//    @Deprecated
//    private void altitudeControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
//        //get the assigned cruising altitude
//        float cruisingAltitude = AirportNavigationController.this.getCruisingAltitude();
//        float currentAltitude = extractAltitude(currentInputs);
//
//        //get the difference in altitude (the error in the altitude)
//        float altitudeDiff = cruisingAltitude - currentAltitude;
//
//        //get the pid and the other parameters needed (eg delta time)
//        float deltaTime = getDeltaTime(currentInputs, previousInputs);
//        PIDController altitudeController  = this.getBankingAltitudeController();
//        float pidOutput = altitudeController.getPIDOutput(altitudeDiff, deltaTime);
//
//        //now adjust for the error that was made
//        float desiredHorizontal = pidOutput + getHorizontalStable();
//        float horizontalInclination = capInclination(desiredHorizontal,getHorizontalStable(), getHorizontalDeltaIncl());
//
//        outputs.setHorStabInclination(horizontalInclination);
//
//    }
//    /**
//     * Getter for the altitude controller, this controller is responsible for maintaining the reference altitude
//     * in this case the assigned cruising altitude by the autopilot overseer
//     * @return the PID controller tuned for maintaining the assigned altitude during the banking turn
//     */
//    @Deprecated
//    private PIDController getBankingAltitudeController() {
//        return bankingAltitudeController;
//    }
//private final static float ALTITUDE_GAIN = 0.0008f;
//    private final static float ALTITUDE_INTEGRAL = 0.00008f;
//    private final static float ALTITUDE_DERIVATIVE = 0.006f;
//    @Deprecated
//    private final PIDController bankingAltitudeController = new PIDController(ALTITUDE_GAIN, ALTITUDE_INTEGRAL, ALTITUDE_DERIVATIVE);

/*
Code graveyard for the to next turn controller
 */
//    /**
//     * A controller that generates control actions for the heading of the drone via steering with the
//     * vertical stabilizer (beta function)
//     * @param outputs  the outputs object to write the result of the calculations to
//     * @param currentInputs the inputs most recently received from the testbed
//     * @param previousInputs the inputs previously received from the testbed
//     * @param referencePoint the reference point used to calculate the heading error from
//     */
//    private void headingControlVertical(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, Vector referencePoint){
//        //get the current heading error
//        float headingError = this.getHeadingError(currentInputs, referencePoint);
//        float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
//        errorLog(headingError);
//        //get the controller responsible for the vertical stabilizer
//        PIDController verticalPid = this.getVerticalPID();
//        //get the outputs for the PID controller
//        float pidOutputs = verticalPid.getPIDOutput(headingError, deltaTime);
//        //generate the control actions (inclinations) based on the error
//        //if the inclination of the vertical stabilizer is positive the drone will steer to the left
//        //if the heading error is positive (and the pid outputs thus negative) we'll have to steer to the left
//        //change the sign of the pid outputs before passing them to the outputs
//        float verticalInclination = getVerticalStable() - pidOutputs;
//        //cap the result
//        verticalInclination = capInclination(verticalInclination, getVerticalStable(), getVerticalDelta());
//        //save it
//        outputs.setVerStabInclination(verticalInclination);
//    }
//    /**
//     * Controls the roll of the drone during the flight from turn to turn
//     * If the specified roll threshold (see getRollThreshold())  is breached, the drone is 'locked' into
//     * the current roll by keeping the main wings at the same inclination, the drone can become 'unlocked' if
//     * the controller tries to roll in a different direction
//     *
//     * @param outputs the outputs object to write the control actions to
//     * @param currentInput the most recently received inputs from the testbed
//     */
//    private void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInput){
//        float rollThreshold = this.getRollThreshold();
//        float roll = extractRoll(currentInput);
//
//        if(roll >= rollThreshold&&isSteeringLeft(outputs)){
//            outputs.setRightWingInclination(getMainStable());
//            outputs.setLeftWingInclination(getMainStable());
//        }
//        else if(roll <= - rollThreshold&&isSteeringRight(outputs)){
//            outputs.setLeftWingInclination(getMainStable());
//            outputs.setRightWingInclination(getMainStable());
//
//        }else{
//            // change nothing
//        }
//    }
//    /**
//     * Getter for the vertical PID, the pid controller that is tuned for generating the error factor for
//     * the vertical stabilizer of the drone
//     * @return a pid controller tuned for the vertical stabilizer
//     */
//    private PIDController getVerticalPID() {
//        return verticalPID;
//    }
//    /**
//     * Control parameters & the vertical stabilizer PID for controlling the heading of the drone (beta function)
//     */
//    private final static float VERTICAL_GAIN = 1.0f;
//    private final static float VERTICAL_INTEGRAL = 0.0f;
//    private final static float VERTICAL_DERIVATIVE = 0.0f;
//    private final PIDController verticalPID = new PIDController(VERTICAL_GAIN, VERTICAL_INTEGRAL, VERTICAL_DERIVATIVE);