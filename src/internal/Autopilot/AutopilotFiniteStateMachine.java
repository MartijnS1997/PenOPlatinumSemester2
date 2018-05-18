package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import TestbedAutopilotInterface.Overseer.AutopilotDelivery;
import TestbedAutopilotInterface.Overseer.AutopilotInfo;
import TestbedAutopilotInterface.Overseer.MapAirport;
import internal.Helper.Vector;

import static internal.Autopilot.AutopilotState.*;

/**
 * Created by Martijn on 3/04/2018.
 * The finite state machine describing the behavior of the autopilot during the simulation
 * note: changes since the controller selector
 *       --> the machine does not set inputs for the next controller
 *       --> the machine does not save any inputs for future reference
 */
public class AutopilotFiniteStateMachine {

    /**
     * Constructor for the finite state machine that is responsible for getting the control actions for the drone
     * manages all the controller activity for the autopilot
     * @param autopilot the autopilot the state machine is working for
     */
    public AutopilotFiniteStateMachine(AutoPilot autopilot){
        //create all the controllers
        this.takeoffController = new AutopilotTakeoffController(autopilot);
        this.takeoffStabilizerController = new AutopilotStabilization(autopilot);
        this.airportNavigationController = new AirportNavigationController(autopilot);
        this.descendWaitController = new DescendWaitController(autopilot);
        this.descendController = new DescendController(autopilot);
        this.landingController = new AutopilotLandingController(autopilot);
        this.gateTaxiingController = new GateTaxiingController(autopilot);
        this.runwayTaxiingController = new RunwayTaxiingController(autopilot);
        this.autopilot = autopilot;

    }

    /**
     * Getter for the outputs of the finite state machine that describes the autopilot
     * @param currentInputs the current inputs for the state machine, will also be saved for the next iteration
     * @return the control outputs generated by the currently active controller
     */
    public AutopilotOutputs getMachineOutput(AutopilotInputs_v2 currentInputs){
        //check the state that we should start next
        AutopilotState activeState = toNextState(currentInputs);
        //get the controller that needs to be active
        Controller controller = this.getStateController(activeState);
        //get the controller outputs
        AutopilotInputs_v2 previousInputs = this.getPreviousInputs();
        AutopilotOutputs outputs = controller.getControlActions(currentInputs, previousInputs);
        //save the inputs for the next round
        this.updatePreviousOutputs(currentInputs);
        this.generateInfo(currentInputs, previousInputs, activeState);

        //return the result
        return outputs;
    }


    /**
     * Getter for the state that will be active during the next iteration (not the logically next state)
     * @param currentInputs the inputs used to determine the next state
     * @return the next active state of the autopilot
     */
    private AutopilotState getNextActiveState(AutopilotInputs_v2 currentInputs, AutopilotState currentState){
        //get the previous inputs
        AutopilotInputs_v2 previousInputs = this.getPreviousInputs();
        //determine which controller is next
        switch(currentState){
            case INIT_FLIGHT:
                //check if we've already passed a single iteration (needed to configure the autopilot)
                return this.getInitController().hasReachedObjective(currentInputs, previousInputs) ? TAKEOFF : INIT_FLIGHT;
            case TAKEOFF:
                //get the takeoff controller
                AutopilotTakeoffController takeoffController = this.getTakeoffController();
                //check if it has reached its objective, if so return the next state, if not continue the takeoff
                return takeoffController.hasReachedObjective(currentInputs, previousInputs ) ? STABILIZE_TAKEOFF : TAKEOFF;
            case STABILIZE_TAKEOFF:
                //get the takeoff stabilization controller
                AutopilotStabilization stabilizationController = this.getTakeoffStabilizerController();
                return stabilizationController.hasReachedObjective(currentInputs, previousInputs) ? /*DESCEND_WAIT*/ FLIGHT: STABILIZE_TAKEOFF;
            case FLIGHT:
                AirportNavigationController flightController = this.getAirportNavigationController();
                //check if the controller has finished doing its job
                return flightController.hasReachedObjective(currentInputs, previousInputs) ? /*DESCEND*/ DESCEND_WAIT : FLIGHT;
            case DESCEND_WAIT:
                DescendWaitController descendWaitController = this.getDescendWaitController();
                return descendWaitController.hasReachedObjective(currentInputs, previousInputs) ? DESCEND : DESCEND_WAIT;
            case DESCEND:
                DescendController descendController = this.getDescendController();
                //check if the controller is done doing its job
                return descendController.hasReachedObjective(currentInputs, previousInputs) ? LANDING : DESCEND;
            case LANDING:
                AutopilotLandingController landingController = this.getLandingController();
                //check if we're on ground, if not continue landing, otherwise, start taxiing
                return landingController.hasReachedObjective(currentInputs, previousInputs) ? TAXIING_TO_GATE : LANDING;
            case TAXIING_TO_GATE:
                GateTaxiingController taxiingControllerGate = this.getGateTaxiingController();
                //check if we've reached the gate, if not keep on going, if so start taxiing to the runway
                return taxiingControllerGate.hasReachedObjective(currentInputs, previousInputs) ? TAXIING_TO_RUNWAY : TAXIING_TO_GATE;
            case TAXIING_TO_RUNWAY:
                RunwayTaxiingController taxiingControllerRunway = this.getRunwayTaxiingController();
                //check if we've reached the runway, if so start the takeoff, if not keep taxiing
                return taxiingControllerRunway.hasReachedObjective(currentInputs, previousInputs) ? TAKEOFF : TAXIING_TO_RUNWAY;
            default:
                //Default action, may change later (determine based on the inputs which state should be appropriate
                return TAKEOFF;
        }

    }

    /**
     * Advances the finite state machine to the next state (may be the same one as before)
     * while setting the right parameters for each controller
     * @param inputs the inputs to base the configuration on
     * @return the state that needs to be active to generate the next outputs
     * note: may change the configuration of the controllers during the method call (eg targets may be changed)
     * note: usecase --> should be called every iteration to get the next state
     */
    private AutopilotState toNextState(AutopilotInputs_v2 inputs){

        //get the current state
        AutopilotState currentState = this.getState();
        //get the next state needed by the controller
        AutopilotState nextState = this.getNextActiveState(inputs, currentState);

        //check if they are the same, if not, configure the controller & save the next state
        if(!nextState.equals(currentState)){
            configureState(nextState, inputs);
            this.setState(nextState);
            System.out.println("\ndrone ID: " + this.getAutopilot().getID());
            System.out.println("Switched states, from " + AutopilotState.getString(currentState) +
                    ", to " + AutopilotState.getString(nextState));
            System.out.println("current position: " + Controller.extractPosition(inputs));
            //we do a recursive call until the states are stable
            toNextState(inputs);
        }

        //now return the next state
        return nextState;
    }

    /**
     * Configure the controller responsible for the given state
     * @param state the state to configure the controller for
     * @param inputs the inputs used to (partly) configure the controller
     * note: doesn't do any configuring when receiving init state
     */
    private void configureState(AutopilotState state, AutopilotInputs_v2 inputs){

        switch(state){
            case TAKEOFF:
                configureTakeoff(inputs);
                break;
            case STABILIZE_TAKEOFF:
                configureTakeoffStabilization(inputs);
                break;
            case FLIGHT:
                configureFlight(inputs);
                break;
            case DESCEND_WAIT:
                configureDescendWait(inputs);
                break;
            case DESCEND:
                configureDescend(inputs);
                break;
            case LANDING:
                configureLanding(inputs);
                break;
            case TAXIING_TO_GATE:
                configureGateTaxiing(inputs);
                break;
            case TAXIING_TO_RUNWAY:
                configureRunwayTaxiing(inputs);
                break;
        }
    }

    /**
     * Configures the takeoff controller, must be called every time the controller
     * is changed to the takeoff controller (is used to set the new target)
     * @param inputs the inputs  to configure the controller with
     */
    private void configureTakeoff(AutopilotInputs_v2 inputs){
        //set the cruising altitude
        float cruisingAlt = this.getAutopilot().getCommunicator().getAssignedCruiseAltitude();
//        System.out.println("Assigned altitude for this flight: " + cruisingAlt);
        AutopilotTakeoffController takeoffController = this.getTakeoffController();
        takeoffController.reset();
        //configure the controller
        AutopilotConfig config = this.getAutopilot().getConfig();
        takeoffController.configureController(config);
    }

    /**
     * Configures the stabilization controller responsible for stabilizing the drone after a takeoff
     * must be called every time the state switches to takeoff stabilization
     * @param inputs the inputs used to configure the stabilizer
     */
    private void configureTakeoffStabilization(AutopilotInputs_v2 inputs){
        //we need to release the lock on the airport
        //TODO create more general API for the requests to be called --> the collision avoidance overlay
        this.getAutopilot().getCommunicator().removeRequest();
        //set the cruising altitude
        float cruisingAlt = this.getAutopilot().getCommunicator().getAssignedCruiseAltitude();
        AutopilotStabilization stabilization = this.getTakeoffStabilizerController();
        stabilization.reset();
        stabilization.setCruisingAltitude(cruisingAlt);

        //set the config for the controller
        AutopilotConfig config = this.getAutopilot().getConfig();
        stabilization.setConfig(config);
    }

    /**
     * Configures the flight controller, must be called every time the controller is switched to the
     * flight controller --> used to configure for the flight to the next airport
     * @param inputs the inputs  to configure the controller with
     */
    private void configureFlight(AutopilotInputs_v2 inputs){

        AirportNavigationController navigationController = this.getAirportNavigationController();

        //first we need to clear the controller
        navigationController.reset();

        AutoPilot autopilot = this.getAutopilot();
        AutopilotCommunicator communicator = autopilot.getCommunicator();
        AutopilotConfig config = autopilot.getConfig();
        float cruisingAltitude = communicator.getAssignedCruiseAltitude();
        float descendThreshold = this.getLandingDescendThreshold();
        float standardLandingAlt = this.getStandardLandingAltitude();

        //get the data about the package that we currently need to deliver
        AutopilotDelivery currentDelivery = communicator.getCurrentRequest();

        //select the airport needed by the navigator
        if(!currentDelivery.isPickedUp()){
            //if the delivery is not yet picked up, get the source address
//            System.out.println("need to pick up package at source airport");
            int sourceAirportID = currentDelivery.getSourceAirport();
            MapAirport targetAirport = communicator.getAirportByID(sourceAirportID);
//            System.out.println("target airport: " + targetAirport);
            //generate the path generator for the controller
            PathGenerator_v2 pathGenerator = new PathGenerator_v2(inputs, targetAirport, cruisingAltitude, descendThreshold, standardLandingAlt);
            navigationController.configureNavigation(config, pathGenerator);

        }else{
            //if the delivery is already picked up, get to the destination address
//            System.out.println("package must be delivered to destination airport");
            int destinationAirportID = currentDelivery.getDestinationAirport();
            MapAirport targetAirport = communicator.getAirportByID(destinationAirportID);
//            System.out.println("target airport: " + targetAirport);
            //create the path generator
            PathGenerator_v2 pathGenerator = new PathGenerator_v2(inputs, targetAirport, cruisingAltitude, descendThreshold, standardLandingAlt);
            navigationController.configureNavigation(config, pathGenerator);
        }



        //TODO implement, use the information available on the packages to set the parameters needed to fly to
        //TODO next airport for delivery --> all the info needed is in the autopilot communicator class
        //TODO can be accessed via api chain: this.getAutopilot().getCommunicator();
    }


    /**
     * Configures the descend wait controller
     * @param inputs the inputs used to configure the wait controller
     */
    private void configureDescendWait(AutopilotInputs_v2 inputs){
        DescendWaitController descendWaitController = this.getDescendWaitController();
        descendWaitController.reset();
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        float cruisingAltitude = communicator.getAssignedCruiseAltitude();
        AutopilotDelivery delivery = communicator.getCurrentRequest();
        int airportToLock = delivery.isPickedUp() ? delivery.getDestinationAirport() : delivery.getSourceAirport();

        descendWaitController.configureWaitController(inputs,cruisingAltitude,airportToLock);
        descendWaitController.setConfig(this.getAutopilot().getConfig());
    }
    /**
     * Configures the descend controller for the drone this is the controller used to descend to a landing
     * friendly altitude once an airport has been reached, will be skipped if the altitude reaches a certain
     * threshold --> should always be called if the controller is used for another descend
     * @param inputs_v2 the inputs used to configure the controller with
     */
    private void configureDescend(AutopilotInputs_v2 inputs_v2){
        //always reset before usage
        DescendController descendController = this.getDescendController();
        descendController.reset();

        AutopilotConfig config = this.getAutopilot().getConfig();

        float activationThreshold = this.getLandingDescendThreshold();
        float targetAltitude = this.getStandardLandingAltitude();
        System.out.println("target altitude of drone: " + targetAltitude);
        descendController.configureController(config,inputs_v2, activationThreshold, targetAltitude);

    }

    /**
     * Configures the landing controller to safely land at the airport at which the next package needs to be delivered
     * @param inputs the inputs  to configure the controller with
     */
    private void configureLanding(AutopilotInputs_v2 inputs){
        AutopilotLandingController landingController = this.getLandingController();
        landingController.reset();

    	//get airport where to land
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        AutopilotDelivery delivery = communicator.getCurrentRequest();
        //check if the delivery is picked up or not, if not we fly to the source airport otherwise to the destination
        int airportID = delivery.isPickedUp() ? delivery.getDestinationAirport() : delivery.getSourceAirport();

        //set the target airport
    	MapAirport airport = communicator.getAirportByID(airportID);
    	landingController.setTargetAirport(airport);

    	//Set configuration of the autopilot
    	AutopilotConfig config = this.getAutopilot().getConfig();
        landingController.setConfig(config);
        //TODO implement, use the landing information contained in the overseer (and the delivery package)
        //TODO to get the right angle for the landing, we may also ship the "get in place" functionality
        //TODO to the flight controller. To access the package use the api chain this.getAutopilot().getCommunicator().getCurrentRequest()
    }

    /**
     * Configures the taxiing controller for the gate taxiing phase, where the drone taxis to the gate where
     * to pick up/deliver the package
     * @param inputs the inputs to configure the controller with
     */
    private void configureGateTaxiing(AutopilotInputs_v2 inputs){
        //TODO implement, set the taxiing target to the gate that needs to be reached to get the package
        //TODO a package is acquired if the drone moves slower than 1m/s and is in a 5m range of the gate
        //TODO use the method setTarget() for the gate taxiing controller (with null for the direction)
        //TODO access the package information via the API chain this.getAutopilot().getCommunicator().getCurrentRequest()
        //TODO get the gate location using the following API chain
        //TODO this.getAutopilot().getCommunicator().getDelivery().getDestinationGate()
        //TODO and then calling the .getAirportByID() in the communicator class


        GateTaxiingController taxiingController = this.getGateTaxiingController();
        taxiingController.reset();
        //package that has to be delivered
        AutopilotDelivery packageToDeliver = this.getAutopilot().getCommunicator().getCurrentRequest();

        taxiingController.configureGateTaxiing(inputs, packageToDeliver);

        //set the config for the controller
        AutopilotConfig config = this.getAutopilot().getConfig();
        taxiingController.setConfig(config);

    }

    private void configureRunwayTaxiing(AutopilotInputs_v2 inputs){
        //TODO implement, set the taxiing target to the runway to takeoff to the next airport
        //TODO first acquire the takeoff direction from the overseer and then get in position
        //TODO set the target of the taxiing controller with setTarget(RunwayPos, Direction) so that
        //TODO the taxiing controller knows in which direction to stand for takeoff
        //TODO get the current airport to configure for by the following API chain:
        //TODO this.getAutopilot().getCommunicator().getAirportAtLocation();

        RunwayTaxiingController taxiingController = this.getRunwayTaxiingController();
        taxiingController.reset();
        //AutopilotDelivery packageToDeliver = this.getAutopilot().getCommunicator().getCurrentRequest();

        //taxiingController.runwayTaxiing(inputs);

        //set the config for the controller
        AutopilotConfig config = this.getAutopilot().getConfig();
        taxiingController.setConfig(config);
        taxiingController.configureRunwayController(inputs);

    }

    /**
     * Generates the info to be sent over to the overseer & saves it to the info variable
     * this information should contain all the info needed to do collision detection and avoidance
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @param currentState the current state of the autopilot (eg flight, takeoff etc)
     */
    private void generateInfo(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, AutopilotState currentState){
        AutoPilot autopilot = this.getAutopilot();
        //get the id of the drone to add to the info
        String droneID = autopilot.getID();
        //get the current position and the previous position
        Vector currentPosition = Controller.extractPosition(currentInputs);
        //check if the previous inputs were null (meaning we're in init)
        if(previousInputs == null){
            previousInputs = currentInputs;
        }
        Vector previousPosition = Controller.extractPosition(previousInputs);


        //get the delta time
        float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);

        //use the state of the drone to extract the flight path if needed
        FlightPath flightPath = null;
        if(currentState == FLIGHT){
            //get the flight controller
            AirportNavigationController navigationController = this.getAirportNavigationController();
            flightPath = navigationController.getFlightPath();
        }

        //get the cruising altitude form the communicator
        float cruisingAltitude = autopilot.getCommunicator().getAssignedCruiseAltitude();

        //get the flight path
        FlightPath anonymousFlightPath = flightPath;
        //get the current delivery
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        AutopilotDelivery currentDelivery = communicator.getCurrentRequest();
        //now generate the info to be sent to the overseer
        AutopilotInfo info = new AutopilotInfo() {
            @Override
            public String droneID() {
                return droneID;
            }

            @Override
            public Vector getCurrentPosition() {
                return currentPosition;
            }

            @Override
            public Vector getPreviousPosition() {
                return previousPosition;
            }

            @Override
            public float getDeltaTime() {
                return deltaTime;
            }

            @Override
            public float getCruisingAltitude() {
                return cruisingAltitude;
            }

            @Override
            public AutopilotState getAutopilotState() {
                return currentState;
            }

            @Override
            public FlightPath getFlightPath() {
                return anonymousFlightPath;
            }
            
            @Override
            public boolean isIdle() {
            	return currentDelivery == null;
            }
        };

        this.setAutopilotInfo(info);
    }

    /**
     * Getter for the state that is the next in the row for the finite state machine, does only provide
     * the state that follows the provided state. Does not determine what the next state should be
     * --> only for reference
     * @param state the state to find the successor for
     * @return the state that succeeds the provided state
     */
    private AutopilotState successorState(AutopilotState state){
        switch(state){
            case INIT_FLIGHT:
                return TAKEOFF;
            case TAKEOFF:
                return STABILIZE_TAKEOFF;
            case STABILIZE_TAKEOFF:
                return FLIGHT;
            case FLIGHT:
                return DESCEND_WAIT;
            case DESCEND_WAIT:
                return DESCEND;
            case DESCEND:
                return LANDING;
            case LANDING:
                return TAXIING_TO_GATE;
            case TAXIING_TO_GATE:
                return TAXIING_TO_RUNWAY;
            case TAXIING_TO_RUNWAY:
                return TAKEOFF;
            default:
                return TAKEOFF;
        }
    }

    /**
     * Get the controller responsible for the provided state
     * @param state the state to get the controller for
     * @return the controller assiciated with the state
     */
    private Controller getStateController(AutopilotState state){
        switch(state){
            case INIT_FLIGHT:
                return this.getInitController();
            case TAKEOFF:
                return this.getTakeoffController();
            case STABILIZE_TAKEOFF:
                return this.getTakeoffStabilizerController();
            case FLIGHT:
                return this.getAirportNavigationController();
            case DESCEND_WAIT:
                return this.getDescendWaitController();
            case DESCEND:
                return this.getDescendController();
            case LANDING:
                return this.getLandingController();
            case TAXIING_TO_GATE:
                return this.getGateTaxiingController();
            case TAXIING_TO_RUNWAY:
                return this.getRunwayTaxiingController();
            default:
                return this.getInitController();
        }
    }


    /**
     * Getter for the initializer controller, this is a dummy controller used to wait one
     * iteration to get the previous state of the world (previous inputs) such that the other controllers
     * can function correctly
     * @return a controller used for initializing the finite state machine
     */
    private Controller getInitController() {
        return initController;
    }

    /**
     * Getter for the takeoff controller, the controller that is responsible for guiding
     * the drone's takeoff
     * @return the controller used to guide the takeoff
     */
    private AutopilotTakeoffController getTakeoffController() {
        return takeoffController;
    }

    /**
     * Getter for the takeoff stabilizer controller, this controller is responsible for stabilizing the drone
     * after takeoff
     * @return the controller used to guide the stabilization of the drone after takeoff
     */
    private AutopilotStabilization getTakeoffStabilizerController() {
        return takeoffStabilizerController;
    }

    /**
     * Getter for the flight controller, the controller that is responsible for guiding the
     * drone's flight to the next airport
     * @return the controller to guide the flight
     */
    private AirportNavigationController getAirportNavigationController() {
        return airportNavigationController;
    }

    /**
     * Getter for the descend wait controller, this is the controller used when the drone has to wait for descend
     * permission, if the permission is not granted, the drone has to wait until it has it (thus circling above the airport)
     * @return the descend wait controller, responsible for waiting and reserving the airport for the drone
     */
    private DescendWaitController getDescendWaitController(){
        return descendWaitController;
    }

    /**
     * Getter for the descend controller of the drone, used to descend to a specified value (usually 75m)
     * for a better landing. This is achieved by making a turn & lowering the altitude of the drone
     * @return the descend controller
     */
    private DescendController getDescendController(){
        return descendController;
    }

    /**
     * Getter for the landing controller, the controller that is responsible for guiding the
     * drone's landing on the target airport
     * @return the controller to guide the landing of the drone
     */
    private AutopilotLandingController getLandingController() {
        return landingController;
    }

    /**
     * Getter for the gate taxiing controller, this is the controller used to drive the drone to the destination gate
     * @return the gate controller used to taxi to the gate with
     */
    private GateTaxiingController getGateTaxiingController() {
        return gateTaxiingController;
    }

    /**
     * Getter for the taxiing controller responsible for taxiing to the runway for takeoff
     * @return the runway taxiing controller
     */
    private RunwayTaxiingController getRunwayTaxiingController() {
        return runwayTaxiingController;
    }

    /**
     * Getter for the autopilot that is represented by the finite state machine
     * @return the autopilot the finite state machine is working for
     */
    private AutoPilot getAutopilot() {
        return autopilot;
    }

    /**
     * Getter for the current state of the machine
     * @return the current state of the autopilot
     */
    private AutopilotState getState() {
        return state;
    }

    /**
     * Getter for the previously received inputs from the testbed
     * @return an autopilot inputs object previously received by the autopilot
     */
    private AutopilotInputs_v2 getPreviousInputs(){
        return this.previousInputs;
    }

    /**
     * Updates the previous outputs that are saved by the finite state machine (required to call a controller)
     * @param inputs the inputs used to update the previous inputs
     * note: the previous outputs are replaced by the currently provided inputs (name is purely for semantics)
     */
    private void updatePreviousOutputs(AutopilotInputs_v2 inputs){
        this.previousInputs = inputs;
    }

    /**
     * Setter for the state of the machine
     * @param state the state to assign the autopilot
     */
    private void setState(AutopilotState state) {
        this.state = state;
    }

    /**
     * Getter for the autopilot info, this object contains all the information about the autopilot
     * to do collision detection for the drone
     * @return the autopilot info containing all the necessary data about the autopilot
     */
    public AutopilotInfo getAutopilotInfo() {
        return autopilotInfo;
    }

    /**
     * Setter for the autopilot info
     * this method must be called after every generation of the outputs
     * such that the communicator can send the data about the autopilots to the overseer
     * @param autopilotInfo the info to be set, this is the info generated at the current state
     */
    private void setAutopilotInfo(AutopilotInfo autopilotInfo) {
        this.autopilotInfo = autopilotInfo;
    }

    /**
     * Getter for the standard landing altitude this is the altitude to which the drone drops before actually initiating
     * the landing if the landing descend threshold is breached
     * @return the standard landing altitude in meters
     */
    private float getStandardLandingAltitude() {
        return standardLandingAltitude;
    }

    /**
     * Getter for the landing descend threshold, this is the cruising altitude wherefore the descend phase is triggered
     * before initiating the landing, this is done to gain a favorable position for the landing
     * @return the descend threshold in meters
     */
    private float getLandingDescendThreshold() {
        return landingDescendThreshold;
    }

    /**
     * The state of the finite state machine, is initialized on takeoff
     */
    private AutopilotState state = AutopilotState.INIT_FLIGHT;

    /**
     * The autopilot wherefore the finite state machine is currenly active
     */
    private AutoPilot autopilot;

    /**
     * The inputs previously received by the finite state machine
     * these are used to initiate the controllers used by the autopilot after another one was used
     * or when they are used for the first time
     */
    private AutopilotInputs_v2 previousInputs = null;

    /**
     * The information generated for the autopilot to send to the overseer
     * this variable must be set & generated on every iteration --> will be read by the overseer communicator
     * & the info will be sent to the overseer
     */
    private AutopilotInfo autopilotInfo = AutopilotInfo.generateInitInfo();

    /*
    Some landing parameters needed for coordination between landing descend and navigation controller
     */

    /**
     * The standard altitude to start the landing of the drone for
     * this altitude is used when the drone is assigned a higher altitude than the landing descend
     * threshold
     */
    private final float standardLandingAltitude = 30f;

    /**
     * The threshold wherefore the descend phase will be activating previous to the actual landing
     * --> used to get a safe altitude to initiate the landing
     */
    private final float landingDescendThreshold = 35f;

    /**
     * The controllers used by the finite state machine to generate the control actions
     */
    private AutopilotTakeoffController takeoffController;
    private AutopilotStabilization takeoffStabilizerController;
    private AirportNavigationController airportNavigationController;
    private DescendWaitController descendWaitController;
    private DescendController descendController;
    private AutopilotLandingController landingController;
    private GateTaxiingController gateTaxiingController;
    private RunwayTaxiingController runwayTaxiingController;

    private Controller initController = new Controller(null) {
        @Override
        public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            //create a output that does absolutely nothing, because it is the first iteration
            return new AutopilotOutputs() {
                @Override
                public float getThrust() {
                    return 0;
                }

                @Override
                public float getLeftWingInclination() {
                    return 0;
                }

                @Override
                public float getRightWingInclination() {
                    return 0;
                }

                @Override
                public float getHorStabInclination() {
                    return 0;
                }

                @Override
                public float getVerStabInclination() {
                    return 0;
                }

                @Override
                public float getFrontBrakeForce() {
                    return 0;
                }

                @Override
                public float getLeftBrakeForce() {
                    return 0;
                }

                @Override
                public float getRightBrakeForce() {
                    return 0;
                }
            };
        }

        @Override
        public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            return AutopilotFiniteStateMachine.this.previousInputs != null;
        }

        @Override
        public void reset() {
            //literally does nothing for this case
        }
    };
}
