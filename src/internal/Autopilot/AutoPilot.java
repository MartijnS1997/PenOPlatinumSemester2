package internal.Autopilot;


import AutopilotInterfaces.*;
import AutopilotInterfaces.Path;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

//TODO: only recalculate after the new frame is rendered or make seperate controls for the case no new
//visual input was generated

/**
 * Created by Martijn on 14/10/2017.
 * Extended by Bart on 15/10/2017.
 * Extended by Anthony Rathé on 16/10/2017 and later
 */
public class AutoPilot implements Autopilot_v2{


	/**
	 * Primary constructor for the AutopilotInterfaces
	 * @param overseer the autopilot overseer, used to manage package delivery and collision avoidance
	 */
	public AutoPilot(AutopilotOverseer overseer){
		//first get the controller selector
		//this.setSelector(new ControllerSelector(this));
		this.stateMachine = new AutopilotFiniteStateMachine(this);
		this.overseer = overseer;
		this.communicator = new AutopilotCommunicator(this, overseer);
		//may be out commented if the transitions are smooth
		//this.getSelector().forceActiveController(FlightState.TAXIING_TO_GATE);

	}

	public AutoPilot() {
		this(null);
	}

	@Override
	public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs_v2 inputs) {
		configureAutopilot(config, inputs);
		return getControlOutputs(inputs);
	}

	/**
	 * generates control outputs destined for the drone (see the controller implementations for details
	 * @param inputs the input data from the drone
	 * @return the commands for the drone
	 */
	@Override
	public AutopilotOutputs timePassed(AutopilotInputs_v2 inputs) {
		return  getControlOutputs(inputs);
	}

	/**
	 * sets the path that can be used for the autopilot
	 * @param path the path to be followed
	 */
	@Override
	public void setPath(Path path) {
		this.path = path;
	}

	/**
	 * Signals that the simulation has been started
	 */
	@Override
    public void simulationEnded() {

    }
    

    /**
     * configures the autopilot at the start of the simulation
     * @param configuration the configuration of the autopilot
     * @param inputs the inputs of the autopilot
     * @author Martijn Sauwens
     */
    public void configureAutopilot(AutopilotConfig configuration, AutopilotInputs_v2 inputs) {

    	//save the configuration:
		this.setConfig(configuration);
    	//initialize the Physics Engine
		this.setPhysXEngine(new PhysXEngine(configuration));
		//initialize the Physics Engine Optimisations
		this.setPhysXOptimisations(this.getPhysXEngine().createPhysXOptimisations());
		//initialize the controllers
		//this.getSelector().setConfig(configuration);


        //Initialize the autopilot camera
        byte[] inputImage = inputs.getImage();
        int nbRows = configuration.getNbRows();
        int nbColumns = configuration.getNbColumns();
        float horizViewAngle = configuration.getHorizontalAngleOfView();
        float verticViewAngle = configuration.getVerticalAngleOfView();

		//this.setAPCamera(new AutoPilotCamera(inputImage, horizViewAngle, verticViewAngle, nbRows, nbColumns));


        setStartPosition(new Vector(inputs.getX(), inputs.getY(), inputs.getZ()));

    }


	/**
	 * Generates the control outputs of the autpilot destined for the drone
	 * @param inputs the inputs of the drone containing information such as location, orientation etc.
	 * @return control outputs for the drone
	 */
    private AutopilotOutputs getControlOutputs(AutopilotInputs_v2 inputs){
		//set the current inputs so the communicator can access them
		this.setCurrentInputs(inputs);
		//get the overseer
		AutopilotCommunicator communicator = this.getCommunicator();
		communicator.communicateWithOverseer();
		//return this.getSelector().getControlActions(inputs);

		//get the outputs from the state machine
		AutopilotOutputs outputs = this.getStateMachine().getMachineOutput(inputs);
		return outputs;
	}


	/**
	 * @author anthonyrathe
	 */
	protected AutoPilotCamera getAPCamera() throws NullPointerException{
		if (this.APCamera == null){
			throw new NullPointerException("No APCamera was assigned to this AutoPilot");
		}
		return this.APCamera;
	}
	
	/**
	 * @author anthonyrathe
	 */
	private void setAPCamera(AutoPilotCamera newAPCamera){
		this.APCamera = newAPCamera;
	}
	

	/*
    Getters & Setters
     */

	/**
	 * Getter for the ID of the drone that the autopilot is currently simulating
	 * @return a string containing the ID
	 */
	public String getID(){
		return this.getConfig().getDroneID();
	}

	/**
	 * Getter for the controller selector
	 * the controller selector is responsible for the in flight transitions between the different
	 * controllers that were developed for the drone
	 * @return the controller selector responsible for selecting the correct controller
	 */
	private ControllerSelector getSelector() {
		return selector;
	}

	/**
	 * Setter for the selector of the drone (see getter for more info)
	 * @param selector the selector to be used by the autopilot
	 */
	private void setSelector(ControllerSelector selector) {
		if(!canHaveAsSelector(selector)){
			throw new IllegalStateException("Selector error, there is already a selector configured");
		}
		this.selector = selector;
	}

	private AutopilotTaxiingController getTaxiingController() { return taxiingController; }
	private void setTaxiingController(AutopilotTaxiingController taxiingController){
		this.taxiingController = taxiingController;
	}

	private boolean canHaveAsTaxiingController(AutopilotTaxiingController controller){

		return controller != null && controller.getAutopilot() == this && this.taxiingController == null;
	}


	/**
	 * Checks if the provided selector can be a selector for the autopilot
	 * @param selector the selector to be tested
	 * @return true if the selector is a non null reference and the current selector instance
	 * is uninitialized (null reference)
	 */
	private boolean canHaveAsSelector(ControllerSelector selector){
		return selector != null && this.getSelector() == null;
	}


	/**
	 * The physics engine configured for the autopilot (eg usage of physx optimisations for controller use)
	 * @return
	 */
	public PhysXEngine getPhysXEngine() {
		return physXEngine;
	}

	/**
	 * Setter for the physics engine configured for the autopilot
	 * @param physXEngine
	 */
	private void setPhysXEngine(PhysXEngine physXEngine) {
		this.physXEngine = physXEngine;
	}

	/**
	 * The physicsEngineOptimisations configured for the autopilot, helper methods are provided
	 * for the autopilot
	 * @return the physXOptimisations configured for the autopilot
	 */
	protected PhysXEngine.PhysXOptimisations getPhysXOptimisations() {
		return physXOptimisations;
	}

	/**
	 * Setter for the physicsEngineOptimisations configured for the autopilot, helper methods are provided
	 * @param physXOptimisations the optimisations object for the autopilot
	 */
	private void setPhysXOptimisations(PhysXEngine.PhysXOptimisations physXOptimisations) {
		this.physXOptimisations = physXOptimisations;
	}

	/**
	 * The configuration of the autopilot, containing all necessary data to configure an autopilot
	 * @return the configuration of the autopilot
	 */
	public AutopilotConfig getConfig() {
		return config;
	}

	/**
	 * Setter for the configuration of the autopilot
	 * @param config the configuration to set
	 */
	public void setConfig(AutopilotConfig config) {
		this.config = config;
	}


	/**
	 * Getter for the path approximation to be followed by the autopilot
	 * @return the path approximation of the autopilot
	 */
	public Path getPath() {
		return path;
	}

	/**
	 * Setter for the start position of the drone, this will be used by the controller selector to
	 * set a target for the taxiing controller (may be omitted later on)
	 * @return
	 */
	protected Vector getStartPosition() {
		return this.startPosition;
	}

	/**
	 * setter for the start position of the drone (see getter for more info)
	 * @param position the current starting position
	 *                 todo: add checker to see if the startpos isn't already configured
	 */
	private void setStartPosition(Vector position) {
		this.startPosition = position;
	}


	/**
	 * Getter for the current inputs of the autopilot (needed by the autopilot overseer communication)
	 * @return an AutopilotInputs_v2 object that contains the current inputs for the autopilot
	 */
	protected AutopilotInputs_v2 getCurrentInputs() {
		return currentInputs;
	}

	/**
	 * Setter for the current inputs of the autopilot (see getter for more info)
	 * @param currentInputs the current inputs provided by the testbed
	 */
	private void setCurrentInputs(AutopilotInputs_v2 currentInputs) {
		this.currentInputs = currentInputs;
	}

	/**
	 * Getter for the overseer communication entity, used to communicate with the overseer
	 * @return an OverseerCommunication object used to communicate with the overseer
	 */
	protected AutopilotCommunicator getCommunicator() {
		return communicator;
	}

	/**
	 * Getter for the overseer that governs all autopilots
	 * @return the overseer that regulates the autopilots
	 */
	private AutopilotOverseer getOverseer() {
		return overseer;
	}

	/**
	 * Getter for the finite state machine used by the autopilot for getting
	 * the control actions and controlling the sequence of controllers needed to guide the
	 * package delivery system
	 * @return the state machine used to guide the autopilot
	 */
	protected AutopilotFiniteStateMachine getStateMachine(){
		return this.stateMachine;
	}

	/**
	 * The controller selector for the AP, selects the currently active controller
	 * that is responsible for each flight state
	 */
	private ControllerSelector selector;


	private AutopilotTaxiingController taxiingController;

	/**
	 * Variable that stores the configuration of the autopilot
	 */
	private AutopilotConfig config;
	/**
	 * Variable that stores the autopilot camera
	 */
	private AutoPilotCamera APCamera;

	/**
	 * variable that stores the physics engine associated with the autopilot
	 */
	private PhysXEngine physXEngine;

	/**
	 * variable that stores the physic engine Optimisations
	 */
	private PhysXEngine.PhysXOptimisations physXOptimisations;

	/**
	 * Object that stores the current path to follow
	 */
	private Path path;

	/**
	 * Variable for storing the startPosition of the drone, which also serves as the destination position
	 */
	private Vector startPosition;

	/**
	 * The overseer used to coordinate all the autopilots
	 */
	private AutopilotOverseer overseer;

	/**
	 * The inputs currently received by the autopilot
	 */
	private AutopilotInputs_v2 currentInputs;

	/**
	 * The entity that regulates the communication with the overseer
	 */
	private AutopilotCommunicator communicator;

	/**
	 * The finite state machine used to govern the controllers for the autopilot
	 */
	private AutopilotFiniteStateMachine stateMachine;


    /*
    Error messages
     */
    public final static String INVALID_THRUST = "The supplied thrust is out of bounds";
	public final static String INVALID_CONTROLLER = "The flightController is already initialized";

}
/*
CODE GRAVEYARD
 */
//
//	private AutopilotOutputs getControlOutputs(AutopilotInputs_v2 inputs){
//
//		AutoPilotFlightController flightController = this.getFlightController();
//		AutopilotTakeoffController takeoffController = this.getTakeoffController();
//		AutopilotLandingController landingController = this.getLandingController();
//		AutopilotWayPointController wayPointController = this.getWayPointController();
//
//		switch (getAPMode()){
//			case TAKEOFF:
//				return takeoffController.getControlActions(inputs);
//			case FLYING_TO_BLOCKS:
//				return flightController.getControlActions(inputs);
//			case WAY_POINT:
//				return wayPointController.getControlActions(inputs);
//			case LANDING:
//				return landingController.getControlActions(inputs);
//			default:
//				throw new IllegalArgumentException("Invalid controller type");
//		}
//
//
//
//	}
//	/**
//	 * getter for the maximum thrust
//	 * @return the maximum thrust
//	 * @author Martijn Sauwens
//	 */
//	public float getMaxThrust() {
//		return this.getConfig().getMaxThrust();
//	}
//	/**
//	 * Setter for the takeoff flightController
//	 * @param controller the takeoff flightController of the autopilot
//	 */
//	private void setTakeoffController(AutopilotTakeoffController controller){
//		if(!canHaveAsTakeoffController(controller))
//			throw new IllegalArgumentException(INVALID_CONTROLLER);
//		this.takeoffController = controller;
//	}
//
//	/**
//	 * Checker if the given takeoff flightController is valid
//	 * @param controller the flightController responsible for the takeoff
//	 * @return true if the flightController is not a null reference and the flightController already references the autopilot
//	 * 		   and the takeoff flightController of the autopilot is uninitialized
//	 */
//	private boolean canHaveAsTakeoffController(AutopilotTakeoffController controller){
//		return controller != null && controller.getAutopilot() == this && this.takeoffController == null;
//	}
//
//	/**
//	 * Getter for the takeoff flightController of the autopilot
//	 * @return the takeoff flightController of the autopilot
//	 */
//	public AutopilotTakeoffController getTakeoffController() {
//		return takeoffController;
//	}
//
//	/**
//	 * Getter for the autopilot flightController
//	 * @return the flightController of the autopilot
//	 */
//	private AutoPilotFlightController getFlightController() {
//		return flightController;
//	}
//
//	/**
//	 * setter for the autopilotController other part of the bidirectional relationship
//	 * @param controller the desired flightController
//	 */
//	private void setFlightController(AutoPilotFlightController controller) {
//		if(!this.canHaveAsFlightController(controller))
//			throw new IllegalArgumentException(INVALID_CONTROLLER);
//		this.flightController = controller;
//	}
//
//	private boolean canHaveAsFlightController(AutoPilotFlightController controller){
//
//		return controller != null && controller.getAutopilot() == this && this.flightController == null;
//	}
//
//	/**
//	 * Setter for the landing flightController of the drone
//	 * @param controller the landing flightController for constraints @see canHaveAsLandingController
//	 */
//	private void setLandingController(AutopilotLandingController controller){
//		if(!canHaveAsLandingController(controller))
//			throw new IllegalArgumentException(INVALID_CONTROLLER);
//		this.landingController = controller;
//	}
//
//	/**
//	 * Checker if the given flightController is valid
//	 * @param controller the flightController to be checked
//	 * @return the flightController may not be a null reference, must already reference the autopilot as its
//	 * 		   designated autopilot and the landingController of the autopilot must be uninitialized
//	 */
//	private boolean canHaveAsLandingController(AutopilotLandingController controller){
//		return controller != null && controller.getAutopilot() == this && this.landingController == null;
//	}
//
//	/**
//	 * Getter for the landingController
//	 * @return the landing flightController of the autopilot
//	 */
//	public AutopilotLandingController getLandingController() {
//		return landingController;
//	}
//
//
//	private AutopilotWayPointController getWayPointController() {
//		return wayPointController;
//	}
//
//	private void setWayPointController(AutopilotWayPointController wayPointController) {
//		this.wayPointController = wayPointController;
//	}
//	/**
//	 * Object that stores the autopilot flight flightController
//	 */
//	private AutoPilotFlightController flightController;
//
//	/**
//	 * Object that stores the autopilot landing flightController
//	 */
//	private AutopilotLandingController landingController;
//
//	/**
//	 * Object that stores the autopilot takeoffController
//	 */
//	private AutopilotTakeoffController takeoffController;
//
//	/**
//	 * Object that stores the way point controller
//	 * for the return phase for the flight
//	 */
//	private AutopilotWayPointController wayPointController;
//	/**
//	 * The mode the autopilot is currently executing eg takeoff landing or flying
//	 * @return the current mode
//	 */
//	public APModes getAPMode() {
//		return this.APMode;
//	}
