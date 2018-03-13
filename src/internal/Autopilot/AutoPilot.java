package internal.Autopilot;


import java.io.IOException;

import Autopilot.*;
import internal.Testbed.FlightRecorder;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

//TODO: only recalculate after the new frame is rendered or make seperate controls for the case no new
//visual input was generated

/**
 * Created by Martijn on 14/10/2017.
 * Extended by Bart on 15/10/2017.
 * Extended by Anthony Rathé on 16/10/2017 and later
 */
public class AutoPilot implements Autopilot{


	/**
	 * Primary constructor for the Autopilot
	 * @param controllerConfig
	 */
	public AutoPilot(String controllerConfig){
		// first make sure we can takeoff
		this.setTakeoffController(new AutopilotTakeoffController(this));
		System.out.println(controllerConfig);
		// then we mus assure our customers a smooth ride
		switch(controllerConfig){
			case PhysXEngine.ALPHA_MODE:
				this.setFlightController(new AlphaFlightController(this));
				break;
			case PhysXEngine.BETA_MODE:
				this.setFlightController(new BetaFlightController(this));
				break;
			case PhysXEngine.GAMMA_MODE:
				this.setFlightController(new GammaFlightController(this));
		}
		// and last, we need to land
		this.setLandingController(new AutopilotLandingController(this));
		this.setWayPointController(new AutopilotWayPointController(this));
		//set AP mode 2 to make everything work again
		this.setAPMode(APModes.WAY_POINT); //AP 3 to test the landing controller

	}

	/**
	 * Default constructor for the autopilot
	 */
    public AutoPilot() {

    	// set the flightController of the autopilot
		//Todo uncomment when normal flightController works again
    	this(PhysXEngine.ALPHA_MODE);
    	//this.attackController = new AutoPilotControllerNoAttack(this);
    }

    @Override
    public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) throws IOException {
            configureAutopilot(config, inputs);
//            if (this.getPhysXEngine().chassisTouchesGround(new Vector(inputs.getX(),inputs.getY(),inputs.getZ()), new Vector(inputs.getHeading(),inputs.getPitch(),inputs.getRoll()))) {
//            	this.setAPMode(1);
//            }else {
//            	this.setAPMode(2);
//            }
//            this.startPosition = new Vector(inputs.getX(),inputs.getY(),inputs.getZ());
        return getControlOutputs(inputs);
    }


    @Override
    public AutopilotOutputs timePassed(AutopilotInputs inputs) throws IOException {
        return getControlOutputs(inputs);
    }


	@Override
    public void simulationEnded() {

    }
    

    /**
     * configures the autopilot at the start of the simulation
     * @param configuration the configuration of the autopilot
     * @param inputs the inputs of the autopilot
     * @author Martijn Sauwens
     */
    public void configureAutopilot(AutopilotConfig configuration, AutopilotInputs inputs) {

    	//save the configuration:
		this.setConfig(configuration);
    	//initialize the Physics Engine
		this.setPhysXEngine(new PhysXEngine(configuration));
		//initialize the Physics Engine Optimisations
		this.setPhysXOptimisations(this.getPhysXEngine().createPhysXOptimisations());
		//initialize the takeoff controller
		this.getTakeoffController().setConfig(configuration);
		//initialize the flight controller
		this.getFlightController().setConfig(configuration);
		//initialize the takeoff controller
		this.getTakeoffController().setConfig(configuration);


        //Initialize the autopilot camera
        byte[] inputImage = inputs.getImage();
        int nbRows = configuration.getNbRows();
        int nbColumns = configuration.getNbColumns();
        float horizViewAngle = configuration.getHorizontalAngleOfView();
        float verticViewAngle = configuration.getVerticalAngleOfView();
        this.setAPCamera(new AutoPilotCamera(inputImage, horizViewAngle, verticViewAngle, nbRows, nbColumns));


    }


	/**
	 * Generates the control outputs of the autpilot destined for the drone
	 * @param inputs the inputs of the drone containing information such as location, orientation etc.
	 * @return control outputs for the drone
	 */
	//Todo implement the 3 stages of the flight: takeoff, flight and landing
    private AutopilotOutputs getControlOutputs(AutopilotInputs inputs){
    	
    	AutoPilotFlightController flightController = this.getFlightController();
    	AutopilotTakeoffController takeoffController = this.getTakeoffController();
    	AutopilotLandingController landingController = this.getLandingController();
    	AutopilotWayPointController wayPointController = this.getWayPointController();

    	switch (getAPMode()){
			case TAKEOFF:
				return takeoffController.getControlActions(inputs);
			case FLYING_TO_BLOCKS:
				return flightController.getControlActions(inputs);
			case WAY_POINT:
				return wayPointController.getControlActions(inputs);
			case LANDING:
				return landingController.getControlActions(inputs);
			default:
				throw new IllegalArgumentException("Invalid controller type");
		}

	}


    /**
     * getter for the maximum thrust
     * @return the maximum thrust
     * @author Martijn Sauwens
     */
    public float getMaxThrust() {
        return this.getConfig().getMaxThrust();
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
	 * Setter for the takeoff flightController
	 * @param controller the takeoff flightController of the autopilot
	 */
	private void setTakeoffController(AutopilotTakeoffController controller){
		if(!canHaveAsTakeoffController(controller))
			throw new IllegalArgumentException(INVALID_CONTROLLER);
		this.takeoffController = controller;
	}

	/**
	 * Checker if the given takeoff flightController is valid
	 * @param controller the flightController responsible for the takeoff
	 * @return true if the flightController is not a null reference and the flightController already references the autopilot
	 * 		   and the takeoff flightController of the autopilot is uninitialized
	 */
	private boolean canHaveAsTakeoffController(AutopilotTakeoffController controller){
		return controller != null && controller.getAutopilot() == this && this.takeoffController == null;
	}

	/**
	 * Getter for the takeoff flightController of the autopilot
	 * @return the takeoff flightController of the autopilot
	 */
	public AutopilotTakeoffController getTakeoffController() {
		return takeoffController;
	}

	/**
	 * Getter for the autopilot flightController
	 * @return the flightController of the autopilot
	 */
	private AutoPilotFlightController getFlightController() {
		return flightController;
	}

	/**
	 * setter for the autopilotController other part of the bidirectional relationship
	 * @param controller the desired flightController
	 */
	private void setFlightController(AutoPilotFlightController controller) {
		if(!this.canHaveAsFlightController(controller))
			throw new IllegalArgumentException(INVALID_CONTROLLER);
		this.flightController = controller;
	}

	private boolean canHaveAsFlightController(AutoPilotFlightController controller){

		return controller != null && controller.getAutopilot() == this && this.flightController == null;
	}

	/**
	 * Setter for the landing flightController of the drone
	 * @param controller the landing flightController for constraints @see canHaveAsLandingController
	 */
	private void setLandingController(AutopilotLandingController controller){
		if(!canHaveAsLandingController(controller))
			throw new IllegalArgumentException(INVALID_CONTROLLER);
		this.landingController = controller;
	}

	/**
	 * Checker if the given flightController is valid
	 * @param controller the flightController to be checked
	 * @return the flightController may not be a null reference, must already reference the autopilot as its
	 * 		   designated autopilot and the landingController of the autopilot must be uninitialized
	 */
	private boolean canHaveAsLandingController(AutopilotLandingController controller){
		return controller != null && controller.getAutopilot() == this && this.landingController == null;
	}

	/**
	 * Getter for the landingController
	 * @return the landing flightController of the autopilot
	 */
	public AutopilotLandingController getLandingController() {
		return landingController;
	}


	public AutopilotWayPointController getWayPointController() {
		return wayPointController;
	}

	public void setWayPointController(AutopilotWayPointController wayPointController) {
		this.wayPointController = wayPointController;
	}

	/**
	 * Getter for the main wing mass of the drone
	 * @return a floating point number containing the mass of the main wing
	 */
	public float getMainWingMass() {
		return this.getConfig().getWingMass();
	}


	/**
	 * Getter for the mass of the stabilizer
	 * @return floating point number containing the stabilizer mass
	 */
	public float getStabilizerMass() {
		return this.getConfig().getTailMass();
	}


	/**
	 * Getter for the mass of the engine
	 * @return floating point containing the mass of the enige
	 */
	public float getEngineMass() {
		return this.getConfig().getEngineMass();
	}

	/**
	 * Setter for the flight recorder
	 */
	public void setFlightRecorder(FlightRecorder flightRecorder){
		this.getFlightController().setFlightRecorder(flightRecorder);
		//this.attackController.setFlightRecorder(flightRecorder);
	}

	public PhysXEngine getPhysXEngine() {
		return physXEngine;
	}

	private void setPhysXEngine(PhysXEngine physXEngine) {
		this.physXEngine = physXEngine;
	}

	protected PhysXEngine.PhysXOptimisations getPhysXOptimisations() {
		return physXOptimisations;
	}

	private void setPhysXOptimisations(PhysXEngine.PhysXOptimisations physXOptimisations) {
		this.physXOptimisations = physXOptimisations;
	}

	public AutopilotConfig getConfig() {
		return config;
	}

	public void setConfig(AutopilotConfig config) {
		this.config = config;
	}
	
	public APModes getAPMode() {
		return this.APMode;
	}
	
	protected void setAPMode(APModes newAPMode) {
		this.APMode = newAPMode;
	}

	/**
	 * Object that stores the autopilot flight flightController
	 */
	private AutoPilotFlightController flightController;

	/**
	 * Object that stores the autopilot landing flightController
	 */
	private AutopilotLandingController landingController;

	/**
	 * Object that stores the autopilot takeoffController
	 */
	private AutopilotTakeoffController takeoffController;

	/**
	 * Object that stores the way point controller
	 * for the return phase for the flight
	 */
	private AutopilotWayPointController wayPointController;

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
	 * used for engine validation
	 */
	private AutoPilotControllerNoAttack attackController;
	
	/**
	 * store current AutoPilot mode:
	 * 1 = landing mode
	 * 2 = flight mode
	 * 3 = takeoff mode
	 */
	private APModes APMode;
	
	public Vector getStartPosition() {
		return this.startPosition;
	}
	
	// Variable for storing the startPosition of the drone, which also serves as the destination position
	private Vector startPosition;


    /*
    Error messages
     */
    public final static String INVALID_THRUST = "The supplied thrust is out of bounds";
	public final static String INVALID_CONTROLLER = "The flightController is already initialized";

}

enum APModes{
	TAKEOFF, FLYING_TO_BLOCKS, WAY_POINT, LANDING
}

