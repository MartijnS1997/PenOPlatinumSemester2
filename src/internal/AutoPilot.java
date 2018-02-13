package internal;


import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import Autopilot.*;

//TODO: only recalculate after the new frame is rendered or make seperate controls for the case no new
//visual input was generated

/**
 * Created by Martijn on 14/10/2017.
 * Extended by Bart on 15/10/2017.
 * Extended by Anthony Rathé on 16/10/2017 and later
 */
public class AutoPilot implements Autopilot {

	public AutoPilot(String controllerConfig){
		switch(controllerConfig){
			case PhysXEngine.ALPHA_MODE:
				this.setController(new AlphaController(this));
				return;
			case PhysXEngine.BETA_MODE:
				this.setController(new BetaController(this));
		}
	}

    public AutoPilot() {

    	// set the controller of the autopilot
		//Todo uncomment when normal controller works again
    	this(PhysXEngine.ALPHA_MODE);
    	//this.attackController = new AutoPilotControllerNoAttack(this);

    }

    @Override
    public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) throws IOException {
            configureAutopilot(config, inputs);
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


        //Initialize the autopilot camera
        byte[] inputImage = inputs.getImage();
        int nbRows = configuration.getNbRows();
        int nbColumns = configuration.getNbColumns();
        float horizViewAngle = configuration.getHorizontalAngleOfView();
        float verticViewAngle = configuration.getVerticalAngleOfView();
        this.setAPCamera(new AutoPilotCamera(inputImage, horizViewAngle, verticViewAngle, nbRows, nbColumns));


    }



    private AutopilotOutputs getControlOutputs(AutopilotInputs inputs){
    	AutoPilotController controller = this.getController();
    	controller.setCurrentInputs(inputs);
		//AutoPilotControllerNoAttack controller = this.attackController;
		//attackController.setCurrentInputs(inputs);
    	return controller.getControlActions();
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
	public void setAPCamera(AutoPilotCamera newAPCamera){
		this.APCamera = newAPCamera;
	}
	

	/*
    Getters & Setters
     */

	/**
	 * Getter for the autopilot controller
	 * @return the controller of the autopilot
	 */
	public AutoPilotController getController() {
		return controller;
	}

	/**
	 * setter for the autopilotController other part of the bidirectional relationship
	 * @param controller the desired controller
	 */
	public void setController(AutoPilotController controller) {
		if(!this.canHaveAsController(controller))
			throw new IllegalArgumentException(INVALID_CONTROLLER);
		this.controller = controller;
	}

	public boolean canHaveAsController(AutoPilotController controller){

		return controller.getAssociatedAutopilot() == this && this.controller == null;
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
		this.getController().setFlightRecorder(flightRecorder);
		//this.attackController.setFlightRecorder(flightRecorder);
	}

	public PhysXEngine getPhysXEngine() {
		return physXEngine;
	}

	public void setPhysXEngine(PhysXEngine physXEngine) {
		this.physXEngine = physXEngine;
	}

	public PhysXEngine.PhysXOptimisations getPhysXOptimisations() {
		return physXOptimisations;
	}

	public void setPhysXOptimisations(PhysXEngine.PhysXOptimisations physXOptimisations) {
		this.physXOptimisations = physXOptimisations;
	}

	public AutopilotConfig getConfig() {
		return config;
	}

	public void setConfig(AutopilotConfig config) {
		this.config = config;
	}

	/**
	 * Object that stores the autopilot controller
	 */
	private AutoPilotController controller;

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

	//------- Parameters -------
	private static final float STANDARD_INCLINATION = (float)Math.PI/8;
	private static final float SHARP_INCLINATION = (float)Math.PI/4;
	private static final float STABLE_INCLINATION = (float)Math.PI/12;
	private static final float THRESHOLD_ANGLE = (float)Math.PI/36;
	private static final float THRESHOLD_PIXELS = 5f;
	private static final float INCREASE_THRUST_ANGLE = (float)(Math.PI*0.025);
	private static final int STANDARD_CUBE_SIZE = 10;
	private static final float NODE_REACHED_DISTANCE = 4f;
	private static final float STANDARD_THRUST = 32.859283f;
	private static final float CUBE_LOCATION_DELTA_THRESHOLD = 0.5f;
	private static final String ALPHA_CONTROLLER = "ALPHA_CONTROLLER";
	private static final String BETA_CONTROLLER = "BETA_CONTROLLER";



    /*
    Error messages
     */
    public final static String INVALID_THRUST = "The supplied thrust is out of bounds";
	public final static String INVALID_CONTROLLER = "The controller is already initialized";

}

