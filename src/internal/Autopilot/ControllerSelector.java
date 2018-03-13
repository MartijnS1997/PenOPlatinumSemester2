package internal.Autopilot;

import AutopilotInterfaces.Autopilot;
import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;

/**
 * Created by Martijn on 13/03/2018.
 * A class of controller selectors
 * selects the controller that is currently used by the autopilot
 */
public class ControllerSelector {

    /**
     * Constructor for a controller selector, initializes all the needed controllers
     * @param autopilot the autopilot used for the drone
     */
    ControllerSelector(AutoPilot autopilot){
        this.autopilot = autopilot;
        //initialize all the needed controllers
        this.takeoffController = new AutopilotTakeoffController(autopilot);
        this.flightController =  new GammaFlightController(autopilot);
        this.wayPointController = new AutopilotWayPointController(autopilot);
        this.landingController = new AutopilotLandingController(autopilot);
        //the first controller we'll use is the takeoff controller
        //set the active controller to null reference so we can configure the controller if needed
        this.activeController = null;
        this.followUpController = this.getTakeoffController();
    }

    /**
     * Gets the control actions for the currently active controller
     * @param inputs the current inputs of the autopilot
     * @return the control actions generated by the currently active controller
     */
    AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs){
        Controller activeController = this.getActiveController();
        Controller nextActiveController = this.getFollowUpController();

        //check if the controller is finished doing its job
        if(activeController == null||activeController.hasReachedObjective(inputs)){
            //get the next controller
            activeController = nextActiveController;
            nextActiveController = this.getNextController(activeController);
            //configure the active controller if needed  controller if needed
            this.configureController(activeController);

            this.setActiveController(activeController);
            this.setFollowUpController(nextActiveController);
            //note: not checking if the second controller is also ready prevents issues with init
            //the next next controller will then be selected on the next invocation of the method
        }

        //set the current input for the next controller
        try {
            nextActiveController.setCurrentInputs(inputs);
        }catch( NullPointerException e){
            //if there is no next active controller it will be set to null, so ignore it
        }
        //then get the outputs
        return activeController.getControlActions(inputs);
    }

    /**
     * Sets the configuration of the autopilot for each controller (for easy reference and less layers of indirection)
     * @param config the configuration that is assigned to the controllers
     */
    void setConfig(AutopilotConfig config){
        this.getTakeoffController().setConfig(config);
        this.getFlightController().setConfig(config);
        this.getWayPointController().setConfig(config);
        this.getLandingController().setConfig(config);
    }


    /**
     * Configures the provided controller by invoking its corresponding configuring method
     * @param controller the controller to be configured
     */
    private void configureController(Controller controller){
        if(controller instanceof AutopilotTakeoffController){
            this.configureTakeoffController((AutopilotTakeoffController) controller);
            return;
        }
        if(controller instanceof AutoPilotFlightController){
            this.configureFlightController((AutoPilotFlightController) controller);
            return;
        }
        if(controller instanceof  AutopilotWayPointController) {
            this.configureWayPointController((AutopilotWayPointController) controller);
            return;
        }
        if(controller instanceof AutopilotLandingController){
            //no next controller is needed
            return;
        }else{
            return;
        }
    }

    /**
     * Configures the takeoff controller based on the path approx of the cubes
     * @param takeoffController the takeoff controller to be configured
     */
    private void configureTakeoffController(AutopilotTakeoffController takeoffController) {
        AutoPilot autopilot = this.getAutopilot();
        AutopilotInterfaces.Path path = autopilot.getPath();
        Vector firstCubePos = new Vector(path.getX()[0], path.getY()[0], path.getZ()[0]);
        float desiredHeight = firstCubePos.getyValue();
        takeoffController.setPIDReferenceAltitude(desiredHeight);
    }

    /**
     * Configures the way point controller (needed the before the landing stage, we need to set
     * a landing trajectory)
     * @param wayPointController the WayPointController to be configured
     */
    private void configureWayPointController(AutopilotWayPointController wayPointController){
        Vector startPos = this.getAutopilot().getStartPosition();
        wayPointController.setLandingPath(startPos);
    }

    /**
     * Configures the flight controller by setting the approximate path
     * @param flightController the flight controller to be configured
     */
    private void configureFlightController(AutoPilotFlightController flightController){
        AutopilotInterfaces.Path path = this.getAutopilot().getPath();
        flightController.setFlightPath(path);
    }
    /**
     * Gets the next controller in line to be used by the autopilot
     * @param activeController the currently active controller
     * @return the next controller that will be used
     */
    private Controller getNextController(Controller activeController){
        //create the switch of instance of's
        if(activeController instanceof AutopilotTakeoffController){
            return this.getFlightController();
        }
        if(activeController instanceof AutoPilotFlightController){
            return this.getWayPointController();
        }
        if(activeController instanceof  AutopilotWayPointController) {
            return this.getLandingController();
        }
        if(activeController instanceof AutopilotLandingController){
            //no next controller is needed
            return null;
        }else{
            return null;
        }
    }

    /**
     * Disrupts the normal flight sequence by setting the active controller to a different state
     * this method's only purpose is for controller behavior testing
     * @param flightState the flight state the controller needs to be brought in
     */
    void forceActiveController(FlightState flightState){
        switch(flightState){
            case TAKEOFF:
                this.setActiveController(getTakeoffController());
                this.setFollowUpController(getFlightController());
                return;
            case FLIGHT:
                this.setActiveController(getFlightController());
                this.setFollowUpController(getWayPointController());
                return;
            case WAY_POINT:
                this.setActiveController(getWayPointController());
                this.setFollowUpController(getLandingController());
                return;
            case LANDING:
                this.setActiveController(getLandingController());
                this.setFollowUpController(null);
                return;
            default:
                //nothing to do here
                return;
        }
    }

    /**
     * Setter for the currently active controller, may be accessed publicly for testing purposes only
     * @param activeController the active controller
     */
    private void setActiveController(Controller activeController) {
        this.activeController = activeController;
    }

    /**
     * Getter for the active controller, the active controller is the controller currently used by the
     * autopilot
     * @return the currently used controller
     */
    private Controller getActiveController() {
        return activeController;
    }

    /**
     * Getter for the next active controller, the controller that will be used next in the simulation
     * @return the next controller
     * note: this method exists because we want to update the state of the next controller for smooth transition
     */
    private Controller getFollowUpController() {
        return followUpController;
    }

    /**
     * Setter for the next active controller
     * @param followUpController the controller that will be active next
     */
    private void setFollowUpController(Controller followUpController) {
        this.followUpController = followUpController;
    }

    /**
     * Getter for the takeoff controller that steers the takeoff of the drone
     * @return the takeoff controller
     */
    private AutopilotTakeoffController getTakeoffController() {
        return takeoffController;
    }

    /**
     * Getter for the flight controller, the controller that controls the flight between the cubes themselves
     * @return the flight controller used for navigating to the cubes
     */
    private AutoPilotFlightController getFlightController() {
        return flightController;
    }

    /**
     * Getter for the way point controller, this controller follows a predefined path
     * is used to navigate back to the starting point
     * @return the waypoint controller
     */
    private AutopilotWayPointController getWayPointController() {
        return wayPointController;
    }

    /**
     * Getter for the landing controller, the controller that is responsible for landing the drone
     * and bringing it to a standstill
     * @return the controller responsible for the landing of the drone
     */
    private AutopilotLandingController getLandingController() {
        return landingController;
    }

    /**
     * Getter for the autopilot that is configured with the controller selector
     * @return an autopilot
     */
    private AutoPilot getAutopilot() {
        return autopilot;
    }

    /**
     * The controller that is currently used to generate the output of the drone
     */
    private Controller activeController;

    /**
     * The controller that is next in line to be used
     */
    private Controller followUpController;

    /**
     * the takeoff controller to be used by the autopilot
     */
    private AutopilotTakeoffController takeoffController;

    /**
     * the flight controller used by the autopilot
     */
    private AutoPilotFlightController flightController;

    /**
     * The way point controller used by the autopilot for following way points
     */
    private AutopilotWayPointController wayPointController;

    /**
     * The landing controller used by the autopilot for the landing
     */
    private AutopilotLandingController landingController;

    /**
     * The autopilot connected with the controller selector
     */
    private AutoPilot autopilot;
}
