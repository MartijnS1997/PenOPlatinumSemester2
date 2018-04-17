package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;

/**
 * Created by Martijn on 13/03/2018.
 * A class of controller selectors
 * selects the controller that is currently used by the autopilot
 */
//TODO change the finite state machine so it fits with the new assignment
//the state machine:  taxiing --> takeoff --> flight --> landing --> taxiing (to gate and to takeoff) --> ... (repeat)
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
        //this.flightController = new TestFlightController(autopilot);
        this.wayPointController = new AutopilotWayPointController(autopilot);
        this.landingController = new AutopilotLandingController(autopilot);
        this.taxiingController = new AutopilotTaxiingController(autopilot);
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
//        Controller activeController = this.getActiveController();
//        Controller nextActiveController = this.getFollowUpController();
//
//        //check if the controller is finished doing its job
//        if(activeController == null||activeController.hasReachedObjective(inputs)){
//            //get the next controller
//            activeController = nextActiveController;
//            nextActiveController = this.getNextController(activeController);
//            //configure the active controller if needed  controller if needed
//            this.configureController(activeController);
//
//            this.setActiveController(activeController);
//            this.setFollowUpController(nextActiveController);
//            //note: not checking if the second controller is also ready prevents issues with init
//            //the next next controller will then be selected on the next invocation of the method
//            System.out.println("Switched controller");
//            System.out.println("Currently active controller: " + this.getActiveController());
//            System.out.println("Next active Controller: " + this.getNextController(activeController));
//        }
//
//        //set the current input for the next controller
//        try {
//            nextActiveController.updateInputs(inputs);
//        }catch( NullPointerException e){
//            //if there is no next active controller it will be set to null, so ignore it
//        }
        //then get the outputs
        //return activeController.getControlActions(inputs);
        return new AutopilotOutputs() {
            @Override
            public float getThrust() {
                return 2000;
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

    /**
     * Sets the configuration of the autopilot for each controller (for easy reference and less layers of indirection)
     * @param config the configuration that is assigned to the controllers
     */
    void setConfig(AutopilotConfig config){
        this.getTakeoffController().setConfig(config);
        this.getFlightController().setConfig(config);
        this.getWayPointController().setConfig(config);
        this.getLandingController().setConfig(config);
        this.getTaxiingController().setConfig(config);
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
        if(controller instanceof AutopilotFlightController){
            this.configureFlightController((AutopilotFlightController) controller);
            return;
        }
        if(controller instanceof  AutopilotWayPointController) {
            this.configureWayPointController((AutopilotWayPointController) controller);
            return;
        }
        if(controller instanceof AutopilotLandingController){
            //no next controller is needed
            this.configureLandingController((AutopilotLandingController) controller);
            return;
        }

        if(controller instanceof AutopilotTaxiingController){
            //configure the taxiing controller
            configureTaxiingController((AutopilotTaxiingController) controller);
        }
        else{
            return;
        }
    }

    /**
     * Configures the takeoff controller based on the path approx of the cubes
     * @param takeoffController the takeoff controller to be configured
     */
    private void configureTakeoffController(AutopilotTakeoffController takeoffController) {
        //Get the autopilot communicator
        OverseerCommunication communicator = this.getAutopilot().getCommunicator();
        //get the assigned altitude
        float cruisingAlt = communicator.getAssignedCruiseAltitude();
        //TODO get the airport and the runway to check in which direction we need to take off
        //takeoffController.setTarget(new Vector(0,cruisingAlt, 0));
//        AutoPilot autopilot = this.getAutopilot();
//        AutopilotInterfaces.Path path = autopilot.getPath();
//        List<Vector> pathList = Controller.extractPath(path);
//        System.out.println("The path: " + pathList);
//        //take the (0,0,0) as a reference
//        Vector startPos = autopilot.getStartPosition();
//        Optional<Vector> closestPos = pathList.stream().reduce((closest, next)-> closest.distanceBetween(startPos) < next.distanceBetween(startPos) ? closest : next);
//
//        if(!closestPos.isPresent()){
//            takeoffController.setTarget(new Vector(0,30,-500));
//        }
//        Vector target = closestPos.get();
//        System.out.println("Selected Target: "+ target);
//        takeoffController.setTarget(target);
//        float desiredHeight = pathList.get(0).getyValue();
//        System.out.println("desired heigth " + desiredHeight);
//        System.out.println("total path: " + pathList);
        //takeoffController.setReferenceAltitude(desiredHeight);
    }



    /**
     * Configures the flight controller by setting the approximate path
     * @param flightController the flight controller to be configured
     */
    private void configureFlightController(AutopilotFlightController flightController){
        AutopilotInterfaces.Path path = this.getAutopilot().getPath();
        flightController.setFlightPath(path);
    }

    /**
     * Configures the way point controller (needed the before the landing stage, we need to set
     * a landing trajectory)
     * @param wayPointController the WayPointController to be configured
     */
    private void configureWayPointController(AutopilotWayPointController wayPointController){
        Vector startPos = this.getAutopilot().getStartPosition();
        wayPointController.setDestination(startPos);
    }

    private void configureLandingController(AutopilotLandingController landingController){
        //get the height of the final
    }

    private void configureTaxiingController(AutopilotTaxiingController taxiingController){
        //set the target for the controller
        Vector target = this.getAutopilot().getStartPosition();
        taxiingController.setTarget(target);
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
        if(activeController instanceof AutopilotFlightController){
            //return this.getWayPointController();
            return this.getLandingController();
        }
        if(activeController instanceof  AutopilotWayPointController) {
            return this.getLandingController();
        }
        if(activeController instanceof AutopilotLandingController){
            //no next controller is needed
            //generate a generic controller that goes full brakes
            return this.getTaxiingController();

        }if(activeController instanceof AutopilotTaxiingController){
            return  null;
//            new Controller(this.getAutopilot()) {
//                @Override
//                public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
//                    return new AutopilotOutputs() {
//                        @Override
//                        public float getThrust() {
//                            return 0;
//                        }
//
//                        @Override
//                        public float getLeftWingInclination() {
//                            return 0;
//                        }
//
//                        @Override
//                        public float getRightWingInclination() {
//                            return 0;
//                        }
//
//                        @Override
//                        public float getHorStabInclination() {
//                            return 0;
//                        }
//
//                        @Override
//                        public float getVerStabInclination() {
//                            return 0;
//                        }
//
//                        @Override
//                        public float getFrontBrakeForce() {
//                            return ControllerSelector.this.getAutopilot().getConfig().getRMax();
//                        }
//
//                        @Override
//                        public float getLeftBrakeForce() {
//                            return ControllerSelector.this.getAutopilot().getConfig().getRMax();
//                        }
//
//                        @Override
//                        public float getRightBrakeForce() {
//                            return ControllerSelector.this.getAutopilot().getConfig().getRMax();
//                        }
//                    };
//                }
//
//                @Override
//                public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
//                    return false;
//                }
//
//            };
        }
        else{
            return null;
        }
    }

    /**
     * Disrupts the normal flight sequence by setting the active controller to a different state
     * this method's only purpose is for controller behavior testing
     * @param flightState the flight state the controller needs to be brought in
     */
    void forceActiveController(AutopilotState flightState){
        switch(flightState){
            case TAKEOFF:
                this.setActiveController(getTakeoffController());
                this.setFollowUpController(getFlightController());
                return;
            case FLIGHT:
                this.setActiveController(getFlightController());
                this.setFollowUpController(getWayPointController());
                return;
//            case WAY_POINT:
//                this.setActiveController(getWayPointController());
//                this.setFollowUpController(getLandingController());
//                return;
            case LANDING:
                this.setActiveController(getLandingController());
                this.setFollowUpController(null);
                return;
            case TAXIING_TO_GATE:
                this.setActiveController(getTaxiingController());
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
    private AutopilotFlightController getFlightController() {
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
     * Getter for the taxiing controller, the controller that is responsible for taxiing the drone
     * and bringing it to a standstill
     * @return the controller responsible for the taxiing of the drone
     */
    private AutopilotTaxiingController getTaxiingController() {
        return taxiingController;
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
    private AutopilotFlightController flightController;

    /**
     * The way point controller used by the autopilot for following way points
     */
    private AutopilotWayPointController wayPointController;

    /**
     * The landing controller used by the autopilot for the landing
     */
    private AutopilotLandingController landingController;

    /**
     * The landing controller used by the autopilot for the landing
     */
    private AutopilotTaxiingController taxiingController;

    /**
     * The autopilot connected with the controller selector
     */
    private AutoPilot autopilot;

}
