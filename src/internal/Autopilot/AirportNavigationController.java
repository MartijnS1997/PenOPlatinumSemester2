package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import javax.naming.ldap.Control;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import static internal.Physics.PhysXEngine.worldOnDrone;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

/**
 * Created by Martijn on 3/04/2018.
 * A class of controllers used to navigate between the airports present in the world
 * TODO implement this class for the final stage of the project
 * TODO use for the banking lift the newly found formula for the AOA (see mupad file)
 * TODO implement the methods to handle multiple turns in a row and to fly to the next one
 * matlab commands to plot the flight path:
 * M = dlmread('turnlog.txt',';')
 * plot3(M(:,1), M(:,2), M(:,3))
 * with the current folder the project folder of this project
 *
 */
public class AirportNavigationController extends AutopilotFlightController {


    public AirportNavigationController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //generate the outputs object to write the control actions to
        ControlOutputs outputs = new ControlOutputs(this.getStandardOutputs());
        //get the bank controls: note this controller will have two states in the future, first the banking controls
        //and second the navigation to the next turn entry point
        turnLog(currentInputs);
        bankControl(outputs, currentInputs, previousInputs);
        System.out.println("Navigator outputs: " + outputs);
        return outputs;
    }

    /**
     * Debugging method used to save data to a .txt file
     * @param currentInputs the outputs to write
     */
    private void turnLog(AutopilotInputs_v2 currentInputs){
        Vector position = extractPosition(currentInputs);
        String logstring = position.getxValue() + ";" + position.getyValue() + ";" + position.getzValue() + "\n";
        try {
            Files.write(Paths.get("turnLog.txt"), logstring.getBytes(), StandardOpenOption.APPEND);
        }catch (IOException e) {
            //exception handling left as an exercise for the reader
        }
    }


    private void bankControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        bankingRollControl(outputs, currentInputs, previousInputs);
        altitudeControl(outputs, currentInputs, previousInputs);
        angleOfAttackControl(getAoaErrorMargin(), outputs, currentInputs, previousInputs);
        thrustControl(outputs, currentInputs, previousInputs);

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
        PIDController thrustPID = this.getBankingThrustController();
        this.flightCruiseControl(outputs, currentInputs, previousInputs, thrustPID, referenceVelocity);
    }


    /**
     * Generates the control actions for maintaining the altitude of the drone
     * @param outputs the outputs to write the altitude controls to
     * @param currentInputs the inputs the most recent received by the autopilot from the testbed
     * @param previousInputs the inputs previously received by the drone
     */
    private void altitudeControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the assigned cruising altitude
        float cruisingAltitude = this.getCruisingAltitude();
        float currentAltitude = extractAltitude(currentInputs);

        //get the difference in altitude (the error in the altitude)
        float altitudeDiff = cruisingAltitude - currentAltitude;

        //get the pid and the other parameters needed (eg delta time)
        float deltaTime = getDeltaTime(currentInputs, previousInputs);
        PIDController altitudeController  = this.getBankingAltitudeController();
        float pidOutput = altitudeController.getPIDOutput(altitudeDiff, deltaTime);

        //now adjust for the error that was made
        float desiredHorizontal = pidOutput + getHorizontalStable();
        float horizontalInclination = capInclination(desiredHorizontal,getHorizontalStable(), getHorizontalDeltaIncl());

        outputs.setHorStabInclination(horizontalInclination);

    }

    /**
     * A controller designed to handle the flight in between two turns
     * @param outputs the outputs to write the control actions to
     * @param currentInputs the latest inputs received from the testbed
     * @param previousInputs the inputs previously received from the
     */
    private void navigateToNextTurnControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){

    }

    /**
     * Generates the control actions to keep the roll in check during a banking turn with turning radius
     * ||dronePos - turnCenter|| and with center turnCenter
     * @param outputs the outputs to write the calculated results to
     * @param currentInputs the latest inputs received by the autopilot from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void bankingRollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        //calculate the angle needed to make the turn (may be done only once in the future)
        //get the parameters first:

        //call the turn control to check if the current turn is finished and if so, to configure the controller
        //for the next turn
        this.turnStateControl(currentInputs, previousInputs);
        //calculate the roll needed to make the turn
        float bankingRoll = this.getCurrentTurnRollReference();
        //correct the banking roll for the error on the turn radius
        float correctedBanking = correctReferenceRoll(bankingRoll, currentInputs);
        //get the PID and the parameters needed to get the errorOutput
        float roll = extractRoll(currentInputs);
        float rollError = correctedBanking - roll; // the error on the roll of the drone
        float deltaTime = getDeltaTime(currentInputs, previousInputs);

        System.out.println("banking roll needed: " + bankingRoll + ", actual roll: " + roll + ", error: " + rollError);

        PIDController rollController = this.getBankingRollController();
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

    private float correctReferenceRoll(float referenceRoll, AutopilotInputs_v2 currentInputs){
        //get the current turn
        ControlTurn CtrlTurn = this.getCurrentControlTurn();
        AutopilotTurn currentTurn = CtrlTurn.getTurn();
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
     * Checks if the current turn has finished and if needed to call the path generator to get the next turn
     * if there is a new turn to make, the controller is configured for the next turn
     */
    private void turnStateControl(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){

        PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();

        //first check if the turn physics are configured
        if(turnPhysX == null){
            //if not configure the turn physics
            turnPhysX = this.getAutopilot().getPhysXEngine().createTurnPhysics();
            //save the newly created physics
            this.setTurnPhysX(turnPhysX);
        }

        ControlTurn turn = this.getCurrentControlTurn();

        //check first of all if there is a turn set (init)
        if(turn == null){
            //set the turn:
            //TODO make call to the path generator & delete the dummy
            //dummy configuration to test the turning controller
            //create a circle 1000m to the right of the drone
            //1000m on the x-axis in the heading axis system
            float turnRadius = 1000;
            Vector xHeading = new Vector(turnRadius, 0,0);
            Vector currentGroundPos = Controller.extractGroundPosition(currentInputs);
            Vector orientation = Controller.extractOrientation(currentInputs);
            Vector xWorld = PhysXEngine.headingOnWorld(xHeading, orientation);
            //sum them both
            Vector turnCenter = currentGroundPos.vectorSum(xWorld);
            Vector entryPoint = currentGroundPos;
            AutopilotTurn currentTurn = new AutopilotTurn() {
                @Override
                public Vector getTurnCenter() {
                    return turnCenter;
                }

                @Override
                public float getTurnRadius() {
                    return turnRadius;
                }

                @Override
                public float getTurnAngle() {
                    //for now we'll make a 360° turn around the turn center left
                    return (float) (-2*PI);
                }
                @Override
                public Vector getEntryPoint(){
                    return entryPoint;
                }
            };
            System.out.println("Turn center: " + turnCenter +", drone position: " + currentGroundPos);

            //create the turn control needed to make the current turn
            ControlTurn newTurn = new ControlTurn(currentTurn, turnPhysX);
            //set the current turn, it will configure all we need
            this.setCurrentControlTurn(newTurn);
            //set the turn to prevent null pointer for the check
            turn = newTurn;
        }
        //TODO implement the connection with the path generator
        //check if the current turn has finished
        if(turn.hasFinishedTurn(currentInputs, previousInputs)){
            //get next turn
            //for the moment, do nothing, just print something
            System.out.println("Turn finished");
        }
    }

    /*
    Controller instances
     */

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
     * Getter for the reference velocity used by the cruise control. This velocity should be maintained
     * as well as possible
     * @return the reference velocity in m/s (scalar value of the velocity --> total)
     */
    //TODO dynamically pick the turn reference velocity and the "normal flight" reference velocity
    private float getReferenceVelocity() {
        ControlTurn currentTurn = this.getCurrentControlTurn();
        return currentTurn.getTurnVelocity();
    }

    /**
     * Getter for the control turn object(containing all the info needed to make the currently specified turn)
     * @return the control turn specified for the current turn
     */
    private ControlTurn getCurrentControlTurn() {
        return currentControlTurn;
    }

    /**
     * Setter for the current control turn, must be invoked once the drone is finished making its previous turn
     * @param currentControlTurn the next turn to make with the navigation controller
     */
    private void setCurrentControlTurn(ControlTurn currentControlTurn) {
        this.currentControlTurn = currentControlTurn;
    }

    /**
     * Getter for the roll reference, this is the roll that the drone has to make to take the
     * currently assigned turn
     * @return the roll needed to make the turn currently specified in currentTurn (in radians)
     */
    private float getCurrentTurnRollReference() {
        //get the turn controls for the currently assigned turn
        ControlTurn currentTurn = this.getCurrentControlTurn();
        return currentTurn.getBankingTurnRoll();
    }

    /**
     * Getter for the banking thrust PID controller, this PID controller needs to be passed
     * to the cruise control method implemented in the main controller class
     * @return a pid controller tuned to maintain the reference velocity of the autopilot
     */
    private PIDController getBankingThrustController() {
        return bankingThrustController;
    }

    /**
     * Getter for the roll controller, this controller is responsible for keeping the right rolling angle for
     * the drone to make a banking turn in the provided airspace
     * @return the PID controller tuned for reaching and maintaining the roll needed for banked turn
     */
    private PIDController getBankingRollController() {
        return bankingRollController;
    }

    /**
     * Getter for the altitude controller, this controller is responsible for maintaining the reference altitude
     * in this case the assigned cruising altitude by the autopilot overseer
     * @return the PID controller tuned for maintaining the assigned altitude during the banking turn
     */
    private PIDController getBankingAltitudeController() {
        return bankingAltitudeController;
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
     * Getter for the maximum horizontal delta inclinaton (same semantics as the main delta inclination)
     * the available range for the horizontal stabilizer is: [HORIZONTAL_STABLE -HOR_MAX, HORIZONTAL_STABLE + HOR_MAX]
     * @return the maximum deviation in the horizontal inclination from the stable value
     */
    private static float getHorizontalDeltaIncl() {
        return HORIZONTAL_DELTA_INCL;
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
     * The cruising altitude assigned to the drone by the overseer, this is the reference altitude for the
     * altitude PID controllers ( there are different controllers for different scenario's)
     */
    private float cruisingAltitude;


    /**
     * The control turn object used to manage all the turn related state and checks is used to
     * --> calculate and save the reference roll needed to make the turn
     * --> calculate and save the reference velocity needed to keep the drone level
     * --> check if the turn has finished
     * --> save the specifications of the turn we're currently making
     * --> even more in the future...
     */
    private ControlTurn currentControlTurn;


    /**
     * The turn PhysX object used by the autopilot to make turns
     * --> calculates banking angles and such
     */
    private PhysXEngine.TurnPhysX turnPhysX;


    /**
     * Controllers needed for banking the drone for a given radius
     * --> thrust controller: needed for the cruise control maintaining the velocity
     * --> roll controller: needed to keep the roll stable to maintain a good bank
     * --> altitude controller: needed to keep the drone at the same level (constant height)
     * note: the roll and the altitude controllers need to react slowly to changes (so the control actions
     * do not go in overdrive, maybe use low pass filter?)
     */
    private final static float THRUST_GAIN = 1.0f;
    private final static float THRUST_INTEGRAL = 0.1f ;
    private final static float THRUST_DERIVATIVE = 0.1f;
    private PIDController bankingThrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

    private final static float ROLL_GAIN = 0.02f;
    private final static float ROLL_INTEGRAL = 0.01f;
    private final static float ROLL_DERIVATIVE = 0f;
    private PIDController bankingRollController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    private final static float ALTITUDE_GAIN = 0.0008f;
    private final static float ALTITUDE_INTEGRAL = 0.00008f;
    private final static float ALTITUDE_DERIVATIVE = 0.006f;
    private PIDController bankingAltitudeController = new PIDController(ALTITUDE_GAIN, ALTITUDE_INTEGRAL, ALTITUDE_DERIVATIVE);


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
     * A private class to manage the turn related instances
     */
    //TODO configure the controller such that it only uses this class to make the turn (keep the turns and the controls separated)
    private class ControlTurn {

        public ControlTurn(AutopilotTurn turn, PhysXEngine.TurnPhysX turnPhysX) {
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
         * The turn that the turn state is specifying
         */
        private final AutopilotTurn turn;

        /**
         * The physics engine turn physics used for calculating the reference velocity and the banking roll
         */
        private final PhysXEngine.TurnPhysX turnPhysX;

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
    }

    /**
     * A class responsible for specifying the control constants used for the flight between the two turns
     */
    private class betweenTurnControl{

        /**
         * Constructor for the control specifications for a flight between two
         * @param nextTurn the next turn to make, the drone needs the entry point such that it can correctly position
         *                 itself for entering the next turn
         */
        public betweenTurnControl(AutopilotTurn nextTurn, AutopilotTurn previousTurn, PhysXEngine.TurnPhysX turnPhysX) {
            this.nextTurn = nextTurn;
            this.previousTurn = previousTurn;
            this.turnPhysX = turnPhysX;
        }

        private void configureInterTurnFlight(PhysXEngine.TurnPhysX turnPhysX){
            //TODO configure the reference velocity and other params needed
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

    }
}
