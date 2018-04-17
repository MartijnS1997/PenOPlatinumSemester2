package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;

/**
 * Created by Martijn on 3/04/2018.
 * A class of controllers used to navigate between the airports present in the world
 * TODO implement this class for the final stage of the project
 * TODO account for losses in lift due to the roll of the drone to increase the reference velocity accordingly
 * TODO use for the banking lift the newly found formula for the AOA (see mupad file)
 * TODO configure the navigation controller in the autopilot finite state machine class
 */
public class AirportNavigationController extends AutopilotFlightController {


    public AirportNavigationController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
        return null;
    }

    private void bankControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, Vector turnCenter){
        bankingRollControl(outputs, currentInputs, previousInputs, turnCenter);
        //TODO add altitude control and cruise control

    }

    /**
     * Generates the control actions to keep the roll in check during a banking turn with turning radius
     * ||dronePos - turnCenter|| and with center turnCenter
     * @param outputs the outputs to write the calculated results to
     * @param currentInputs the latest inputs received by the autopilot from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @param turnCenter the center of the turn where the drone has to turn around
     */
    private void bankingRollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, Vector turnCenter) {
        //calculate the angle needed to make the turn (may be done only once in the future)
        //get the parameters first:
        float gravityAcc = this.getConfig().getGravity();
        float referenceVelocity = this.getReferenceVelocity();
        //calculate the roll needed to make the turn
        float bankingRoll = calculateBankingRoll(currentInputs, turnCenter, gravityAcc, referenceVelocity);

        //get the PID and the parameters needed to get the errorOutput
        float roll = extractRoll(currentInputs);
        float rollError = bankingRoll - roll; // the error on the roll of the drone
        float deltaTime = getDeltaTime(currentInputs, previousInputs);

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

        float cappedRightMain = capInclination(referenceVelocity, mainStableInclination, mainDeltaIncl);
        float cappedLeftMain = capInclination(referenceVelocity, mainStableInclination, mainDeltaIncl);

        outputs.setRightWingInclination(cappedRightMain);
        outputs.setLeftWingInclination(cappedLeftMain);
    }

    /**
     * Calculates the roll needed to make the banking turn around a circle with radius || dronePos - turnCenter || and
     * center turnCenter
     * @param currentInputs the latest inputs received from the testbed
     * @param turnCenter the center to turn around with a radius of ||dronePos - turnCenter||
     * @param gravityAcc the gravitational acceleration
     * @param velocity the reference velocity used in the calculation for the banking angle
     * @return the roll needed to make the banking turn as specified above
     * note: negative roll means turn to right and positive roll means turn to left
     */
    private static float calculateBankingRoll(AutopilotInputs_v2 currentInputs, Vector turnCenter, float gravityAcc, float velocity){
        //first calculate the turning radius & calculate the angle needed to bank
        Vector position = extractPosition(currentInputs);
        Vector orientation = extractOrientation(currentInputs);

        float turningRadius = turnCenter.distanceBetween(position); //get the radius of the turn
        float bankingAngle = calculateBankingAngle(turningRadius, gravityAcc, velocity);
        //the banking angle will always be positive so we also need to get the direction needed
        //if the turn center is to the left, the roll needs to be positive and if the turn center is to the right
        //the roll needs to be negative, to get the relative position we need to get the sign of the x-coordinate
        //of the difference vector between the turn center and the drone position in the drone axis system

        Vector centerDiff= turnCenter.vectorDifference(position);
        //get the center turn in the drone axis system
        Vector centerDiffDrone = PhysXEngine.worldOnDrone(centerDiff, orientation);
        //get the x-component & look at the sign
        float xCenterDiffDrone = centerDiffDrone.getxValue();

        //if the x-component is negative, we need to turn left, if positive we need to turn right
        //meaning that the roll needs to be positive is the sign is negative and right if the sign is positive
        return xCenterDiffDrone > 0 ? -bankingAngle : bankingAngle;


    }

    /**
     * Calculates the banking angle needed to make a turn with the provided turning radius and the given
     * velocity assuming there is only a motion in the xz-plane of the world and none in the y-direction
     * @param turningRadius the desired radius of the turn
     * @param gravitationalAcceleration the gravitational acceleration
     * @param velocity the velocity of the drone during the turn
     * @return the banking angle the drone needs to maintain for turning with the given radius
     */
    private static float calculateBankingAngle(float turningRadius, float gravitationalAcceleration, float velocity){
        //formula roll = - arcTan(v^2 / g*r)
        float numerator = velocity*velocity;
        float denominator = turningRadius * gravitationalAcceleration;

        //calculate the result
        float theta = (float) atan2(numerator, denominator);
        //return the roll
        return theta;
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
    private float getReferenceVelocity() {
        return referenceVelocity;
    }


    /**
     * Setter for the reference velocity used by the cruise control. This is the velocity that will be maintained
     * by the drone during the flight
     * @param referenceVelocity the total velocity to be set as a new reference (>0)
     */
    public void setReferenceVelocity(float referenceVelocity) {
        this.referenceVelocity = referenceVelocity;
    }

    /**
     * Checks if the provided reference velocity is valid
     * @param referenceVelocity the reference velocity to check
     * @return true if and only if referenceVelocity > 0
     */
    private boolean isValidReferenceVelocity(float referenceVelocity){
        return referenceVelocity > 0;
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
     * The cruising altitude assigned to the drone by the overseer, this is the reference altitude for the
     * altitude PID controllers ( there are different controllers for different scenario's)
     */
    private float cruisingAltitude;

    /**
     * The reference velocity for drone during the flight between the airports, this value is used as a set point for
     * the cruise control controller
     */
    private float referenceVelocity;


    /**
     * Controllers needed for banking the drone for a given radius
     * --> thrust controller: needed for the cruise control maintaining the velocity
     * --> roll controller: needed to keep the roll stable to maintain a good bank
     * --> altitude controller: needed to keep the drone at the same level (constant height)
     * note: the roll and the altitude controllers need to react slowly to changes (so the control actions
     * do not go in overdrive, maybe use low pass filter?)
     */
    private final static float THRUST_GAIN = 1f;
    private final static float THRUST_INTEGRAL = 0f ;
    private final static float THRUST_DERIVATIVE = 0f;
    private PIDController bankingThrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

    private final static float ROLL_GAIN = 1f;
    private final static float ROLL_INTEGRAL = 0f;
    private final static float ROLL_DERIVATIVE = 0f;
    private PIDController bankingRollController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    private final static float ALTITUDE_GAIN = 1f;
    private final static float ALTITUDE_INTEGRAL = 0f;
    private final static float ALTITUDE_DERIVATIVE = 0f;
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

}
