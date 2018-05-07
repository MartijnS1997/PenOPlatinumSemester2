package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

/**
 * Created by Martijn on 2/05/2018.
 * A class of taxiing controllers, to be implemented by controllers that take care of part of the taxiing
 */
public abstract class TaxiingController extends Controller {

    public TaxiingController(AutoPilot autopilot) {
        super(autopilot);
    }

    /**
     * The controller for the brakes while navigating for turning the drone to the target
     *
     * @param outputs       the outputs to write the control actions to
     * @param currentInputs the current outputs used to extract data
     * @param prevInputs    the current inputs used to extract data
     * @param target        the target for the turning controls to move to
     * @param brakeController the controller used to steer the brakes, must be configured by the subclass using this method
     */
    protected void brakeTurningControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs, Vector target, PIDController brakeController) {
        //get the time difference between two steps, note that this is an extrapolation, we assume here that
        //the time between simulation steps stay the same
        float deltaTime = Controller.getDeltaTime(currentInputs, prevInputs);
        //get the max R:
        float maxBrake = this.getConfig().getRMax();

        //then we need to calculate our angle
        float angle = angleToTarget(currentInputs, target);
        //get the outputs
        float pidOutputs = brakeController.getPIDOutput(angle, deltaTime);
        //will give negative output if we need to go right, positive if we need to go to the left
//        System.out.println("brakePidOutputs " + pidOutputs);
        outputs.setLeftBrakeForce(min(abs(min(pidOutputs, 0)), maxBrake));
        outputs.setRightBrakeForce(min(abs(max(pidOutputs, 0)), maxBrake));
//        System.out.println("Turning " + (outputs.getRightBrakeForce() > 0? "right" : "left"));
    }

    public void stayWithinAirportBorders(Vector[] airportBorders, ControlOutputs outputs, AutopilotInputs_v2 inputs){
        Vector topLeft = airportBorders[0];
        Vector topRight = airportBorders[1];
        Vector bottomLeft = airportBorders[2];
        Vector bottomRight = airportBorders[3];

        Vector position = Controller.extractPosition(inputs);
        float maxBrake = this.getConfig().getRMax();


        float x = position.getxValue();
        float z = position.getzValue();

        float x1 = topLeft.getxValue();
        float z1 = topLeft.getzValue();
        float x2 = bottomLeft.getxValue();
        float z2 = bottomLeft.getzValue();

        float x3 = topRight.getxValue();
        float z3 = topRight.getzValue();
        float x4 = bottomRight.getxValue();
        float z4 = bottomRight.getzValue();

        float d;

        //check on what side of a line A(x1,y1) & B(x2,y2) a point (x,y) is:
        //d = (x-x1)(y2-y1)-(y-y1)(x2-x1)
        //d < 0 = left, d > 0 right
        if (topLeft.getxValue() < topRight.getxValue()){
            d = (x-(x1+AIRPORT_BOUND_BUFFER))*(z2-z1) - (z-z1)*((x2+AIRPORT_BOUND_BUFFER)-(x1+AIRPORT_BOUND_BUFFER));
            if (d < 0){
                outputs.setRightBrakeForce(maxBrake);
            }

            d = (x-(x3-AIRPORT_BOUND_BUFFER))*(z4-z3) - (z-z3)*((x4-AIRPORT_BOUND_BUFFER)-(x3-AIRPORT_BOUND_BUFFER));
            if (d > 0){
                outputs.setLeftBrakeForce(maxBrake);
            }
        }
        else{
            d = (x-(x1-AIRPORT_BOUND_BUFFER))*(z2-z1) - (z-z1)*((x2-AIRPORT_BOUND_BUFFER)-(x1-AIRPORT_BOUND_BUFFER));
            if (d > 0){
                outputs.setRightBrakeForce(maxBrake);
            }

            d = (x-(x3+AIRPORT_BOUND_BUFFER))*(z4-z3) - (z-z3)*((x4+AIRPORT_BOUND_BUFFER)-(x3+AIRPORT_BOUND_BUFFER));
            if (d < 0){
                outputs.setLeftBrakeForce(maxBrake);
            }
        }
    }

    private final static float AIRPORT_BOUND_BUFFER = 2f;

    /**
     * Calculates the angle between vector difference of the current target and the drone, and the drone's heading vector
     * positive angles indicate a need for steering to the left
     * and negative angles indicate a need for steering to the right
     *
     * @param target the target of the controller
     * @return the angle between the target and the heading vector of the drone
     * the sign of the angle indicates in which direction the controller needs to steer
     */
    protected static float angleToTarget(AutopilotInputs_v2 inputs, Vector target) {
        Vector position = Controller.extractPosition(inputs);
        Vector orientation = Controller.extractOrientation(inputs);
        //calculate the difference vector between the target and our current position
        Vector diffVector = target.vectorDifference(position);
        //now project the vector on the xz-plane && normalize
        Vector normal = new Vector(0, 1, 0); // the normal vector of the xz-plane
        Vector projDiffVector = diffVector.orthogonalProjection(normal); // project onto the plane

        //now calculate the current heading vector of the drone in the world axis system
        Vector headingDrone = new Vector(0, 0, -1);
        //transform
        Vector headingWorld = PhysXEngine.droneOnWorld(headingDrone, orientation);
        //project
        Vector projHeadingWorld = headingWorld.orthogonalProjection(normal);

        //now get the angle between both
        float angle = abs(projHeadingWorld.getAngleBetween(projDiffVector));

//        System.out.println("Projected heading vector: " + projHeadingWorld);
//        System.out.println("Projected diff vector: " + projDiffVector);
        //calculate the direction, we use the vector product of the heading vector and the difference vector
        //a positive vector indicates that the target is located to the left, a negative angle indicates the
        //target is located at the right of the drone (the result is completely located on the y-axis, the normal)
        float direction = signum(projHeadingWorld.crossProduct(projDiffVector).scalarProduct(normal));
//        System.out.println("Direction: " + direction);

        float result = angle * direction;
        //check for NaN
        if (Float.isNaN(result)) {
            return 0f;
        }

        return result;
    }

    /**
     * The controller that regulates the velocity during the taxiing phase of the drone
     *
     * @param outputs       the outputs where the control actions are written to
     * @param prevInputs    the previous inputs of the autopilot
     * @param currentInputs the current inputs of the autopilot
     *                      note: only invoke this function AFTER writing the brake controls, otherwise the behaviour
     *                      of the drone is unknown
     */
    protected void cruiseControl(ControlOutputs outputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs) {
        //we need to get an approx for the velocity before we can make any further calculations
        Vector approxVel = Controller.getVelocityApprox(currentInputs, prevInputs);
        float totalVelocityApprox = approxVel.getSize();
        //get the total mass of the drone (needed for the calculations)
        float totalMass = this.getTotalMass();
        //get the velocity controller
        PIDController velocityController = this.getVelocityController();
        //get the delta time, note this is an extrapolation, we assume the time step stays the same across the simulation
        float deltaTime = Controller.getDeltaTime(currentInputs, prevInputs);
        float errorVelocity = this.getTaxiVelocity() - totalVelocityApprox;
//        System.out.println(approxVel);
        //now calculate the output of the PID
        float errorVelPID = velocityController.getPIDOutput(-errorVelocity, deltaTime);
//        System.out.println("error on velocity: "+ errorVelPID);
        //based on the error in the velocity we can calculate our control actions
        //if the error is positive we need to accelerate (setpoint - velocity = error)
        //if the error is negative we need to brake
        if (errorVelPID > 0) {
//            System.out.println("Adjusting thrust");
            float maxThrust = this.getConfig().getMaxThrust();
            float thrust = getCorrectionThrust(totalMass, errorVelPID, maxThrust, deltaTime);
            outputs.setThrust(thrust);
        } else if (errorVelPID < 0) {
//            System.out.println("Adjusting brakes");
            float maxBrake = this.getConfig().getRMax();
            Vector brakeVector = getCorrectionBrakes(totalMass, errorVelPID, maxBrake, deltaTime, outputs);
            setBrakeVector(outputs, brakeVector);
        }
    }

    /**
     * Writes the provided brake vector to the provided outputs, overwriting the previous outputs
     *
     * @param outputs     the outputs where the brake vector is written to
     * @param brakeVector a vector containing the brake forces, the front, left and right brake forces
     *                    are respectively the x, y and z components of the vector
     */
    private static void setBrakeVector(ControlOutputs outputs, Vector brakeVector) {
        outputs.setFrontBrakeForce(brakeVector.getxValue());
        outputs.setLeftBrakeForce(brakeVector.getyValue());
        outputs.setRightBrakeForce(brakeVector.getzValue());
    }

        /**
         * Calculates the correction needed by the brakes to compensate for the error on the velocity
         *
         * @param totalMass     the total mass of the drone
         * @param deltaVelocity the velocity error
         * @param maxBrake      the maximum brake force that the drone can exert on a tyre
         * @param deltaTime     the time difference between two simulation steps
         * @param outputs       the outptus that were already written before this method was invoked
         * @return a vector containing the brake forces needed to correct the erroneous velocity
         * the front, left and right brake forces are the x, y and z components of the vector respectively
         */
    private static Vector getCorrectionBrakes(float totalMass, float deltaVelocity, float maxBrake, float deltaTime, ControlOutputs outputs) {
        //TODO corrective forces of the brakes may override the forces calculated by the steering controller, maybe scale the brake force down until the calculated ones for turning are at cap
        //assumption, all the brakes carry the same brake force and this scalar may be divided by 3 to calc the force needed at each tyre
        float totalBrakeForce = -deltaVelocity * totalMass / deltaTime;//minus to make the result positive
        //now calc the brake force needed for each tyre and cap it to the max brake force
        float desiredTyreBrakeForce = min(totalBrakeForce / 3f, maxBrake);


        //get the current brake vector
        Vector currentBrakeVector = extractBrakeVector(outputs);
        //cap the vector components
        return getCompatibleBrakeVector(maxBrake, desiredTyreBrakeForce, currentBrakeVector);
    }

    /**
     * Extracts a vector of brake forces from the given outputs
     *
     * @param outputs the outputs that are currently generated (may be overwritten later)
     * @return an immutable vector containing the front, left and right brake force as its x, y and z components
     */
    protected static Vector extractBrakeVector(ControlOutputs outputs) {
        float currentFrontBrake = outputs.getFrontBrakeForce();
        float currentLeftBrake = outputs.getLeftBrakeForce();
        float currentRightBrake = outputs.getRightBrakeForce();
        return new Vector(currentFrontBrake, currentLeftBrake, currentRightBrake);
    }

    /**
     * note: ignore the whole documentation hogwash and read the example
     * Checks if the current desired brake force doesn't overwrite any of the control actions taken by the steering controls
     *
     * @param maxBrake              the maximum brake force
     * @param desiredTyreBrakeForce the desired brake force for the tyres
     * @param currentBrakeVector    the brake vector currently stored in the outputs of the controller
     *                              front, left and right brake are the x, y and z components respectively
     * @return returns a new brake vector that is either the sum of the current brake force with the desired one
     * or a version where the desired brake force was capped in such a way that a uniform desired brake force vector
     * (with the same components on x,y,z) summed with the current brake vector produces the maximum brake force
     * for the largest component of the current brake force
     * <p>
     * (eg if desired brake force is (1000,1000,1000) and current is (500,0,0)
     * with a maximum brake force of 1000, the resulting vector is (1000, 500, 500)
     */
    private static Vector getCompatibleBrakeVector(float maxBrake, float desiredTyreBrakeForce, Vector currentBrakeVector) {
        Vector desiredBrakeVector = new Vector(desiredTyreBrakeForce, desiredTyreBrakeForce, desiredTyreBrakeForce);
        //check if we will overwrite a previous control action
        Vector sumVector = currentBrakeVector.vectorSum(desiredBrakeVector);
        float maxVal = sumVector.getMaxComponent();
        //the difference between the max value of the sum and the maximum exertable brake force
        float diffBrakeForce = maxVal - maxBrake;
        //if the max value is larger than the max brake, we need to rescale
        if (maxVal > maxBrake) {
            float modTyreBrakeForce = desiredBrakeVector.getzValue() - diffBrakeForce; // the size of the brake vector at z is equal to the break force exerted on the tyre (in ideal circumstances)
            Vector modBrakeVector = new Vector(modTyreBrakeForce, modTyreBrakeForce, modTyreBrakeForce);
            sumVector = modBrakeVector.vectorSum(currentBrakeVector);
        }
        return sumVector;
    }

        /**
         * Calculates the corrective thrust action needed to keep the velocity up to level
         *
         * @param totalMass     the total mass, needed for the calcultations
         * @param deltaVelocity the error on the velocity
         * @param maxThrust     the maximum thrust the drone can deliver
         * @param deltaTime     the time difference
         * @return the thrust needed to make the error on the velocity zero, capped on the max thrust
         */
    private static float getCorrectionThrust(float totalMass, float deltaVelocity, float maxThrust, float deltaTime) {
        //first calculate the thrust needed to reach the desired velocity
        float desiredThrust = deltaVelocity * totalMass / deltaTime;
        //now check if we can reach it
        return min(desiredThrust, maxThrust);

    }

    protected StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

    /**
     * The standard outputs for this controller, is used to initialize the control outputs of the drone
     */
    private StandardOutputs standardOutputs = new StandardOutputs() {
        @Override
        public float getStandardRightMainInclination() {
            return STANDARD_MAIN;
        }

        @Override
        public float getStandardLeftMainInclination() {
            return STANDARD_MAIN;
        }

        @Override
        public float getStandardHorizontalStabilizerInclination() {
            return STANDARD_HORIZONTAL;
        }

        @Override
        public float getStandardVerticalStabilizerInclination() {
            return STANDARD_VERTICAL;
        }

        @Override
        public float getStandardThrust() {
            return 0;
        }
    };

    /**
     * Getter for the taxi velocity, this is the reference velocity for the taxiing controller
     * @return the taxiing velocity in m/s
     */
    private float getTaxiVelocity() {
        return taxiVelocity;
    }

    /**
     * Setter for the taxi velocity, this is the reference velocity used by the cruise control
     * @param taxiVelocity the reference velocity used while taxiing
     */
    protected void setTaxiVelocity(float taxiVelocity) {
        this.taxiVelocity = taxiVelocity;
    }

    /**
     * Getter for the standstill velocity, this is the velocity wherefore the drone is practically in standstill
     * is used by the extending controllers to check termination conditions
     * @return the standstill velocity in m/s
     */
    protected float getStandStillVelocity() {
        return standStillVelocity;
    }

    /**
     * Setter for the standstill velocity (see getter for more info)
     * @param standStillVelocity the standstill velocity in m/s and > 0
     */
    protected void setStandStillVelocity(float standStillVelocity) {
        this.standStillVelocity = standStillVelocity;
    }

    /**
     * The velocity that is used as a reference for the cruise control of the taxi controller
     */
    private float taxiVelocity = 2f;

    /**
     * The velocity wherefore the drone is practically in standstill, this constant is used by the controllers
     * to check their termination conditions
     */
    private float standStillVelocity = 0.1f;


    /**
     * Main wing configuration
     * --> standard main, the usual inclination of the main wings
     * --> max inclination delta, the maximum deviation from the standard inclination of the main wings
     */
    private static float STANDARD_MAIN = (float) (5 * PI / 180);
    private static float MAX_MAIN_DELTA = (float) (2 * PI / 180);

    /**
     * Horizontal stabilizer configuration
     * --> standard horizontal, the usual inclination of the stabilizer
     * --> max horizontal, the maximum horizontal stabilizer inclination
     */
    private static float STANDARD_HORIZONTAL = 0f;
    private static float MAX_HORIZONTAL = (float) (8f * PI / 180f);

    /**
     * Vertical stabilizer configuration
     * --> standard vertical, the usual vertical inclination
     * --> max vertical, the maximum vertical stabilizer inclination
     */
    private static float STANDARD_VERTICAL = 0f;
    private static float MAX_VERTICAL = 0f;

    /*
    General controllers needed for the cruise control
     */
    /**
     * Getter for the controller that controls the velocity of the drone for navigating to our target
     *
     * @return the controller used to keep our velocity
     */
    private PIDController getVelocityController() {
        return velocityController;
    }

    /**
     * The controller used to determine the velocity of the drone
     */
    private final static float VELOCITY_GRAIN = 1;
    private final static float VELOCITY_INTEGRAL = 0;
    private final static float VELOCITY_DERIVATIVE = 0;
    private PIDController velocityController = new PIDController(VELOCITY_GRAIN, VELOCITY_INTEGRAL, VELOCITY_DERIVATIVE);
}
