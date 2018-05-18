package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.SquareMatrix;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;
import static java.lang.Math.PI;

/**
 * Created by Martijn on 18/05/2018.
 * An abstract class of controllers that use turns, is used to define common methods used by controllers that make turns
 */
public abstract class TurnBasedController extends Controller{

    public TurnBasedController(AutoPilot autopilot) {
        super(autopilot);
    }

    protected static boolean hasFinishedTurn(AutopilotTurn turn, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //get the exit point of the turn
        Vector turnCenter = turn.getTurnCenter();
        Vector turnEntry = turn.getEntryPoint();
        float turnAngle = turn.getTurnAngle();
        //calculate the previous and current positions relative to the turn center
        Vector currentGround = extractGroundPosition(currentInputs);
        Vector previousGround = extractGroundPosition(previousInputs);

        Vector currentRel = currentGround.vectorDifference(turnCenter);
        Vector previousRel = previousGround.vectorDifference(turnCenter);

        //calculate the exit point
        Vector exitPoint = rotateTurnVector(turnEntry, turnAngle);

        //first do the cheapest test
        if(!exitTest(currentRel, exitPoint)){
            return false;
        }
        //then the more expensive one
        return orthogonalCrossedTest(currentRel, previousRel, exitPoint);


    }

    /**
     * Checks if the current position of the drone is in the same half of the turn as the turn exit
     * @param currentPositionRelativeDrone the position of the drone relative to the center of the turn
     * @param turnExit the exit point of the turn relative to the center of the turn
     * @return true if and only if the scalar product of both vectors is positive
     * note: this test is used in combination with the orthogonal test to see if the drone has finished the turn
     *       --> because this one does require less computation, do this one before the other test
     */
    private static boolean exitTest(Vector currentPositionRelativeDrone, Vector turnExit){
        //take the scalar product of  the relative position of the drone with  the vector pointing from the center to the exit point
        float scalarProd = currentPositionRelativeDrone.scalarProduct(turnExit);
        //check if the sign is positive, if so, the drone is located at the same half of the turn as the exit point
        return scalarProd > 0;
    }

    /**
     * Checks if the current position has a different value as the previous position when projected onto
     * the vector orthogonal to the exit point vector
     * (another test is needed because this one will also return true at the other side
     * @param currentRelativePositionDrone the current relative position of the drone (relative to turn center)
     * @param previousRelativePositionDrone the previous relative position of the drone (relative to turn center)
     * @param turnExitPoint the exit point of the turn relative to the turn center
     * @return true if the sign of the scalar product of the projections onto the right orthogonal is negative
     */
    private static boolean orthogonalCrossedTest(Vector currentRelativePositionDrone, Vector previousRelativePositionDrone, Vector turnExitPoint) {
        //get the right orthogonal of the exit point
        Vector rightOrthonormal = turnExitPoint.getRightOrthonormal();

        //project the current and previous relative positions (to turn center) onto the orthogonal
        Vector currentProj = currentRelativePositionDrone.projectOn(rightOrthonormal);
        Vector previousProj = previousRelativePositionDrone.projectOn(rightOrthonormal);

        //check if the sign of the scalar product of the projections, if positive, they are faced in the same
        //direction and the drone has not yet passed the end point, if negative, it has.
        return signum(currentProj.scalarProduct(previousProj)) < 0;
    }

    /**
     * rotates the given turn vector along the given angle in the xz-plane
     * if the angle is positive the turn vector is rotated anti-clockwise
     * if the angle is negative the turn vector is rotated clockwise
     * @param turnVector the vector to rotate for the given angle
     * @param angle the angle to rotate the turn vector for (in radians)
     * @return a vector that is rotated as described above
     */
    protected static Vector rotateTurnVector(Vector turnVector, float angle){
        //create a new square matrix
        float[] matrixArray = new float[]{(float) cos(angle), 0, (float) sin(angle),
                0,1,0,
                (float) -sin(angle), 0, (float) cos(angle)};
        SquareMatrix rotationMatrix = new SquareMatrix(matrixArray);
        //transform
        return rotationMatrix.matrixVectorProduct(turnVector);
    }


    /**
     * A class for configuring the controller for a turn and generating the control actions needed
     * to make the specified turn given the current and previous inputs of the autopilot
     * note: on every new turn a new turn controller has to be initialized
     */
    protected class TurnControl {

        public TurnControl(AutopilotTurn turn, PhysXEngine.TurnPhysX turnPhysX, StandardOutputs standardOutputs, float cruisingAltitude) {
            this.turn = turn;
            this.turnPhysX = turnPhysX;
            this.angleToGo = turn.getTurnAngle();
            this.cruisingAltitude = cruisingAltitude;
            this.standardOutputs = standardOutputs;
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
        protected AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //create the control outputs object to store the control actions for the drone
            ControlOutputs outputs = new ControlOutputs(this.getStandardOutputs());

            //get all the control actions needed to make the turn
            rollControl(outputs, currentInputs, previousInputs);
            pitchControl(outputs, currentInputs, previousInputs);
            angleOfAttackControl(getAoaErrorMargin(), outputs, currentInputs, previousInputs);
            thrustControl(outputs, currentInputs, previousInputs);

            //update the angle to go every time the turn control is invoked:
            AutopilotTurn currentTurn = this.getTurn();
            float deltaAngle = this.calcDeltaAngle(currentInputs, previousInputs, currentTurn);
            reduceAngleToGo(deltaAngle);

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
        protected void thrustControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the parameters needed to call the cruise control
            float referenceVelocity = this.getTurnVelocity();
            PIDController thrustPID = this.getThrustController();
            TurnBasedController.this.flightCruiseControl(outputs, currentInputs, previousInputs, thrustPID, referenceVelocity);
        }

        /**
         * Generates the control actions to keep the roll in check during a banking turn with turning radius
         * ||dronePos - turnCenter|| and with center turnCenter
         * @param outputs the outputs to write the calculated results to
         * @param currentInputs the latest inputs received by the autopilot from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        protected void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
            //calculate the angle needed to make the turn (may be done only once in the future)
            //get the parameters first:

            //calculate the roll needed to make the turn
            float bankingRoll = this.getBankingTurnRoll();
            //correct the banking roll for the error on the turn radius
            float correctedBanking = correctReferenceRoll(bankingRoll, currentInputs, previousInputs);
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
         * @param previousInputs  the inputs previously received from the testbed
         * @return the corrected roll
         */
        private float correctReferenceRoll(float referenceRoll, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            //get the current turn
            AutopilotTurn currentTurn = this.getTurn();
            float turnRadius = currentTurn.getTurnRadius();

            //get the current distance from the center (measured in ground coordinates)
            Vector droneGroundPos = extractGroundPosition(currentInputs);
            Vector turnCenter = currentTurn.getTurnCenter();
            Vector radiusVector = droneGroundPos.vectorDifference(turnCenter);
            float actualRadius =  radiusVector.getSize();
            float errorInput = actualRadius-turnRadius;
            float deltaTime = getDeltaTime(currentInputs, previousInputs);
            PIDController rollCorrectionPID = this.getRollCorrectController();


            float errorOutput = rollCorrectionPID.getPIDOutput(errorInput, deltaTime);
            //if we're making the turn too sharp the error input will be negative (the output positive)
            //if the turn is too shallow we the error input will be positive (the output negative)
            //thus if the output is positive the reference roll must become smaller (we're steering to sharp)
            //and if the output is negative the roll must become sharper
            float scalingFactor = this.getCorrectRollFactor();
            //get the correction based on the error and a scaling factor
            float correctedReference = (referenceRoll  -  errorOutput*scalingFactor);
            float percentageCap = this.getRollReferenceCorrectCapPercentage();
            correctedReference = percentageCap(correctedReference, referenceRoll, percentageCap);
            //errorLog(correctedReference);
//            errorLog(errorInput);
            //get the error ratio, if the the drone is currently to far from the center we have to roll harder
            //if we're to close we may loosen the roll a bit
            return correctedReference;
        }

        /**
         * Caps the input within an percentage error of the reference
         * the acceptable range is defined as (reference*(1-maxErrorPercent), (reference*(1+maxErrorPercent))
         * @param input the inputs to cap
         * @param reference the reference point, this is the point where the borders are calculated from
         * @param maxErrorPercentage the error percentage
         * @return if the input is within range (reference*(1-maxErrorPercent), (reference*(1+maxErrorPercent)) it is
         *         returned unchanged, if it lies outside of the range, returns the closest border point of the range
         */
        private float percentageCap(float input, float reference, float maxErrorPercentage){
            //establish borders
            float lower = reference*(1-maxErrorPercentage);
            float upper = reference*(1+maxErrorPercentage);

            //cap the actual input, if it lies outside of the range, bring it to the closest border
            if(input > upper){
                return upper;
            }
            if(input < lower){
                return lower;
            }

            return input;
        }

        /**
         * Getter for the control actions needed to keep the drone at the cruising altitude during the turn
         * @param outputs the outputs of the drone to write the result to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        protected void pitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
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
            float cruisingAltitude = this.getCruisingAltitude();
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
            Vector projDiffPa = new Vector(0, diffPa.getyValue(), diffPa.getzValue());//diffPa.orthogonalProjection(yzNormalPa);

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
         * Checks if the controller has already started the turn
         * @return true if and only if !floatEquals(turnAngle, angleToGo)
         */
        protected boolean hasStartedTurn(){
            AutopilotTurn turn = this.getTurn();
            float turnAngle = turn.getTurnAngle();
            float angleToGo = this.getAngleToGo();

            return !floatEquals(turnAngle, angleToGo);
        }
        /**
         * Checks if the drone has finished making the current specified turn
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
        protected boolean hasFinishedTurn(){

            //get the specifications for the turn we're currently making
            AutopilotTurn turn = this.getTurn();
            float turnAngle = turn.getTurnAngle();

            //check if we've made the whole turn this can be checked easily by comparing the signs of the
            //angle to go and the turn angle, if they differ the angle is finished
            float angleToGo = this.getAngleToGo();
            return signum(angleToGo) != signum(turnAngle);
        }

        /**
         * Calculates the difference in the angle to go based on the previous and current inputs
         * the angle to go decreases to zero if the turn is counterclockwise
         * and increases to zero if the turn is clockwise
         * --> this method is used as a prelude to the increment angle to go and also for the final turn
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         * @param turn the turn to make
         */
        protected float calcDeltaAngle(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, AutopilotTurn turn) {
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
            float turnDiffAngle = abs(previousTurnVector.getAngleBetween(currentTurnVector));

            //determine the sign, take the vector product of the previous and the current turn vector
            //take the y-component of the product and take the sign
            float sign = signum(previousTurnVector.crossProduct(currentTurnVector).getyValue());

            //calculate and return the delta angle
            float deltaAngle = sign*turnDiffAngle;
            return deltaAngle;
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
            PhysXEngine.TurnPhysX turnPhysX = this.getTurnPhysX();

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
        protected AutopilotTurn getTurn() {
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
        protected float getAngleToGo() {
            return angleToGo;
        }

        /**
         * Decrements the angle to go with the given delta angle
         * @param deltaAngle the rotation around the center that was achieved during the last iteration
         *                   in radians
         *                   note: may me negative in the case that we need to do a clockwise turn, needs to be
         *                   positive if we're doing a counter clockwise turn
         */
        private void reduceAngleToGo(float deltaAngle) {
            this.angleToGo -= deltaAngle;
        }

        /**
         * Resets the state of the angle to go variable
         * after the call the angle to go is yet again equal to the angle needed to make the whole turn
         * --> must be called if the controller is to be re-used for a new turn
         */
        protected void resetAngleToGo(){
            AutopilotTurn turn = this.getTurn();
            float turnAngle = turn.getTurnAngle();
            //hard-reset the angle to go (not normal flow)
            this.angleToGo = turnAngle;
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
         * Getter for the cruising altitude of the drone, this is the altitude the drone has to maintain during a turn
         * @return the cruising altitude in meters
         */
        private float getCruisingAltitude() {
            return cruisingAltitude;
        }

        /**
         * Getter for the percentage cap on the reference roll
         * the correction is used to counter an error in radius of the turn that the drone is currently making
         * in respect to the current turn
         * @return the reference correction correction cap range (0,1)
         */
        private float getRollReferenceCorrectCapPercentage() {
            return rollReferenceCorrectCapPercentage;
        }

        /**
         * Getter for the correction factor on the roll, used to scale the error on the roll
         * @return the scale factor for correcting the roll
         */
        private float getCorrectRollFactor() {
            return correctRollFactor;
        }

        /**
         * Getter for the standard output for the drone
         * @return the standard outputs
         */
        private StandardOutputs getStandardOutputs() {
            return standardOutputs;
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
         * The cruising altitude that the controller has to maintain during the flight
         */
        private final float cruisingAltitude;

        /**
         * The turn PhysX object used by the autopilot to make turns
         * --> calculates banking angles and such
         */
        private final PhysXEngine.TurnPhysX turnPhysX;

        /**
         * The variable that indicates a percentage cap on the correction that is made on the roll reference
         * of the drone
         */
        private final float rollReferenceCorrectCapPercentage = 0.10f;


        /**
         * The factor used to scale the roll factor correction
         * 100 in denominator = error distance in meters
         * 100 in numerator = for converting the error against the error distance to percent
         * PI/180 = to convert an angle from degree to radians
         */
        private final float correctRollFactor = (float) ((100*PI)/(100*180));

        /**
         * The standard outputs of the drone, these are used to create a new ControlOutputs object
         */
        private final StandardOutputs standardOutputs;

        /*
        Methods belonging to the standard outputs
         */

        private float getMainStable(){
            return standardOutputs.getStandardLeftMainInclination();
        }

        private float getHorizontalStable(){
            return standardOutputs.getStandardHorizontalStabilizerInclination();
        }


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

        /**
         * Controller for altering the reference roll such that the drone keeps a distance equal to the radius
         * of the turn from the center
         * @return the controller tuned for correcting the reference roll
         */
        private PIDController getRollCorrectController(){
            return rollCorrectController;
        }

        private final static float THRUST_GAIN = 1.0f;
        private final static float THRUST_INTEGRAL = 0.2f ;
        private final static float THRUST_DERIVATIVE = 0.1f;
        private final PIDController thrustController = new PIDController(THRUST_GAIN, THRUST_INTEGRAL, THRUST_DERIVATIVE);

        private final static float ROLL_GAIN = 1.0f;
        private final static float ROLL_INTEGRAL = 0.0f;
        private final static float ROLL_DERIVATIVE = 0.5f;
        private final PIDController bankingRollController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

        private final static float PITCH_GAIN = 1.0f;
        private final static float PITCH_INTEGRAL = 0.6f;
        private final static float PITCH_DERIVATIVE = 0.7f;
        private final PIDController pitchController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);


        //TODO try other approach to correct for the roll
        private final static float ROLL_CORRECT_GAIN = 0.3f;
        private final static float ROLL_CORRECT_INTEGRAL = 0.f;
        private final static float ROLL_CORRECT_DERIVATIVE = 1.0f;
        private final PIDController rollCorrectController = new PIDController(ROLL_CORRECT_GAIN, ROLL_CORRECT_INTEGRAL, ROLL_CORRECT_DERIVATIVE);

        /*
        Some constants needed to steer the drone (and their getters)
         */

        /**
         * Getter for the maximum deviance of the main wing inclination, used to cap the wing inclination of the drone
         * @return the main wing inclination deviation cap in radians
         */
        private float getMainDeltaIncl(){
            return MAIN_DELTA_INCL;
        }

        /**
         * Getter for the maximum deviance of the horizontal stabilizer inclination, used to cap the wing inclination of the drone
         * @return the horizontal stabilizer inclination cap in radians
         */
        private float getHorizontalDeltaIncl(){
            return HORIZONTAL_DELTA_INCL;
        }

        /**
         * Getter for the safety margin to be used when adjusting the wing inclinations such that
         * the inclination wont exceed the AOA
         * @return the safety margin in radians
         */
        private float getAoaErrorMargin(){
            return AOA_ERROR_MARGIN;
        }

        private final static float MAIN_DELTA_INCL = (float) (2*PI/180);
        private final static float HORIZONTAL_DELTA_INCL= (float) (8*PI/180);
        private final static float AOA_ERROR_MARGIN = (float) (2*PI/180);
    }
}
