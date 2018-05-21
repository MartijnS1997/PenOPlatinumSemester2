package internal.Autopilot;

import AutopilotInterfaces.*;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

/**
 * Created by Martijn on 24/04/2018.
 * A class of controllers to make a turn based descend for the drone
 * TODO implement the cas controller in the pitch reference method (we need to adjust the pitch accordingly)
 */
public class DescendController extends TurnBasedController {

    public DescendController(AutoPilot autopilot) {
        super(autopilot);
    }

    /**
     * Getter for the control actions for the turn controller
     * the control outputs contain the commands for the drone to make the specified turn
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @return a control outputs object containing the outputs for the drone to make the turn
     */
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
        //create the control outputs object to store the control actions for the drone
        ControlOutputs outputs = new ControlOutputs(this.getStandardOutputs());

        //update the angle to go
        AutopilotTurn turn = this.getTurn();
        updateAngleToGo(currentInputs, previousInputs, turn);

        if(this.getTargetAltitude() > extractAltitude(currentInputs)){
            this.setDescendRate(0);
        }

        //get the cas commands to avoid collisions
        DescendTurnCAS descendTurnCAS = this.getDescendTurnCAS();
        CasCommand command = descendTurnCAS.getCASCommand(currentInputs, previousInputs);

        //get all the control actions needed to make the turn
        rollControl(outputs, currentInputs, previousInputs);
        pitchControl(outputs, currentInputs, previousInputs, command);
        angleOfAttackControl(getAoaErrorMargin(), outputs, currentInputs, previousInputs);
        thrustControl(outputs, currentInputs, previousInputs);

        outputs.capInclinations(getMainDeltaIncl(), getMainDeltaIncl(), getHorizontalDeltaIncl(), 0);
        return outputs;
    }


    /**
     * Checks if the current controller has reached its objective, the controller is ready if the turn was made
     * or if the altitude upon the initialization of the controller is under 100m
     * @param currentInputs the current inputs (this is the base of the check)
     * @param previousInputs the inputs previously received from the testbed
     * @return true if the controller has finished its turn or if
     *         the drone is below the 100m mark at the first call of the controller
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
       //we check if the controller is called for the first time by checking if the entry point of the turn is
       //close enough to the position of the entry point

        float altitude = extractAltitude(currentInputs);
        if(altitude < getActivationThreshold()){
            if(hasJustStartedTurn()){
                return true;
            }else{
                AutopilotTurn turn = this.getTurn();
                return hasFinishedTurn(turn, currentInputs, previousInputs);
            }
        }

        return false;
//        //we only need to check if the altitude is lower than the activation threshold
//        //check if the current turn has an angle to go of almost equal the turn angle
//        if(altitude < getActivationThreshold()&&hasJustStartedTurn()) {
//            return true;
////            AutopilotTurn turn = this.getTurn();
////            //calculate the entry point
////            Vector relEntry = turn.getEntryPoint();
////            Vector turnCenter = turn.getTurnCenter();
////            Vector entryPoint = relEntry.vectorSum(turnCenter);
////            //the call will be made only after the first invocation of the controller
////            Vector groundPosDrone = extractGroundPosition(previousInputs);
////            float distance = groundPosDrone.distanceBetween(entryPoint);
////            //we account for some distance to be covered after the previous iteration
////            return floatEquals(distance, 0, 10.0f);
//////            return floatEquals(this.getAngleToGo(), TURN_ANGLE, 1E-2f);
//        }
//        //standard objective
//        AutopilotTurn turn = this.getTurn();
//        return (altitude < getActivationThreshold())&&!hasJustStartedTurn()&&hasFinishedTurn(turn, currentInputs, previousInputs); //this.hasFinishedTurn();
    }

    /**
     * Checks if the drone has started the turn
     * @return true if the angle to go is almost (within error) is equal to the turn angle
     */
    private boolean hasJustStartedTurn(){
        float angleToGo = this.getAngleToGo();
        float turnAngle = this.getTurn().getTurnAngle();
        //check if we're still close enough to say that the turn hasn't started yet (with margin for rounding errors)
        return floatEquals(angleToGo, turnAngle, 1E-3f);
    }

    /**
     * Checks if the given floating point number is almost equal to the other floating point number
     * given a absolute error value
     * @param input1 the first float to check
     * @param input2 the second float to check
     * @param absError the absolute error ( >= 0)
     * @return true if the second floating point number is in range (input1 - absError, input1 + absError)
     */
    private static boolean floatEquals(float input1, float input2, float absError){
        absError = abs(absError);
        float upperBorder = input1 + absError;
        float lowerBorder = input1 - absError;

        if(upperBorder <= input2){
            return false;
        }

        if(lowerBorder >= input2){
            return false;
        }

        return true;
    }

    /**
     * Resets all the parameters of the controller that were calculated for the previous descend
     * this method should be called every time the controller is used for a new descend
     */
    @Override
    public void reset(){
        this.turn = null;
        this.turnPhysX = null;
        this.descendRate = 0;
        this.bankingTurnRoll = 0;
        this.turnVelocity = 0;
        try {
            this.descendTurnCAS.getCasPitchPid().reset();
        }catch(NullPointerException e){
            //do nothing, no descend cas was configured
        }
    }


    /**
     * Configures the controller to calculate and make the turn
     * --> sets the roll needed to make the specified turn
     * --> sets the velocity needed to make the specified turn
     * @param currentInputs the current inputs of the drone, this will also be the target of the descend
     */
    protected void configureController(AutopilotConfig config, AutopilotInputs_v2 currentInputs, float descendThreshold, float targetAltitude){
        //save the configuration
        this.setConfig(config);
        //generate the turnPhysics
        PhysXEngine.TurnPhysX turnPhysX = this.getAutopilot().getPhysXEngine().createTurnPhysics();
        this.setTurnPhysX(turnPhysX);
        //generate the turn to be made
        AutopilotTurn turn = this.generateTurn(currentInputs);
        this.setTurn(turn);

        this.setActivationThreshold(descendThreshold);
        this.setTargetAltitude(targetAltitude);
        //calculate the reference roll and velocity for making the turn
        float roll = calculateBankingRoll(turn);
        float velocity = calculateTurnVelocity(roll);

        //save the results
        this.setBankingTurnRoll(roll);
        this.setTurnVelocity(velocity);

        //calculate the descend rate for the drone
        float turnAngle = turn.getTurnAngle();
        float currentAltitude = extractAltitude(currentInputs);
        float descendRate = (currentAltitude - targetAltitude)/(turn.getTurnAngle());
        this.setDescendRate(descendRate);

        this.angleToGo = turnAngle;

        //configure the CAS
        AutopilotCommunicator communicator = this.getAutopilot().getCommunicator();
        DescendTurnCAS descendTurnCAS = new DescendTurnCAS(communicator);
        descendTurnCAS.configureDescendCas(getThreatDistance());
        setDescendTurnCAS(descendTurnCAS);
    }

    /**
     * Generates a counter clockwise turn specifications to make the descend
     * --> center of the turn is located to the left of the drone (viewing from heading axis)
     * --> turn entry is the current position
     * --> turn angle is the whole turn (360)
     * @param currentInputs the current inputs to base the turn generation on
     * @return the specifications of the turn
     */
    private AutopilotTurn generateTurn(AutopilotInputs_v2 currentInputs){
        //grab the parameters needed to define the drone
        Vector dronePosGround = extractGroundPosition(currentInputs);
        Vector droneOrientation = extractOrientation(currentInputs);
        float descendRadius = this.getDescendTurnRadius();
        Vector negXAxisHa = new Vector(-1,0,0); //the negative x axis in the heading axis system, used to determine the turn radius

        //rescale the x-unit vector to get the turn center in the heading axis system
        Vector turnCenterHa = negXAxisHa.normalizeToLength(descendRadius);
        //convert the heading axis center to the relative axis system in the world
        Vector turnCenterRelWorld = PhysXEngine.headingOnWorld(turnCenterHa, droneOrientation);
        //convert the turn center to absolute coordinates
        Vector turnCenterWorld = turnCenterRelWorld.vectorSum(dronePosGround);

        //define the other parameters of the turn
        Vector entryPoint = dronePosGround.vectorDifference(turnCenterWorld);
        float turnAngle = TURN_ANGLE;
        float turnRadius = descendRadius;

        return new AutopilotTurn() {
            @Override
            public Vector getTurnCenter() {
                return turnCenterWorld;
            }

            @Override
            public Vector getEntryPoint() {
                return entryPoint;
            }

            @Override
            public float getTurnRadius() {
                return turnRadius;
            }

            @Override
            public float getTurnAngle() {
                return turnAngle;
            }

            @Override
            public Vector getExitPoint(){return new Vector();}
        };

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
        float referenceVelocity = this.getTurnVelocity();
        PIDController thrustPID = this.getThrustController();
        this.flightCruiseControl(outputs, currentInputs, previousInputs, thrustPID, referenceVelocity);
    }

    /**
     * Generates the control actions to keep the roll in check during a banking turn with turning radius
     * ||dronePos - turnCenter|| and with center turnCenter
     * @param outputs the outputs to write the calculated results to
     * @param currentInputs the latest inputs received by the autopilot from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs) {
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

    private void pitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, CasCommand casCommand){
        //create a switch to select the right action
        switch(casCommand){
            case ASCEND:
                //abnormal behavior of drone, invoke the cas pitch controller
                DescendTurnCAS descendTurnCAS = this.getDescendTurnCAS();
                descendTurnCAS.casPitchControls(outputs, currentInputs, previousInputs);
                break;
            default:
                //reset to be sure
                this.getDescendTurnCAS().getCasPitchPid().reset();
                //reset the pitch controls
                normalPitchControl(outputs, currentInputs, previousInputs);
                //break the case
                break;
        }
    }

    /**
     * Getter for the control actions needed to keep the drone at the cruising altitude during the turn
     * @param outputs the outputs of the drone to write the result to
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     */
    private void normalPitchControl(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
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
     * Calculates the reference point for the pitch controller, it is used to descend to the given altitude
     * the reference point will descend based on the angle to go parameter (if it hits zero the drone shouldn't descend anymore
     * @param currentInputs the current inputs, used to calculate the reference point
     * @return a reference point for the controller in the world axis system to fly to
     */
    private Vector getPitchReferencePoint(AutopilotInputs_v2 currentInputs){

        //grab the parameters needed for the calculations
        Vector groundPosition = extractGroundPosition(currentInputs);
        Vector orientation = extractOrientation(currentInputs);
        Vector headingHa = new Vector(0,0,-1); //the heading vector in the heading axis system
//        float targetAltitude = this.getTargetAltitude();
//        float descendRate = this.getDescendRate();
//        float angleToGo = this.getAngleToGo();
        float lookaheadDistance = this.getLookaheadDistance();

        //calculate the reference altitude, the current angle to go and the descend rate should be used
        float referenceAltitude = getNormalOpReferenceAltitude();/*angleToGo*descendRate + targetAltitude;*/
        Vector altitudeVector = new Vector(0,referenceAltitude,0);

        //calculate the reference point by setting a point in front of the drone (using the lookahead distance)
        Vector lookaheadVectorHa = headingHa.scalarMult(lookaheadDistance);
        Vector lookaheadVectorRel = PhysXEngine.headingOnWorld(lookaheadVectorHa, orientation);
        Vector lookaheadVectorGround = lookaheadVectorRel.vectorSum(groundPosition);
        Vector lookaheadVector = lookaheadVectorGround.vectorSum(altitudeVector);

        return lookaheadVector;
    }

    /**
     * Getter for the reference altitude needed for calculating the pitch reference point
     * The reference altitude is based on the different Cas commands
     * --> see getCasAscendReferenceAltitude() for the ASCEND case
     * --> see getNormalOpReferenceAltitude() for other cases
     * @param currentInputs the inputs most recently received from the testbed
     * @param casCommand the command used to determine the reference altitude
     * @return the reference altitude needed to handle the cas command
     */
    private float getReferenceAltitude(AutopilotInputs_v2 currentInputs, CasCommand casCommand){
        switch (casCommand){
            case ASCEND:
                return getCasAscendReferenceAltitude(currentInputs);
            default:
                return getNormalOpReferenceAltitude();
        }
    }

    /**
     * Getter for the reference altitude in the case that the cas command is an ascend command
     * @param currentInputs the inputs most recently received from the testbed
     * @return the reference altitude for the ascend command
     */
    private float getCasAscendReferenceAltitude(AutopilotInputs_v2 currentInputs){
        //get the lookahead distance and extract the current altitude
        float lookaheadDistance = this.getLookaheadDistance();
        float lookaheadFraction = this.getCasAscendFraction();
        float extraAltitude = lookaheadDistance*lookaheadFraction;

        float currentAltitude = extractAltitude(currentInputs);

        return currentAltitude + extraAltitude;
    }

    /**
     * Getter for the reference altitude in normal operation of the descending contoller
     * @return the reference altitude needed for a steady descend
     */
    private float getNormalOpReferenceAltitude(){
        float targetAltitude = this.getTargetAltitude();
        float descendRate = this.getDescendRate();
        float angleToGo = this.getAngleToGo();

        return angleToGo*descendRate + targetAltitude;

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
        Vector projDiffPa = diffPa.orthogonalProjection(yzNormalPa);

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
    private boolean hasFinishedTurn(){

        //get the specifications for the turn we're currently making
        AutopilotTurn turn = this.getTurn();
        float turnAngle = turn.getTurnAngle();
        float angleToGo =getAngleToGo();

        //check if we've made the whole turn this can be checked easily by comparing the signs of the
        //angle to go and the turn angle, if they differ the angle is finished
        return signum(angleToGo) != signum(turnAngle);
    }

    /**
     * Updates the angle to go, this is the number of randians the drone has still to turn to finish the descend
     * note that this method should be called every iteration
     * @param currentInputs the inputs most recently received from the testbed
     * @param previousInputs the inputs previously received from the testbed
     * @param turn the turn to make
     */
    private void updateAngleToGo(AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs, AutopilotTurn turn) {
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
        float prevAngleToGo = this.getAngleToGo();
        this.setAngleToGo(prevAngleToGo - deltaAngle);
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
        PhysXEngine.TurnPhysX turnPhysX = getTurnPhysX();

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
     * Getter for the turn to be made during the descend, is used as a reference for the controller
     * @param turn the turn to be made by the descending controller
     */
    private void setTurn(AutopilotTurn turn) {
        this.turn = turn;
    }

    /**
     * Getter for the turn physics engine used to calculate the physics related stuff for making the turn
     * @return the turn physics engine
     */
    private PhysXEngine.TurnPhysX getTurnPhysX() {
        return turnPhysX;
    }

    /**
     * Setter for the turn physics used by the controller to regulate the descend of the drone
     * @param turnPhysX the turn physics used by the controller to make the descend
     */
    private void setTurnPhysX(PhysXEngine.TurnPhysX turnPhysX) {
        this.turnPhysX = turnPhysX;
    }

    /**
     * Getter for the radius of the turn used to descend to the target altitude
     * @return the radius of the turn to make in meters
     */
    private float getDescendTurnRadius() {
        return descendTurnRadius;
    }

    /**
     * Setter for the descend radius of the turn (see getter)
     * @param descendTurnRadius the radius of the turn in meters
     */
    private void setDescendTurnRadius(float descendTurnRadius) {
        this.descendTurnRadius = descendTurnRadius;
    }

    /**
     * Getter for the target altitude of the drone, this is the altitude to reach with the drone during the descend
     * @return the altitude to reach after the turn of the drone in meters
     */
    private float getTargetAltitude() {
        return targetAltitude;
    }

    /**
     * Setter for the target altitude, this is the altitude to reach after the turn
     * @param targetAltitude the altitude to reach after the turn in meters
     */
    protected void setTargetAltitude(float targetAltitude) {
        this.targetAltitude = targetAltitude;
    }

    /**
     * Getter for the descend rate, this is the altitude the drone should descend for every radian covered
     * of the turn
     * @return the descend rate of the drone for making this specific descend in meters/radian
     */
    private float getDescendRate() {
        return descendRate;
    }

    /**
     * Setter for the descend rate of the drone (see getter for more info)
     * @param descendRate the descend rate in meters/radian
     */
    private void setDescendRate(float descendRate) {
        this.descendRate = descendRate;
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
     * Setter for the angle to go, this method may only be used to decrement the angle to go
     * keeps track of how many radians are left to turn
     * @param angleToGo the angle to go (updated) in radians
     */
    private void setAngleToGo(float angleToGo) {
        this.angleToGo = angleToGo;
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
     * Getter for the collision avoidance system used during the descend of the drone
     * @return the CAS used during the descend of the drone
     */
    private DescendTurnCAS getDescendTurnCAS() {
        return descendTurnCAS;
    }

    /**
     * Setter for the descend turn CAS used by the controller to avoid a collision during the descend turn
     * @param descendTurnCAS the control avoidance system used during the descend of the drone
     */
    private void setDescendTurnCAS(DescendTurnCAS descendTurnCAS) {
        this.descendTurnCAS = descendTurnCAS;
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
     * Getter for the ascend fraction, used to adjust the reference pitch for the pitch controller in case
     * the cas commanded an ascend. The altitude added to the current altitude of the drone
     * is the specified fraction of the lookahead distance
     * @return the ascend fraction ( 0 < x < 1)
     */
    private float getCasAscendFraction() {
        return casAscendFraction;
    }

    /**
     * Setter for the ascend fraction --> see getter for more info
     * @param casAscendFraction the fraction of of the lookahead distance used for adjusting the reference point
     */
    private void setCasAscendFraction(float casAscendFraction) {
        this.casAscendFraction = casAscendFraction;
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
     * Getter for the activation threshold of the controller if the altitude of the drone is below this value
     * the controller will be skipped and the next controller (landing) must be invoked to guide the landing
     * if not, this controller is triggered and we will start a downwards descend to the target altitude
     * @return the acitvation threshold in meters
     */
    private float getActivationThreshold() {
        return activationThreshold;
    }

    /**
     * Getter for the threat distance, this is the distance wherefore a drone is considered a threat to the current drone
     * @return the threat distance
     */
    private static float getThreatDistance() {
        return threatDistance;
    }

    /**
     * Setter for the activation threshold, if the cruising altitude exceeds this value the descend phase is
     * triggered, if not the descend is skipped
     * @param activationThreshold the activation threshold for the descend phase in meters
     */
    public void setActivationThreshold(float activationThreshold){
        this.activationThreshold = activationThreshold;
    }

    /**
     * Getter for the main stable inclination the reference inclination for the main wings
     * @return the main stable inclination in radians
     */
    private static float getMainStable() {
        return MAIN_STABLE;
    }

    /**
     * Getter for the maximum deviance on the main wings of the drone, this limits te range of possible
     * inclinations for the wings of the drone: (getMainStable - getMainDeltaIncl, getMainStable + getMainDeltaIncl)
     * @return the maximum deviance on the main wings in radians
     */
    private static float getMainDeltaIncl() {
        return MAIN_DELTA_INCL;
    }

    /**
     * Getter for the horizontal stabilizer stable inclination, this is the reference inclination for the horizontal
     * stabilizer
     * @return the horizontal stabilizer stable inclination in radians
     */
    private static float getHorizontalStable() {
        return HORIZONTAL_STABLE;
    }

    /**
     * The maximum deviance in the horizontal stable inclination (analog to the main delta inclination)
     * @return the horizontal deviance in radians
     */
    private static float getHorizontalDeltaIncl() {
        return HORIZONTAL_DELTA_INCL;
    }

    /**
     * Getter for the vertical stabilizer stable inclination, reference inclination for the vertical stabilizer
     * @return the vertical stabilizer stable inclination
     */
    private static float getVerticalStable() {
        return VERTICAL_STABLE;
    }

    /**
     * The maximum deviance of the vertical stabilizer
     * @return the deviance in radians
     */
    private static float getVerticalMax() {
        return VERTICAL_MAX;
    }

    /**
     * Getter for the error of the angle of attack error used to cap down the calculated angle of attack
     * to account for rounding errors and velocity approximation
     * @return the error margin on the aoa calculations
     */
    private float getAoaErrorMargin() {
        return aoaErrorMargin;
    }

    /**
     * Getter for the standard outputs of the controller, used to initialize the control outputs
     * @return the standard outputs object used for defaulting the controller outputs
     */
    private StandardOutputs getStandardOutputs() {
        return standardOutputs;
    }

    /**
     * The turn that the turn state is specifying
     */
    private AutopilotTurn turn;

    /**
     * The collision avoidance system used during the descend turn of the drone
     */
    private DescendTurnCAS descendTurnCAS;

    /**
     * The radius of the turn used to descend to the desired altitude
     */
    private float descendTurnRadius = 1000f;

    /**
     * The target altitude, this is the altitude to reach during the turned descend
     */
    private float targetAltitude = 75f;

    /**
     * The descend rate of the drone, this is the altitude the drone has to descend per radian
     * covered of the turn, is used to adjust the pitch
     */
    private float descendRate;

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
     * The fraction of the lookahead altitude used for calculating the pitch reference point
     * if a ascend command was given (for 0.15 gives approx 8.5 degree pitch reference)
     */
    private float casAscendFraction = 0.15f;

    /**
     * The turn PhysX object used by the autopilot to make turns
     * --> calculates banking angles and such
     */
    private PhysXEngine.TurnPhysX turnPhysX;

    /**
     * The variable that indicates a percentage cap on the correction that is made on the roll reference
     * of the drone
     */
    private final static float rollReferenceCorrectCapPercentage = 0.10f;


    /**
     * The factor used to scale the roll factor correction
     * 100 in denominator = error distance in meters
     * 100 in numerator = for converting the error against the error distance to percent
     * PI/180 = to convert an angle from degree to radians
     */
    private final static float correctRollFactor = (float) ((100*PI)/(100*180));

    /**
     * The altitude that must be breached (higher) for the controller to trigger
     * if the altitude of the drone is below this threshold the controller will be skipped
     * and the next controller (landing) will be invoked
     */
    private float activationThreshold = 100f;

    /**
     * Getter for the threat distance, used for setting the reaction distance for the collision avoidance system
     */
    private final static float threatDistance = 50f;

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
     * The turn angle used for making the turn while descending
     */
    private float TURN_ANGLE = (float) (2*PI);

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

    /**
     * The descend turn CAS used to avoid mid-air collisions during the descending turn
     */
    private class DescendTurnCAS extends DescendCas{

        public DescendTurnCAS(AutopilotCommunicator communicator) {
            super(communicator);
        }


        /**
         * Resets the pitch PID, must be called every time the cas controller is used for the first time
         * after a normal descend phase
         */
        private void resetPitchControls(){
            this.getCasPitchPid().reset();
        }


        /**
         * The pitch controller employed for CAS corrections during the flight
         * @param outputs the outputs to write the results to
         * @param currentInputs the inputs most recently received from the testbed
         * @param previousInputs the inputs previously received from the testbed
         */
        private void casPitchControls(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 previousInputs){
            System.out.println("using cas Controls");
            //get the set point for the pitch
            float referencePitch = this.getReferencePitch();
            //get the current pitch from the inputs
            float currentPitch = Controller.extractPitch(currentInputs);
            //calculate te error
            float errorPitch = referencePitch - currentPitch;
            //get the outputs from the pid controller
            PIDController pitchPid = this.getCasPitchPid();
            float deltaTime = Controller.getDeltaTime(currentInputs, previousInputs);
            //if the reference pitch is larger than the current pitch we have a positive error pitch fed into
            //the PID, the pid will return a negative output --> negative PID output = steer upwards
            //if reference pitch < current pitch, we'll get a negative PID input and a positive output
            //we'll need to steer upwards
            float pidOutputs = pitchPid.getPIDOutput(errorPitch,deltaTime);

            //now calculate the control actions based on the error on the pitch
            //take the linear approach, we want the same inclination of the horizontal stabilizer as the error
            //if possible (first iteration) if we need to go up, negative inclination, if we need to go down
            //positive inclination
            float horizontalInclination = capInclination(pidOutputs, HORIZONTAL_STABLE, HORIZONTAL_DELTA_INCL);
            //put it into the outputs
            outputs.setHorStabInclination(horizontalInclination);
        }

        public float getReferencePitch() {
            return referencePitch;
        }

        /**
         * Configures the descend cas
         * @param descendRate the rate of descend employed by the descend controller
         */
        @Override
        protected void configureDescendCas(float descendRate) {
            float threatDistance = calcThreatRadius(descendRate);
            this.setThreatDistance(threatDistance);
        }

        /**
         * Calculates the threat radius based on the descend rate of the current turn
         * @param descendRate the descend rate to get the threat radius for
         * @return the threat radius needed to avoid collision
         */
        protected float calcThreatRadius(float descendRate){
            //use the fitted poly
            float a0 = 15.93f;
            float a1 = 0.1722f;
            float radiusMult = 1;

            return (a0 + abs(descendRate)*a1)*radiusMult;
        }

        /*
        Some controls stuff
         */

        /**
         * Getter for the PID used by the CAS controller
         * @return the pid used for the cas pitch control
         */
        private PIDController getCasPitchPid() {
            return casPitchPid;
        }

        private final static float CAS_PITCH_GAIN = 1.0f;
        private final static float CAS_PITCH_INTEGRAL = 0.2f;
        private final static float CAS_PITCH_DERIVATIVE = 0.5f;
        private final PIDController casPitchPid = new PIDController(CAS_PITCH_GAIN, CAS_PITCH_INTEGRAL, CAS_PITCH_DERIVATIVE);

        private final float referencePitch = (float) (15*PI/180);
    }
}
