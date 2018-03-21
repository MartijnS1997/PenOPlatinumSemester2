package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Pixel;
import internal.Helper.Vector;
import static java.lang.Float.max;
import static java.lang.Float.valueOf;
import static java.lang.Math.PI;
import static java.lang.Math.min;
import static java.lang.Math.pow;

/**
 * Taxiing Controller
 */
public class AutopilotTaxiingController extends Controller {


    public AutopilotTaxiingController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {

        this.setCurrentInputs(inputs);

        AutopilotInputs_v2 currentInputs = getCurrentInputs();
        AutopilotInputs_v2 previousInputs = getPreviousInputs();
        ControlOutputs outputs = new ControlOutputs();
        Vector velocity = getVelocityApprox(previousInputs,currentInputs);
        Vector orientation = Controller.extractOrientation(inputs);
        Vector position = Controller.extractPosition(inputs);
        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));



        //the z coordinate of the location the drone has to go to (has to be changed to something with the path)
        float desiredZValue = 30;
        //the x coordinate of the location the drone has to go to
        float desiredXValue = 0;

        if (!hasToTurn(desiredXValue,desiredZValue,orientation,position)){
                goForward(velocity, outputs);
        }
        else{
            uTurn(outputs);
        }
        //check if the drone has reached the desired Z position and break if he has
        if (Math.abs(desiredZValue) - Math.abs(position.getzValue()) < Math.abs(velocity.getzValue())) {
            setBrakeForce(outputs, 800,800,800);
            outputs.setThrust(0);
        }



        //graveyard of things that didn't work either
      /*  if (desiredZValue < 0) {

            if (Math.abs(desiredZValue) - Math.abs(position.getzValue()) < Math.abs(velocity.getzValue())) {
                if (Controller.extractOrientation(inputs).getxValue() > 0) {
                    UTurn(outputs);
                    System.out.println("a");

                } else if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
                //    setBrakeForce(outputs, 1000);
                    outputs.setThrust(0);
                    System.out.println("b");

                } else {
                    goForward(velocity,outputs);
                    System.out.println("c");

//                    stabillizeOrientation(outputs, (float) -PI, Controller.extractOrientation(inputs));
                }

            }
        }

        if (desiredZValue > 0) {
            if (Controller.extractOrientation(inputs).getxValue() > -PI) {
              //  UTurn(outputs);
                System.out.println("a");
                turnSouth(outputs);

            }
            else if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
               // setBrakeForce(outputs, 1000);
                outputs.setThrust(0);
                System.out.println("b");

            }
            else{
            //    goForward(velocity,outputs);
                System.out.println("c");

                stabillizeOrientation(outputs,(float)-PI,Controller.extractOrientation(inputs));
            }

        }
*/


      //  System.out.println("x: "+Controller.extractPosition(inputs).getxValue());
        //System.out.println("z: "+Controller.extractPosition(inputs).getzValue());

        //System.out.println("velocity " + velocity);
        System.out.println("orientation " + Controller.extractOrientation(inputs));


        return outputs;
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        return false;
    }

    /**
     * Check if the drone has to turn to reach his destination
     * @param desiredXValue
     * @param desiredZValue
     * @param orientation
     * @param currPos
     * @return
     */
    private boolean hasToTurn(float desiredXValue, float desiredZValue, Vector orientation, Vector currPos){
        //error allowed on the orientation
        float delta = 0.25f;
        //error allowed on the x position
        float delta2 = 0.5f;

        //if the current x position is (almost) correct the drone doesn't have to turn
        if (desiredXValue > currPos.getxValue()-delta2 && desiredXValue < currPos.getxValue()+delta2) {
            //if the the desired z position is smaller than the current and the x orientation is (almost) 0, the drone just has to drive straight forward
            if (desiredZValue < currPos.getzValue()) {
                if (orientation.getxValue() > 0 - delta && orientation.getxValue() < 0 + delta) {
                    return false;
                }
            }
            //if the the desired z position is bigger than the current and the x orientation is (almost) PI, the drone just has to drive straight forward
            if (desiredZValue < currPos.getzValue()) {
                if (Math.abs(orientation.getxValue()) > PI - delta && Math.abs(orientation.getxValue()) < PI + delta) {
                    return false;
                }
            }
        }


        return true;
    }

    /**
     * try to stabilize around an orientation
     * @param outputs
     * @param stabilizeAround
     * @param orientation
     */
    private void stabillizeOrientation(ControlOutputs outputs, float stabilizeAround, Vector orientation){
        if (stabilizeAround > orientation.getxValue()){
            setBrakeForce(outputs,0,100,0);
            outputs.setVerStabInclination(0);
            outputs.setThrust(200);
            System.out.println("l");

        }
        if (stabilizeAround < orientation.getxValue()){
            setBrakeForce(outputs,0,0,100);
            outputs.setVerStabInclination(0);
            outputs.setThrust(200);
            System.out.println("r");

        }
    }

    /**
     * set the force on all 3 breaks (front, left, right)
     * @param outputs
     * @param front
     * @param left
     * @param right
     */
    private void setBrakeForce(ControlOutputs outputs, float front, float left, float right){
        outputs.setFrontBrakeForce(front);
        outputs.setLeftBrakeForce(left);
        outputs.setRightBrakeForce(right);
    }


/*    private void turnNorth(ControlOutputs outputs){
        outputs.setFrontBrakeForce(0);
        outputs.setLeftBrakeForce(0);
        outputs.setRightBrakeForce(300);
        outputs.setVerStabInclination( (float) PI/3);
        outputs.setThrust(400);
    }*/


/*    private void turnSouth(ControlOutputs outputs){
        outputs.setFrontBrakeForce(0);
        outputs.setLeftBrakeForce(300);
        outputs.setRightBrakeForce(0);
        outputs.setVerStabInclination( (float) PI/3);
        outputs.setThrust(400);
    }*/

    /**
        turn 180°
     */
    private void uTurn(ControlOutputs outputs){
        setBrakeForce(outputs,0,0,300);
        outputs.setVerStabInclination( (float) PI/3);
        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
        outputs.setThrust(maxThrust/3);
    }


    /**
     * go forward at a max speed of 10m/s
     * @param velocity
     * @param outputs
     */
    private void goForward(Vector velocity, ControlOutputs outputs){
        outputs.setRightWingInclination(0);
        outputs.setLeftWingInclination(0);
        outputs.setHorStabInclination(0);
        outputs.setVerStabInclination(0);
        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));

        if (velocity1D > TAXI_VELOCITY) {
            outputs.setThrust(0);
            setBrakeForce(outputs, 50f, 50f, 50f);
        }
        else{
            setThrust(outputs);
            setBrakeForce(outputs,0f,0f,0f);
        }
    }



    private void setThrust(ControlOutputs outputs) {
        //get the maximal thrust
        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();
        //elapsed time:
        float elapsedTime = this.getCurrentInputs().getElapsedTime();
        //first get an approx of the current velocity
        Vector velocity = this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs());
        //then get a response from the PID
        Vector velocityPID = this.getVelocityPID().getPIDOutput(velocity, elapsedTime);
        //use the PID output for the corrective action (gives the error on the velocity)
        //we we take the desired output thrust (for stable config) if the velocity is too high, pull back, to low go faster
        //System.out.println("velocityPID: " + velocityPID);
        float outputThrust = (10 - (TAXI_VELOCITY + velocityPID.getzValue()) / TAXI_VELOCITY) * STANDARD_THRUST;
        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));
    }



    /**
     * Constants
     */
    private final static float TAXI_VELOCITY = 10.0f;
    private final static float STANDARD_THRUST = 128.41895f * 3.5f;
    private final static float WING_INCL = (float) (PI / 180);
    private final static float HORIZONTAL_STABILIZER = (float) (PI / (180));
    private final static float PITCH_THRESHOLD = (float) (5 * PI / 180);
    private static final float INIT_LANDING_HEIGHT = 8f;
    private final static float MAIN_STABLE = (float) (5*PI/180);
    private final static float STABILIZER_STABLE = 0;
    private final static float ROLL_THRESHOLD = (float)(3*PI/180);
    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);
    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);
    private final static float PLANE_HEIGHT_FROM_GROUND = 1.2f;

    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);

    private VectorPID getVelocityPID() {
        return this.velocityPID;
    }


    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE;
    }

    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return INCLINATION_AOA_ERROR_MARGIN;
    }

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }




}
