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
        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));


     //   goForward(velocity, outputs);


        float desiredZValue = -10;
        float desiredXValue = 20;

        if (desiredZValue < 0) {

             if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
                    setBrakeForce(outputs, 1000);
                    outputs.setThrust(0);
            }
        }


        if (desiredZValue > 0) {
            if (Controller.extractOrientation(inputs).getxValue() > -PI) {
                UTurn(outputs);
                System.out.println("a");

            }
            else if (Controller.extractOrientation(inputs).getxValue() > -PI){
                System.out.println("d");
                turnSouth(outputs);
            }
            else if (Math.abs(desiredZValue) - Math.abs(Controller.extractPosition(inputs).getzValue()) < Math.abs(velocity.getzValue())) {
                setBrakeForce(outputs, 100);
                outputs.setThrust(0);
                System.out.println("b");

            }
            else{
                goForward(velocity,outputs);
                System.out.println("c");
                /*outputs.setThrust(0);
                outputs.setVerStabInclination(0);
                outputs.setHorStabInclination(0);
                outputs.setLeftBrakeForce(0);
                outputs.setRightBrakeForce(100);
                outputs.setFrontBrakeForce(100);
*/
            }

        }
      //  if (desiredZValue )

/*        boolean intervaal = Controller.extractOrientation(inputs).getxValue() < -1.46f &&
                Controller.extractOrientation(inputs).getxValue() > -1.6f;
        if (desiredXValue > Controller.extractPosition(inputs).getxValue() && intervaal){
                System.out.println(outputs.getThrust());
                if (Controller.extractOrientation(inputs).getxValue() < -PI/2){
             //       turnSouth(outputs);
                  goForward(velocity,outputs);

                    System.out.println("a");
                }

                if (Controller.extractOrientation(inputs).getxValue() > -PI/2) {
                  //  turnNorth(outputs);
                    goForward(velocity,outputs);
                    System.out.println("b");

                }
            }
               // goForward(velocity, outputs);
        else {
            turnNorth(outputs);
            System.out.println("c");

        }*/


      //  System.out.println("x: "+Controller.extractPosition(inputs).getxValue());
        System.out.println("z: "+Controller.extractPosition(inputs).getzValue());

        //System.out.println(velocity);
        System.out.println("velocity " + velocity);
    //    System.out.println("orientation " + Controller.extractOrientation(inputs));


        return outputs;
    }



    private void setBrakeForce(ControlOutputs outputs, float brakeForce){
        outputs.setFrontBrakeForce(brakeForce);
        outputs.setLeftBrakeForce(brakeForce);
        outputs.setRightBrakeForce(brakeForce);
    }


    private void turnNorth(ControlOutputs outputs){
        outputs.setFrontBrakeForce(0);
        outputs.setLeftBrakeForce(0);
        outputs.setRightBrakeForce(100);
        outputs.setVerStabInclination( (float) PI/3);
        outputs.setThrust(500);
    }


    private void turnSouth(ControlOutputs outputs){
        outputs.setFrontBrakeForce(0);
        outputs.setLeftBrakeForce(300);
        outputs.setRightBrakeForce(0);
        outputs.setVerStabInclination( (float) PI/3);
        outputs.setThrust(500);
    }

    private void UTurn(ControlOutputs outputs){
        outputs.setFrontBrakeForce(0);
        outputs.setLeftBrakeForce(0);
        outputs.setRightBrakeForce(300);
        outputs.setVerStabInclination( (float) PI/3);
        outputs.setThrust(400);
    }

    private void goForward(Vector velocity, ControlOutputs outputs){
        outputs.setRightWingInclination(0);
        outputs.setLeftWingInclination(0);
        outputs.setHorStabInclination(0);
        outputs.setVerStabInclination(0);
        float velocity1D = (float) Math.sqrt(pow(velocity.getzValue(),2)+pow(velocity.getxValue(),2));

        if (velocity1D > TAXI_VELOCITY) {
            outputs.setThrust(0);
            setBrakeForce(outputs, 50f);
        }
        else{
            setThrust(outputs);
            setBrakeForce(outputs,0f);
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
