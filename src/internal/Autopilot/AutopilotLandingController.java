
package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;


import static java.lang.Math.*;

import java.util.List;

/**
 * Created by Martijn on 18/02/2018, extended by Jonathan on 12/3/2018
 * A class of landing controllers, responsible for controlling the landing of the drone
 */

public class AutopilotLandingController extends Controller {

    public AutopilotLandingController(AutoPilot autopilot) {
        // implement constructor
        super(autopilot);
        this.getVelocityPID().setSetPoint(this.referenceVelocity);
        this.getOrientationPID().setSetPoint(this.referenceOrientation);
        this.getAltitudePID().setSetPoint(this.referenceAltitude);

    }


    /**
     * Generates the control actions for the autopilot
     *
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        this.setCurrentInputs(inputs);


        // generate path
/*        AutopilotInputs currentInputs = getCurrentInputs();
        AutopilotInputs previousInputs = getPreviousInputs();*/
        ControlOutputs outputs = new ControlOutputs();


/*        Vector position = new Vector(currentInputs.getX(), currentInputs.getY(), currentInputs.getZ());
        Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
        Vector destination = this.getAutopilot().getStartPosition();*/

        //start going down
        loseAltitude(outputs);

        pitchControl(outputs);

        //under INIT_LANDING_HEIGHT start stabilizing the plane
        if (Controller.extractPosition(inputs).getyValue() <= INIT_LANDING_HEIGHT) {
            //activate the breaks on the wheels when plane hits the ground
            if (Controller.extractPosition(inputs).getyValue() <= PLANE_HEIGHT_FROM_GROUND) {
                outputs.setRightBrakeForce(100);
                outputs.setLeftBrakeForce(100);
                outputs.setFrontBrakeForce(100);
                System.out.println(Controller.extractPosition(inputs).getzValue());
            }
            this.stayDown(outputs);
            this.setThrust(outputs);
            this.setHorizontalStabilizer(outputs);

        }
     // System.out.println(Controller.extractPosition(inputs).getyValue());


        return outputs;
    }

    private void loseAltitude(ControlOutputs outputs) {
        float outputThrust  = this.getAutopilot().getMaxThrust();
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(WING_INCL);
        outputs.setLeftWingInclination(WING_INCL);
        outputs.setHorStabInclination(HORIZONTAL_STABILIZER);
    }

    private void stayDown(ControlOutputs outputs) {
        float outputThrust  = this.getAutopilot().getMaxThrust();
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(-WING_INCL);
        outputs.setLeftWingInclination(-WING_INCL);
        outputs.setHorStabInclination(HORIZONTAL_STABILIZER/2);
    }


    private void pitchControl(ControlOutputs outputs) {
        float pitch = Controller.extractOrientation(this.getCurrentInputs()).getyValue();
        if (abs(pitch) >= PITCH_THRESHOLD) {
            outputs.setHorStabInclination(0f);
        }
    }


    private void setThrust(ControlOutputs outputs) {
 /*       //get the maximal thrust
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
        float outputThrust = (2 - (STOP_VELOCITY + velocityPID.getzValue()) / STOP_VELOCITY) * STANDARD_THRUST;
        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));*/
        outputs.setThrust(0f);

    }


    private void setHorizontalStabilizer(ControlOutputs outputs){
        //we want to go for zero (stable inclination of the horizontal stabilizer is zero), so the corrective action needs also to be zero
        Vector orientation = Controller.extractOrientation(this.getCurrentInputs());
        Vector orientationPID = this.getOrientationPID().getPIDOutput(orientation, this.getCurrentInputs().getElapsedTime());
        //extract the pitch (positive is upward looking, negative is downward looking)
        float pitch = orientationPID.getyValue();
        float pitchConstant = 2;
        //calculate the desired action of the stabilizer(negative is upward movement, positive downward)
        float desiredAngle = (float) (-pitch/PI*pitchConstant);//the pitch/PI is to get a % of the pitch that we're off
        float outputInclination = min(abs(HOR_STABILIZER_MAX), abs(desiredAngle));
        outputs.setHorStabInclination(outputInclination*signum(desiredAngle));
    }



    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
    private final static float STOP_VELOCITY = 0.0f;
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



    private Vector referenceVelocity = new Vector(0,0,-STOP_VELOCITY);
    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);
    private Vector referenceOrientation = new Vector();
    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);
    private float referenceAltitude = 10f;

    private VectorPID getVelocityPID() {
        return this.velocityPID;
    }

    public VectorPID getOrientationPID() {
        return orientationPID;
    }

    public PIDController getAltitudePID() {
        return altitudePID;
    }


    //TODO implement these methods accordingly
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


