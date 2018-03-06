package internal.Autopilot;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;
import internal.Helper.Vector;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Martijn on 18/02/2018.
 * A class of landing controllers, responsible for controlling the landing of the drone
 */
public class AutopilotLandingController extends Controller{

    public AutopilotLandingController(AutoPilot autopilot){
        // implement constructor
        super(autopilot);
    }

    /**
     * Generates the control actions for the autopilot
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs inputs){
    	// generate path
    	AutopilotInputs currentInputs = getCurrentInputs();
    	AutopilotInputs previousInputs = getPreviousInputs();

    	Vector position = new Vector(currentInputs.getX(),currentInputs.getY(),currentInputs.getZ());
    	Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
    	Vector destination = this.getAutopilot().getStartPosition();
    	
    	
    	
    }
    
 
    //TODO implement these methods accordingly
    @Override
    protected float getMainStableInclination() {
        return 0;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return 0;
    }

    @Override
    protected float getRollThreshold() {
        return 0;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return 0;
    }

    @Override
    protected float getStandardThrust() {
        return 0;
    }
<<<<<<< HEAD
    
    
=======

    private static final float DISTANCE_BETWEEN_LANDING_BLOCKS = 10f;
    private static final float LANDING_ANGLE = (float)Math.PI/12;
    private static final float STEEPEST_TURN_DIAMETER = 10;


    private List<Vector> path = new ArrayList<>();

>>>>>>> 69749692db1065c121e79d414e0fbd20cb74462d
}
