package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;

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
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs){
    	
    	AutopilotInputs_v2 currentInputs = getCurrentInputs();
    	AutopilotInputs_v2 previousInputs = getPreviousInputs();

    	if (!getPathGenerator().hasPathLocked()) {
    		Vector position = new Vector(currentInputs.getX(),currentInputs.getY(),currentInputs.getZ());
        	Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
        	Vector destination = this.getAutopilot().getStartPosition();
        	
        	getPathGenerator().generateLandingPath(position, velocityApprox, destination);
    	}
    	
    	List<Vector> path = getPathGenerator().getPath();
    	
    	
    	return null;
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
}