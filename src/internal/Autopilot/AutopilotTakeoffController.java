package internal.Autopilot;

import static java.lang.Math.PI;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;
import internal.Helper.Vector;

/**
 * Created by Martijn on 18/02/2018.
 * A class of takeoff controllers, responsible for controlling the takeoff of the drone
 */
public class AutopilotTakeoffController extends Controller{

    public AutopilotTakeoffController(AutoPilot autopilot){
        //implement constructor
        super(autopilot);
    }

    /**
     * Generates the control actions for the autopilot
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs inputs){
    	
    	setCurrentInputs(inputs);
    	
    	ControlOutputs controlOutputs = new ControlOutputs();
    	
    	AutopilotInputs currentInputs = getCurrentInputs();
    	AutopilotInputs previousInputs = getPreviousInputs();
    	
    	float currentHeight = currentInputs.getY();
    	
    	Vector velocityApprox = this.getVelocityApprox(previousInputs, currentInputs);
    	
    	if (currentHeight < STOP_TAKEOFF_HEIGHT) {
    		// Still on the ground
    		
    		// Set max thrust
    		controlOutputs.setThrust(this.getAutopilot().getConfig().getMaxThrust());
    		
    		if (velocityApprox.getyValue() >= LIFTOFF_THRESHOLD ) {
    			// Drone is lifting off
    			
    			if (currentInputs.getPitch() <= TAKEOFF_PITCH) {
    				// Start ascending
    				controlOutputs.setHorStabInclination(-STANDARD_INCLINATION);
    			}else if(currentInputs.getPitch() >= MAX_PITCH){
    				// Start descending
    				controlOutputs.setHorStabInclination(STANDARD_INCLINATION);
    			}else {
    				// Stop ascending/descending
    				controlOutputs.setHorStabInclination(0f);
    			}
    			
    		}
    		
    	}else {
    		// In mid-air
    		// Turn to flight mode
    		this.getAutopilot().setAPMode(2);
    		
    	}
        return controlOutputs;
    }

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
    
    private static final float LIFTOFF_THRESHOLD = 1f;
    private static final float STOP_TAKEOFF_HEIGHT = 10f;
    private static final float TAKEOFF_PITCH = (float)Math.PI/18f;
    private static final float MAX_PITCH = (float)Math.PI/4f;
    private static final float STANDARD_INCLINATION = (float) PI/12;
}
