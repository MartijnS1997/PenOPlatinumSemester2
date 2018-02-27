package internal.Autopilot;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;

/**
 * Created by Martijn on 19/02/2018.
 * A flight controller made for the 15° AOA assignment
 * TODO: Implement the controller fully
 */
public class GammaFlightController extends AutoPilotFlightController {

    public GammaFlightController(AutoPilot autoPilot){
        super(autoPilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs inputs) {
    	
    	// If all blocks were hit, start landingprocedure
    	AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
        APCamera.loadNewImage(inputs.getImage());
        int amountOfCubesInSight = APCamera.getCubesInPicture().size();
        
        if (amountOfCubesInSight <= 0) {
        	this.getAutopilot().setAPMode(3);
        }
        return null;
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
}
