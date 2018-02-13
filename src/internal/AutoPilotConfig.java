package internal;

import Autopilot.AutopilotConfig;

/**
 * 
 * @author Anthony Rathe
 *
 */
public class AutoPilotConfig implements AutopilotConfig{
	public AutoPilotConfig(float gravity, float wingX, float tailSize, float engineMass, float wingMass, float tailMass, float maxThrust, float maxAOA, float wingLiftSlope, float horStabLiftSlope, float verStabLiftSlope, float horizontalAngleOfView, float verticalAngleOfView, int nbColumns, int nbRows) {
		this.gravity = gravity;
		this.wingX = wingX;
		this.tailSize = tailSize;
		this.engineMass = engineMass;
		this.wingMass = wingMass;
		this.tailMass = tailMass;
		this.maxThrust = maxThrust;
		this.maxAOA = maxAOA;
		this.wingLiftSlope = wingLiftSlope;
		this.horStabLiftSlope = horStabLiftSlope;
		this.verStabLiftSlope = verStabLiftSlope;
		this.horizontalAngleOfView = horizontalAngleOfView;
		this.verticalAngleOfView = verticalAngleOfView;
		this.nbColumns = nbColumns;
		this.nbRows = nbRows;
	}
	
	private final float gravity;
	private final float wingX;
	private final float tailSize;
	private final float engineMass;
	private final float wingMass;
	private final float tailMass;
	private final float maxThrust;
	private final float maxAOA;
	private final float wingLiftSlope;
	private final float horStabLiftSlope;
	private final float verStabLiftSlope;
	private final float horizontalAngleOfView;
	private final float verticalAngleOfView;
	private final int nbColumns;
	private final int nbRows;
	
	public float getGravity() {
		return this.gravity;
	}
	
    public float getWingX() {
    	return this.wingX;
    }
    
    public float getTailSize() {
    	return this.tailSize;
    }
    
    public float getEngineMass() {
    	return this.engineMass;
    }
    
    public float getWingMass() {
    	return this.wingMass;
    }
    
    public float getTailMass() {
    	return this.tailMass;
    }
    
    public float getMaxThrust() {
    	return this.maxThrust;
    }
    
    public float getMaxAOA() {
    	return this.maxAOA;
    }
    
    public float getWingLiftSlope() {
    	return this.wingLiftSlope;
    }
    
    public float getHorStabLiftSlope() {
    	return this.horStabLiftSlope;
    }
    
    public float getVerStabLiftSlope(){
    	return this.verStabLiftSlope;
    }
    
    public float getHorizontalAngleOfView() {
    	return this.horizontalAngleOfView;
    }
    
    public float getVerticalAngleOfView() {
    	return this.verticalAngleOfView;
    }
    
    public int getNbColumns() {
    	return this.nbColumns;
    }
    
    public int getNbRows() {
    	return this.nbRows;
    }
}
