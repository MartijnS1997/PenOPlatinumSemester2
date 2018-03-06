package internal.Autopilot;

import java.util.ArrayList;
import java.util.List;

import Autopilot.AutopilotInputs;
import Autopilot.AutopilotOutputs;
import internal.Helper.Vector;

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
    	
    	
    	Vector landingVector = getLandingVector(position, velocityApprox, destination);
    	
    	if (canMakeTurn(position, velocityApprox, destination)) {
    		
    		
    		// generate semi-circle
    		List<Vector> blockCoordinates = new ArrayList<Vector>();
    		
    		// generate landing path
    		Vector lastBlock = destination;
    		while(lastBlock.getyValue() < position.getyValue()) {
    			lastBlock = lastBlock.vectorSum(landingVector);
    			if (lastBlock.getyValue() >= position.getyValue()) {
    				lastBlock = new Vector(lastBlock.getxValue(),position.getzValue(),lastBlock.getzValue());
    			}
    			blockCoordinates.add(lastBlock);
    		}
    		
    		// generate additional path
    		float additionalDistance = forwardDistanceBetween(position, velocityApprox, destination);
    		float prolongedDistance = 0;
    		Vector prolongVector = velocityApprox.makeHorizontal().normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    		if (additionalDistance >= 0) {
    			// path between exit of semi-circle and start of descent should be prolonged
    			while (prolongedDistance < additionalDistance) {
    				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
    				lastBlock = lastBlock.vectorSum(prolongVector);
    			}
    		}
    		
    		// generate semi-circle
    		Vector circleCenter = Vector()
    		
    		// generate additional path
    		if (additionalDistance < 0) {
    			// path before start of semi-circle should be prolonged
    			while (prolongedDistance < additionalDistance - DISTANCE_BETWEEN_LANDING_BLOCKS) {
    				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
    				lastBlock = lastBlock.vectorSum(prolongVector.scalarMult(-1));
    			}
    		}
    		
    		
    		
    	}else {
    		// get the drone further away from the destination in order to be able to turn
    		
    		if (position.makeHorizontal().distanceBetween(destination.makeHorizontal()) >= STEEPEST_TURN_DIAMETER) {
    			// if distance between drone and destination is large enough to make a turn, start aligning
        		
    		}else {
    			// if distance between drone and destination is too small to allow for a proper turn, keep on flying away from destination
    	    	
    		}
    		
    		}
    	
    	// 
        return null;
    }
    
    private Vector getLandingVector(Vector position, Vector direction, Vector destination) {
    	Vector horizontalDirection = direction.makeHorizontal();
    	// Formula: y = sqrt(x^2+z^2)*tan(landingAngle)
    	return new Vector(horizontalDirection.getxValue(),(float)(Math.sqrt(Math.pow(horizontalDirection.getxValue(),2)+Math.pow(horizontalDirection.getzValue(),2))*Math.tan(LANDING_ANGLE)),horizontalDirection.getzValue()).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    }
    
    private boolean canMakeTurn(Vector position, Vector direction, Vector destination) {
    	return parallelDistanceBetween(position, direction, destination) >= STEEPEST_TURN_DIAMETER;
    }

    /**
     * Returns the shortest distance between a vector with direction "direction", placed in 
     * position, and a vector with direction "direction", placed in destination.
     * @param position
     * @param direction
     * @param destination
     * @author Anthony Rathe
     */
    private static float parallelDistanceBetween(Vector position, Vector direction, Vector destination) {
    	Vector horizontalDirection = direction.makeHorizontal();
    	Vector horizontalPosition = position.makeHorizontal();
    	Vector horizontalDestination = destination.makeHorizontal();
    	Vector dMinusP = horizontalDestination.vectorDifference(horizontalPosition);
    	
    	// Formula: d = (destinationVector - positionVector).size()*cos(angleBetween(destinationVector - positionVector,directionVector)-PI/2)
    	return (float)(dMinusP.getSize()*Math.cos(dMinusP.getAngleBetween(horizontalDirection)-Math.PI/2f));
    }
    
    /**
     * Return the distance a vector "direction" should travel horizontally, starting from "destination", to arrive
     * in a parallel position, next to "position".
     * @param position
     * @param direction
     * @param destination
     * @author Anthony Rathe
     */
    private static float forwardDistanceBetween(Vector position, Vector direction, Vector destination) {
    	Vector horizontalDirection = direction.makeHorizontal();
    	Vector horizontalPosition = position.makeHorizontal();
    	Vector horizontalDestination = destination.makeHorizontal();
    	Vector dMinusP = horizontalDestination.vectorDifference(horizontalPosition);
    	
    	// Formula: d = (destinationVector - positionVector).size()*cos(angleBetween(destinationVector - positionVector,directionVector)-PI/2)
    	float alfa = (float)(dMinusP.getAngleBetween(horizontalDirection)-Math.PI/2f);
    	float d = (float)(dMinusP.getSize()*Math.sin(alfa));
    	return Math.signum(alfa)*d;
    	
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
    
    private static final float DISTANCE_BETWEEN_LANDING_BLOCKS = 10f;
    private static final float LANDING_ANGLE = (float)Math.PI/12;
    private static final float STEEPEST_TURN_DIAMETER = 10;
}
