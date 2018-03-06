package internal.Autopilot;

import java.util.ArrayList;
import java.util.List;

import internal.Helper.Vector;

/**
 * Class for generating (landing) paths
 * @author Anthony Rathe
 *
 */
public class PathGenerator {
	
	public PathGenerator() {
		pathUnlock();
	}
	
	private void pathLock() {
		pathLock = true;
	}
	
	public void pathUnlock() {
		pathLock = false;
	}
	
	public boolean hasPathLocked() {
		return pathLock;
	}
	
	public List<Vector> generateLandingPath(Vector position, Vector velocity, Vector destination){
		Vector landingVector = getLandingVector(position, velocity, destination);
    	
    	if (canMakeTurn(position, velocity, destination)) {
    		
    		
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
    		float additionalDistance = forwardDistanceBetween(position, velocity, lastBlock);
    		float prolongedDistance = 0;
    		Vector prolongVector = velocity.makeHorizontal().normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    		if (additionalDistance > 0) {
    			// path between exit of semi-circle and start of descent should be prolonged
    			while (prolongedDistance < additionalDistance) {
    				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
    				lastBlock = lastBlock.vectorSum(prolongVector);
    				blockCoordinates.add(lastBlock);
    			}
    		}
    		
    		// generate semi-circle
    		float circleDiameter = parallelDistanceBetween(position, velocity, destination);
    		
    		// generate additional path
    		if (additionalDistance < 0) {
    			// path before start of semi-circle should be prolonged
    			while (prolongedDistance < additionalDistance - DISTANCE_BETWEEN_LANDING_BLOCKS) {
    				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
    				lastBlock = lastBlock.vectorSum(prolongVector.scalarMult(-1));
    				blockCoordinates.add(lastBlock);
    			}
    		}
    		
    		
    		
    	}else {
    		// get the drone further away from the destination in order to be able to turn
    		
    		if (position.makeHorizontal().distanceBetween(getLandingStartPosition(position, parallelHorizontalVector(position, destination), destination).makeHorizontal()) >= STEEPEST_TURN_DIAMETER) {
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
	    	return parallelDistanceBetween(position, direction, getLandingStartPosition(position, direction, destination)) >= STEEPEST_TURN_DIAMETER;
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
	    
	    private Vector getLandingStartPosition(Vector position, Vector direction, Vector destination) {
	    	Vector landingVector = getLandingVector(position, direction, destination);
	    	
	    	Vector lastBlock = destination;
    		while(lastBlock.getyValue() < position.getyValue()) {
    			lastBlock = lastBlock.vectorSum(landingVector);
    			if (lastBlock.getyValue() >= position.getyValue()) {
    				lastBlock = new Vector(lastBlock.getxValue(),position.getzValue(),lastBlock.getzValue());
    			}
    		}
    		return lastBlock;
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
	    
	    private static Vector parallelHorizontalVector(Vector position1, Vector position2) {
	    	position1 = position1.makeHorizontal();
	    	position2 = position2.makeHorizontal();
	    	
	    	Vector distanceBetween = position1.vectorDifference(position2);
	    	return distanceBetween.rotateAroundYAxis((float)Math.PI/2);
	    	
	    }
	   
	
	private static final float DISTANCE_BETWEEN_LANDING_BLOCKS = 10f;
    private static final float LANDING_ANGLE = (float)Math.PI/12;
    private static final float STEEPEST_TURN_DIAMETER = 10;
    
    private boolean pathLock;
}
