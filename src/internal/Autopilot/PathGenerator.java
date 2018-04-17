package internal.Autopilot;

import java.util.ArrayList;
import java.util.Collections;
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

	public List<Vector> getPath(){
		return this.path;
	}

	private void setPath(List<Vector> path) {
		this.path = path;
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
	
	public void generateLandingPath(Vector position, Vector orientation, Vector goalPosition, Vector goalOrientation) {
		
		
		
	}
	
	public void beamSearch(Vector position, Vector orientation, Vector goalPosition, Vector goalOrientation) {
		// Make sure we are working in the horizontal plane
		position = position.makeHorizontal();
		orientation = orientation.makeHorizontal();
		goalPosition = goalPosition.makeHorizontal();
		goalOrientation = goalOrientation.makeHorizontal();
		
		
		boolean pathFound = false;
		int beamWidth = 1;
		while (!pathFound) {
			// INIT
			List<List<Vector>> queue = new ArrayList<List<Vector>>();
			List<Vector> startingPath = new ArrayList<Vector>();
			startingPath.add(position);
			queue.add(startingPath);
			
			while(queue.size()>0 && !pathFound) {
				queue = expandQueue(queue, orientation, goalPosition, goalOrientation);
				queue = selectBestPaths(queue, beamWidth, orientation, goalPosition, goalOrientation);
				int index;
				if ((index = finalPathIndex(queue, orientation, goalPosition, goalOrientation)) != -1)return queue.get(index);
			}
			
			beamWidth++;
		}
		
	}
	
	public int finalpathIndex(List<List<Vector>> queue, Vector originalOrientation, Vector goalPosition, Vector goalOrientation) {
		int size = queue.size();
		for (int i = 0; i < size; i++) {
			List<Vector> path = queue.get(i);
			if (path.size() > 0) {
				if (hasReachedDestination(path, originalOrientation, goalPosition, goalOrientation))return i;
			};
			
		}
		return -1;
	}
	
	public boolean hasReachedDestination(List<Vector> path, Vector originalOrientation, Vector goalPosition, Vector goalOrientation) {
		Vector currentPosition = path.get(path.size()-1);
		Vector currentOrientation = getCurrentOrientation(path, originalOrientation);
		
		float angle = goalOrientation.getAngleBetween(currentOrientation);
		float distance = goalPosition.distanceBetween(currentPosition);
		if (angle <= ANGLE_MARGIN_OF_ERROR) {
			
		}
	}
	
	public List<List<Vector>> expandQueue(List<List<Vector>> originalQueue, Vector originalOrientation, Vector goalPosition, Vector goalOrientation){
		List<List<Vector>> expandedQueue = new ArrayList<List<Vector>>();
		for (List<Vector> path : originalQueue) {
			Vector currentPosition = path.get(path.size()-1);
			Vector currentOrientation = getCurrentOrientation(path, originalOrientation);
			float originalHeuristic = generateHeuristic(path, originalOrientation, goalPosition, goalOrientation);
			
			List<List<Vector>> extensions = getPieces(currentPosition, currentOrientation, goalPosition, goalOrientation);
			for (List<Vector> extension : extensions) {
				List<Vector> extendedPath = new ArrayList<Vector>();
				extendedPath.addAll(path);
				extendedPath.addAll(extension);
				
				// Only add path when heuristic is improved
				if(generateHeuristic(extendedPath, originalOrientation, goalPosition, goalOrientation) < originalHeuristic)expandedQueue.add(extendedPath);
			}
			
		}
		
		return expandedQueue;
	}
	
	public List<List<Vector>> selectBestPaths(List<List<Vector>> queue, int beamWidth, Vector originalOrientation, Vector goalPosition, Vector goalOrientation){

		List<List<Vector>> bestPaths = new ArrayList<List<Vector>>();
		if (beamWidth < 1)return bestPaths;
		
		List<Vector> worstPath = new ArrayList<Vector>();
		float worstHeuristic = Float.POSITIVE_INFINITY;
		for (List<Vector> path : queue) {
			float heuristic = generateHeuristic(path, originalOrientation, goalPosition, goalOrientation);

			if (bestPaths.size() < beamWidth) {
				// Add path when bestPaths isn't full yet
				bestPaths.add(path);
				worstPath = selectWorstPath(bestPaths, originalOrientation, goalPosition, goalOrientation);
				worstHeuristic = generateHeuristic(worstPath, originalOrientation, goalPosition, goalOrientation);
			}else {
				// Only add path when it has a better heuristic than the worst in bestPaths
				if (heuristic < worstHeuristic) {
					bestPaths.remove(worstPath);
					bestPaths.add(path);
					worstPath = selectWorstPath(bestPaths, originalOrientation, goalPosition, goalOrientation);
					worstHeuristic = generateHeuristic(worstPath, originalOrientation, goalPosition, goalOrientation);
				}
			}
			
		}
		
		return bestPaths;
	}
	
	public List<Vector> selectWorstPath(List<List<Vector>> paths, Vector originalOrientation, Vector goalPosition, Vector goalOrientation){
		float worstHeuristic = -1;
		List<Vector> worstPath = new ArrayList<Vector>();
		
		for (List<Vector> path : paths) {
			float heuristic = generateHeuristic(path, originalOrientation, goalPosition, goalOrientation);
			if (heuristic > worstHeuristic) {
				worstPath = path;
				worstHeuristic = heuristic;
			}
		}
		
		return worstPath;
	}
	
	public List<List<Vector>> getPieces(Vector position, Vector orientation, Vector goalPosition, Vector goalOrientation){
		List<List<Vector>> pieceList = new ArrayList<List<Vector>>();
		pieceList.addAll(getGenericPieces(orientation));
		pieceList.addAll(getAdditionalPieces(position, orientation, goalPosition, goalOrientation));
		return pieceList;
	}
	
	public List<List<Vector>> getGenericPieces(Vector orientation){
		// Make sure we are working in the horizontal space
		Vector baseVector = orientation.makeHorizontal().normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
		
		int nbOfVectors = (int)Math.floor(NB_OF_GENERIC_VECTORS/2f);
		float angle = MAX_TURN_ANGLE*2f/nbOfVectors;
		List<List<Vector>> pieceList = new ArrayList<List<Vector>>();
		
		List<Vector> piece = new ArrayList<Vector>();
		Vector pieceVector = baseVector;
		piece.add(pieceVector);
		pieceList.add(piece);
		
		for (float i = 1; i <= nbOfVectors; i++) {
			piece = new ArrayList<Vector>();
			pieceVector = baseVector.rotateAroundYAxis(angle*i);
			piece.add(pieceVector);
			pieceList.add(piece);
			
			piece = new ArrayList<Vector>();
			pieceVector = baseVector.rotateAroundYAxis(-angle*i);
			piece.add(pieceVector);
			pieceList.add(piece);
		}
		
		return pieceList;
	}
	
	public List<List<Vector>> getAdditionalPieces(Vector position, Vector orientation, Vector goalPosition, Vector goalOrientation){
		// Make sure we are working in the horizontal space
		position = position.makeHorizontal();
		orientation = orientation.makeHorizontal();
		goalPosition = goalPosition.makeHorizontal();
		goalOrientation = goalOrientation.makeHorizontal();
		
		List<List<Vector>> pieceList = new ArrayList<List<Vector>>();
		
		// Add the current orientation, goal orientation and straight path to goal to additional vectors
		List<Vector> straightPathList = new ArrayList<Vector>();
		Vecot
		straightPathList.add(goalPosition.vectorDifference(position).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS));
		pieceList.add(straightPathList);
		
		List<Vector> goalOrientationList = new ArrayList<Vector>();
		goalOrientationList.add(goalOrientation.normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS));
		pieceList.add(goalOrientationList);
		
		List<Vector> currentOrientationList = new ArrayList<Vector>();
		currentOrientationList.add(orientation.normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS));
		pieceList.add(currentOrientationList);
		
		return pieceList;
	}
	
	public float generateHeuristic(Vector position, Vector orientation, Vector goalPosition, Vector goalOrientation) {
		return position.distanceBetween(goalPosition)*(orientation.getAngleBetween(goalOrientation)+1);
	}
	
	public float generateHeuristic(List<Vector> path, Vector originalOrientation, Vector goalPosition, Vector goalOrientation) {
		
		if (path.size() == 0) {
			return Float.POSITIVE_INFINITY;
		}
		
		Vector orientation = getCurrentOrientation(path, originalOrientation);
		Vector position = path.get(path.size()-1);
		
		return generateHeuristic(position, orientation, goalPosition, goalOrientation);
	}
	
	public Vector getCurrentOrientation(List<Vector> path, Vector originalOrientation) {
		Vector orientation = originalOrientation;
		int length = path.size();
		if (length >= 2)orientation = path.get(length-2).vectorDifference(path.get(length-1));
		return orientation;
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
	 * Return the distance a point moving in the direction "direction" should travel horizontally, starting from "destination", to arrive
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

	/**
	 * Generates a vector that is oriented perpendicularly on the horizontal cord between the two given points.
	 * Points to the left when standing in position2, looking towards position1
	 * @param position1
	 * @param position2
	 * @return
	 */
	private static Vector parallelHorizontalVector(Vector position1, Vector position2) {
		position1 = position1.makeHorizontal();
		position2 = position2.makeHorizontal();

		Vector distanceBetween = position1.vectorDifference(position2);
		return distanceBetween.rotateAroundYAxis((float)Math.PI/2);

	}

	/**
	 * Method for retrieving the next waypoint we should reach
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypoint(Vector position, Vector velocity, Vector destination) {
		if (!hasPathLocked())generateLandingPath(position, velocity, destination);
		return getPath().get(0);
	}

	/**
	 * Method for retrieving the next waypoint we should reach, after having just reached one
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypointSuccess(Vector position, Vector velocity, Vector destination) {
		if (!hasPathLocked()) {
			// Generate a path if none was generated
			generateLandingPath(position, velocity, destination);
		}else {
			if (getPath().size() <= 1) {
				// If path contains only one element, generate a new path
				generateLandingPath(position, velocity, destination);
			}else {
				// If path is long enough, erase first element
				List<Vector> newPath = getPath();
				newPath.remove(0);
				setPath(newPath);
			}
		}
		return getPath().get(0);

	}

	/**
	 * Method for retrieving the next waypoint we should reach, after having missed one.
	 * Always regenerates the path.
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypointMissed(Vector position, Vector velocity, Vector destination) {
		// Always generate a new path
		generateLandingPath(position, velocity, destination);
		return getPath().get(0);
	}


	private static final float DISTANCE_BETWEEN_LANDING_BLOCKS = 20f;
	private static final float LANDING_ANGLE = (float)Math.PI/12;
	private static final float STEEPEST_TURN_DIAMETER = 20f;
	private static final float ALIGNMENT_TURN_ANGLE = (float)Math.PI/8;
	private static final float ALIGNMENT_TURN_FACTOR = 2.5f;
	private static final float RUN_DISTANCE = 40f;
	private static final float REGULAR_TURN_BLOCK_DISTANCE_FACTOR = 1.5f;
	private static final float REGULAR_TURN_ANGLE = (float)Math.PI/180f*30f;
	private static final float PRECISION = 0.9999f;
	private static final int NB_OF_GENERIC_VECTORS = 5;
	private static final float MAX_TURN_ANGLE = (float)Math.PI/8f;

	private boolean pathLock;
	private List<Vector> path;
	
	/*
     * DEPRECATED LANDING PATH GENERATOR
     * 
    private Vector getLandingStartPosition(Vector position, Vector direction, Vector destination) {
    	Vector landingVector = getLandingVector(position, direction, destination);

    	Vector lastBlock = destination;

		while(lastBlock.getyValue() < position.getyValue()) {
			lastBlock = lastBlock.vectorSum(landingVector);
			if (lastBlock.getyValue() >= position.getyValue()) {
				lastBlock = new Vector(lastBlock.getxValue(),position.getyValue(),lastBlock.getzValue());
			}
		}
		return lastBlock;
	}
	
	public void generateLandingPath(Vector position, Vector velocity, Vector destination){

		// TODO should add section that repositions drone to horizontal trajectory before generating path

		List<Vector> blockCoordinates = new ArrayList<Vector>();

		if (canFindRegularPath(position, velocity, destination)) {
			System.out.println("generating regular path");
			// generate a regular path
			Vector lastBlock = position;
			Vector lastDirection = velocity.makeHorizontal();
		
			
			// turn drone until it is in a straight path towards the goal
			while (lastDirection.getAngleBetween(destination.vectorDifference(lastBlock).makeHorizontal()) > 1-PRECISION) {
				lastDirection = getRegularTurnVector(lastBlock, lastDirection, destination);
				lastBlock = lastBlock.vectorSum(lastDirection);
				blockCoordinates.add(lastBlock);
			};
			
			// make drone go forward until it can start landing
			float landingMargin = getLandingMargin(position, velocity);
			while (lastBlock.makeHorizontal().distanceBetween(destination.makeHorizontal())*PRECISION > landingMargin ) {
				
				float distanceToLandingStart = lastBlock.makeHorizontal().distanceBetween(destination.makeHorizontal())-landingMargin;
				//System.out.println(landingMargin);
				//System.out.println(distanceToLandingStart+" Stuck in path generator ");
				//System.out.println("Destination: " + destination);
				//System.out.println("Current location: " + lastBlock);
				lastBlock = lastBlock.vectorSum(lastDirection.normalizeToLength(Math.min(distanceToLandingStart, lastDirection.getSize())));
				blockCoordinates.add(lastBlock);
			}
			
			// add landing blocks
			List<Vector> landingBlocksReversed = new ArrayList<Vector>();
			Vector landingVector = getLandingVector(lastBlock, lastDirection, destination);
			// generate landing path
			Vector lastPosition = lastBlock;
			lastBlock = destination;
			landingBlocksReversed.add(lastBlock);
			while(lastBlock.getyValue() < (lastPosition.getyValue()*PRECISION)) {

				lastBlock = lastBlock.vectorSum(landingVector);
				if (lastBlock.getyValue() >= lastPosition.getyValue()) {
					lastBlock = new Vector(lastBlock.getxValue(),lastPosition.getyValue(),lastPosition.getzValue());
				}
				landingBlocksReversed.add(lastBlock);
			}
			
			Collections.reverse(landingBlocksReversed);
			blockCoordinates.addAll(landingBlocksReversed);
			
			pathLock();
			
			
		}else if (canMakeTurn(position, velocity, destination)) {
			System.out.println("generating special path");
			Vector landingVector = getLandingVector(position, velocity, destination);
			
			// generate landing path
			Vector lastBlock = destination;
			blockCoordinates.add(lastBlock);
			while(lastBlock.getyValue() < (position.getyValue()*PRECISION)) {

				lastBlock = lastBlock.vectorSum(landingVector);
				if (lastBlock.getyValue() >= position.getyValue()) {
					lastBlock = new Vector(lastBlock.getxValue(),position.getyValue(),lastBlock.getzValue());
				}
				blockCoordinates.add(lastBlock);
			}

			// generate additional path
			float additionalDistance = forwardDistanceBetween(position, velocity, lastBlock) + RUN_DISTANCE;
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
			int numberOfSemiCircleWaypoints = (int)Math.floor((double)(Math.PI*circleDiameter/2f/DISTANCE_BETWEEN_LANDING_BLOCKS))-1;
			if (lastBlock.toTheRightOf(position, velocity)) {
				Vector circleCenter = lastBlock.vectorSum(prolongVector.rotateAroundYAxis((float)Math.PI/2f).normalizeToLength(circleDiameter/2f));
				Vector radius = prolongVector.rotateAroundYAxis((float)Math.PI/2f).normalizeToLength(circleDiameter/2f).scalarMult(-1f);

				for (int i = 0; i < numberOfSemiCircleWaypoints; i++) {
					radius = radius.rotateAroundYAxis((float)(Math.PI)/numberOfSemiCircleWaypoints);
					lastBlock = circleCenter.vectorSum(radius);
					blockCoordinates.add(lastBlock);
				}
			}else {
				Vector circleCenter = lastBlock.vectorSum(prolongVector.rotateAroundYAxis(-(float)Math.PI/2f).normalizeToLength(circleDiameter/2f));
				Vector radius = prolongVector.rotateAroundYAxis(-(float)Math.PI/2f).normalizeToLength(circleDiameter/2f).scalarMult(-1f);

				for (int i = 0; i < numberOfSemiCircleWaypoints; i++) {
					radius = radius.rotateAroundYAxis(-(float)(Math.PI)/numberOfSemiCircleWaypoints);
					lastBlock = circleCenter.vectorSum(radius);
					blockCoordinates.add(lastBlock);
				}
			}


			// generate additional path
			// path before start of semi-circle should be prolonged
			while (prolongedDistance < Math.max(additionalDistance - DISTANCE_BETWEEN_LANDING_BLOCKS + RUN_DISTANCE, RUN_DISTANCE - DISTANCE_BETWEEN_LANDING_BLOCKS)) {
				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
				lastBlock = lastBlock.vectorSum(prolongVector.scalarMult(-1));
				blockCoordinates.add(lastBlock);
			}
			
			Collections.reverse(blockCoordinates);
    		
    		pathLock();
    		
    	}else {
    		// get the drone further away from the destination in order to be able to turn
    		
    		if (position.makeHorizontal().distanceBetween(getLandingStartPosition(position, parallelHorizontalVector(position, destination), destination).makeHorizontal()) >= STEEPEST_TURN_DIAMETER) {
    			// if distance between drone and destination is large enough to make a turn, start aligning
        		if (getLandingStartPosition(position, parallelHorizontalVector(position, destination), destination).makeHorizontal().toTheRightOf(position, velocity)) {
        			// Turn right
        			blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().rotateAroundYAxis(-ALIGNMENT_TURN_ANGLE).normalizeToLength(velocity.getSize()*ALIGNMENT_TURN_FACTOR)));
        		}else {
        			// Turn left
        			blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().rotateAroundYAxis(ALIGNMENT_TURN_ANGLE).normalizeToLength(velocity.getSize()*ALIGNMENT_TURN_FACTOR)));
        		}
    		}else {
    			// if distance between drone and destination is too small to allow for a proper turn, keep on flying away (horizontally) from destination
    	    	blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().scalarMult(ALIGNMENT_TURN_FACTOR)));
    		}
    		
    		pathLock();
    	}
    	
    	setPath(blockCoordinates);
        
	}

	private Vector getLandingVector(Vector position, Vector direction, Vector destination) {

	    	Vector horizontalDirection = direction.makeHorizontal();
	    	// Formula: y = sqrt(x^2+z^2)*tan(landingAngle)
	    	return new Vector(horizontalDirection.getxValue(),(float)(Math.sqrt(Math.pow(horizontalDirection.getxValue(),2)+Math.pow(horizontalDirection.getzValue(),2))*Math.tan(LANDING_ANGLE)),horizontalDirection.getzValue()).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
	    }
	    
    private boolean canMakeTurn(Vector position, Vector direction, Vector destination) {
    	return parallelDistanceBetween(position, direction, getLandingStartPosition(position, direction, destination)) >= STEEPEST_TURN_DIAMETER;
    }
    
    private boolean canFindRegularPath(Vector position, Vector direction, Vector destination) {
    	return getTurnLimit(position, direction, destination) <= forwardDistanceBetween(position, direction, destination) ;
    }
    
    private float getTurnLimit(Vector position, Vector direction, Vector destination){
    	
    	// Find margin for turning 90 degrees to the left or right
    	Vector startPosition = position.makeHorizontal();
    	Vector horizontalDestination = destination.makeHorizontal();
    	Vector currentPosition = position.makeHorizontal();
    	Vector currentDirection = direction.makeHorizontal().normalizeToLength(direction.getSize()*REGULAR_TURN_BLOCK_DISTANCE_FACTOR);;
    	Vector startDirection = currentDirection;
    	float angle = currentDirection.getAngleBetween(horizontalDestination.vectorDifference(startPosition));
    	while (currentDirection.getAngleBetween(startDirection) < angle*PRECISION) {
    		currentDirection = currentDirection.rotateAroundYAxis(Math.min(REGULAR_TURN_ANGLE, angle-currentDirection.getAngleBetween(startDirection)));
    		currentPosition = currentPosition.vectorSum(currentDirection);
    	}
    	
    	float turnMargin = forwardDistanceBetween(startPosition, currentDirection, currentPosition);
    	
    	// Find margin for landing
    	float landingMargin = getLandingMargin(position, direction);
    	System.out.println("Turn margin: "+turnMargin);
    	return landingMargin + turnMargin;
    	
    }
    
    private float getLandingMargin(Vector position, Vector direction) {
    	// Find margin for landing
		Vector startPosition = new Vector(0,0,0);
		Vector currentPosition = startPosition;
		Vector landingVector = getLandingVector(position, direction, currentPosition);
		while(currentPosition.getyValue() < (position.getyValue()*PRECISION)) {

			currentPosition = currentPosition.vectorSum(landingVector);
			if (currentPosition.getyValue() >= position.getyValue()) {
				currentPosition = new Vector(currentPosition.getxValue(),position.getyValue(),currentPosition.getzValue());
			}
		}
    	return startPosition.makeHorizontal().distanceBetween(currentPosition.makeHorizontal());
    	
    }
    
    private Vector getRegularTurnVector(Vector position, Vector direction, Vector destination) {
    	Vector horizontalPosition = position.makeHorizontal();
    	Vector horizontalDirection = direction.makeHorizontal().normalizeToLength(direction.getSize());
    	Vector horizontalDestination = destination.makeHorizontal();
    	
    	Vector toDestination = horizontalDestination.vectorDifference(horizontalPosition);
    	
    	if (horizontalPosition.toTheRightOf(horizontalDestination, toDestination)) {
    		// If the destination is to the left, we turn left
    		return horizontalDirection.rotateAroundYAxis(Math.min(toDestination.getAngleBetween(horizontalDirection),REGULAR_TURN_ANGLE)).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    		//return horizontalDirection.normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    	}else {
    		// Otherwise we turn to the right
    		//return horizontalDirection.rotateAroundYAxis(-Math.min(toDestination.getAngleBetween(horizontalDirection),REGULAR_TURN_ANGLE)).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    		return horizontalDirection.rotateAroundYAxis(-Math.min(toDestination.getAngleBetween(horizontalDirection),REGULAR_TURN_ANGLE)).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
    	}
    }
    */
}
