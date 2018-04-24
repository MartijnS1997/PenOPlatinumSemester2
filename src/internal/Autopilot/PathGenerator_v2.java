package internal.Autopilot;

import java.util.ArrayList;
import java.util.List;

import internal.Helper.Vector;

/**
 * Class for generating (landing) paths
 * @author Anthony Rathe
 *
 */
public class PathGenerator_v2 {
	
	public PathGenerator_v2() {
		
	}
	
	public PathGenerator_v2(Vector position, Vector orientation, Vector landingPosition, Vector landingOrientation) {
		setPosition(position);
		setOrientation(orientation);
		setLandingPosition(landingPosition);
		setLandingOrientation(landingOrientation);
	}
	
	private Vector position;
	private Vector orientation;
	private Vector landingPosition;
	private Vector landingOrientation;
	
	public void updateVariables(Vector position, Vector orientation, Vector landingPosition, Vector landingOrientation) {
		setPosition(position);
		setOrientation(orientation);
		setLandingPosition(landingPosition);
		setLandingOrientation(landingOrientation);
	}
	
	public Vector getPosition() {
		return this.position;
	}
	
	public void setPosition(Vector position) {
		this.position = position;
	}
	
	public Vector getLandingPosition() {
		return this.landingPosition;
	}
	
	public void setLandingPosition(Vector landingPosition) {
		this.landingPosition = landingPosition;
	}
	
	public Vector getOrientation() {
		return this.orientation;
	}
	
	public void setOrientation(Vector orientation) {
		this.orientation = orientation;
	}
	
	public Vector getLandingOrientation() {
		return this.landingOrientation;
	}
	
	public void setLandingOrientation(Vector landingOrientation) {
		this.landingOrientation = landingOrientation;
	}
	
	public Vector getDescentStartPosition() {
		Vector position = getPosition();
		Vector landingPosition = getLandingPosition();
		float heightDifference = position.getyValue()-landingPosition.getyValue();
		float a = (float) (heightDifference/Math.sin(getLandingAngle()));
		
		Vector landingVector = getLandingVector().normalizeToLength(a);
		return landingPosition.vectorSum(landingVector);
	}
	
	public Vector getLandingVector() {
		Vector landingOrientation = getLandingOrientation().makeHorizontal().normalizeToLength(-1f);
		float r = landingOrientation.getSize();
		return new Vector(landingOrientation.getxValue(), (float)(r*Math.tan((float)getLandingAngle())), landingOrientation.getzValue()).normalizeVector();
	}
	
	public Vector getPointOfIntersection() {
		Vector position = getPosition().makeHorizontal();
		Vector landingPosition = getLandingPosition().makeHorizontal();
		Vector orientation = getOrientation().makeHorizontal();
		Vector landingOrientation = getLandingOrientation().makeHorizontal();
		
		float a_x = landingPosition.getxValue();
		float a_y = landingPosition.getzValue();
		
		float b_x = position.getxValue();
		float b_y = position.getzValue();
		
		float c_x = landingOrientation.getxValue();
		float c_y = landingOrientation.getzValue();
		
		float d_x = orientation.getxValue();
		float d_y = orientation.getzValue();
		
		float q_x;
		float q_y;
		if (c_x != 0) {
			float r = (a_y + (b_x-a_x)*c_y/c_x - b_y)/(d_y - d_x*c_y/c_x);
			q_x = r*d_x + b_x;
			q_y = r*d_y + b_y;
		}else {
			q_x = a_x;
			q_y = (a_x - b_y)*d_y/d_x + b_y;
		}
		
		
		return new Vector(q_x, 0f, q_y);
		
	}
	
	/*
	 * PURE CHAOS
	public void generatePath() {
		Vector position = getPosition().makeHorizontal();
		Vector orientation = getOrientation().makeHorizontal();
		Vector descentStartPosition = getDescentStartPosition().makeHorizontal();
		Vector descentStartOrientation = getLandingOrientation().makeHorizontal();
		
		// First we check if we can generate a path with only a single turn
		Vector intersection = getPointOfIntersection();
		float alfa = orientation.getAngleBetween(descentStartOrientation);
		float R = getStandardRadius();
		float L = R/(float)(Math.cos(alfa/2));
		float l = L*(float)(Math.sin(alfa/2));
		
		Vector L_vector;
		Vector l_in_vector = orientation.normalizeToLength(-l);
		Vector l_out_vector;
		if (descentStartPosition.toTheRightOf(position, orientation)) {
			L_vector = orientation.normalizeToLength(-L).rotateAroundYAxis(((float)Math.PI-alfa)/2f);
			l_out_vector = l_in_vector.rotateAroundYAxis(((float)Math.PI-alfa)/2f);
		}else {
			L_vector = orientation.normalizeToLength(-L).rotateAroundYAxis(-((float)Math.PI-alfa)/2f);
			l_out_vector = l_in_vector.rotateAroundYAxis(-((float)Math.PI-alfa)/2f);
		}
		
		Vector center = intersection.vectorSum(L_vector);
		Vector in = intersection.vectorSum(l_in_vector);
		Vector out = intersection.vectorSum(l_out_vector);
		
		float turnAngle = out.vectorDifference(center).getAngleBetween(in.vectorDifference(center));
		if (descentStartPosition.toTheRightOf(position, orientation)) {
			// Turn to the right, so angle turnAngle should be negative
			turnAngle = -turnAngle;
		}
		
		
		// Alfa is the angle of the subsection of circle that is traversed
		// R is the radius of that circle
		// in is the entry point of the circle
		// out is the exit point of the circle
		
		float a = out.distanceBetween(descentStartPosition, descentStartOrientation);
		float b = position.distanceBetween(in, orientation);
		
		if (a >= 0 && b >= 0 && alfa != (float)Math.PI && alfa != 0f) {
			// It is possible to generate a path with only one turn
			AutopilotTurn firstTurn = new Turn(in, center, out, turnAngle, R);
			addTurn(firstTurn);
			System.out.println(in);
			System.out.println(center);
			System.out.println(out);
		}else {
			// Generate an S-path with two turns
			
			// Generate different combinations of turn centers
			List<List<Vector>> turnCenterCombinations = new ArrayList<List<Vector>>();
			List<Vector> firstTurnCenters = new ArrayList<Vector>();
			List<Vector> secondTurnCenters = new ArrayList<Vector>();
			
			Vector firstTurnBase = orientation.normalizeToLength(R);
			Vector secondTurnBase = descentStartOrientation.normalizeToLength(R);
			if (descentStartPosition.toTheRightOf(position, orientation)) {
				firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
				//firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
			}else {
				//firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
				firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
			}
			
			if (position.toTheRightOf(descentStartPosition, descentStartOrientation)) {
				secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
				secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
			}else {
				secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
				secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
			}
				
			for (Vector firstCenter : firstTurnCenters) {
				for (Vector secondCenter : secondTurnCenters) {
					List<Vector> combination = new ArrayList<Vector>();
					combination.add(firstCenter);
					combination.add(secondCenter);
					turnCenterCombinations.add(combination);
				}
			}
			
			
			for (List<Vector> turnCenterCombination : turnCenterCombinations) {
				// Generate a possible path with given turnCenters
				Vector firstTurnCenter = turnCenterCombination.get(0);
				Vector secondTurnCenter = turnCenterCombination.get(1);
				
				Vector C1C2 = secondTurnCenter.vectorDifference(firstTurnCenter);
				Vector C2C1 = C1C2.normalizeToLength(-1);
				float C1C2Length = C1C2.getSize();
				float theta1 = (float) Math.acos(2f*R/C1C2Length);
				float theta2 = theta1;
				
				if (secondTurnCenter.toTheRightOf(descentStartPosition, descentStartOrientation)) {
					theta2 = -theta2;
				}
				
				Vector firstExit = firstTurnCenter.vectorSum(C1C2.normalizeToLength(R).rotateAroundYAxis(theta1));
				Vector secondEntry = secondTurnCenter.vectorSum(C2C1.normalizeToLength(R).rotateAroundYAxis(theta2));
				
				if (firstExit.vectorDifference(secondEntry).getSize() >= getMinimumStabilizationDistance()) {
					// This is a valid path
					float firstTurnAngle = 
					AutopilotTurn firstTurn = new Turn()
					break;
				}
				
			}
		}
		
	}*/
	
	public void generatePath() {
		
		getTurns().clear();
		
		Vector position = getPosition().makeHorizontal();
		Vector orientation = getOrientation().makeHorizontal();
		Vector descentStartPosition = getDescentStartPosition().makeHorizontal();
		Vector descentStartOrientation = getLandingOrientation().makeHorizontal();
		
		float R = getStandardRadius();
		
		// First place the centers of each turn
		// Select the centers such that the distance between the centers is minimal
		Vector firstTurnCenter;
		Vector secondTurnCenter;
		List<Vector> firstTurnCenters = new ArrayList<Vector>();
		List<Vector> secondTurnCenters = new ArrayList<Vector>();
		
		Vector firstTurnBase = orientation.normalizeToLength(R);
		Vector secondTurnBase = descentStartOrientation.normalizeToLength(R);
		firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
		firstTurnCenters.add(position.vectorSum(firstTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
		secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis(-(float)Math.PI/2f)));
		secondTurnCenters.add(descentStartPosition.vectorSum(secondTurnBase.rotateAroundYAxis((float)Math.PI/2f)));
			
		firstTurnCenter = firstTurnCenters.get(0);
		secondTurnCenter = secondTurnCenters.get(0);
		
		for (Vector firstCenter : firstTurnCenters) {
			for (Vector secondCenter : secondTurnCenters) {
				if (firstCenter.distanceBetween(secondCenter) < firstTurnCenter.distanceBetween(secondTurnCenter)) {
					firstTurnCenter = firstCenter;
					secondTurnCenter = secondCenter;
				}
			}
		}
		
		Vector C1C2 = secondTurnCenter.vectorDifference(firstTurnCenter);
		float C1C2Length = C1C2.getSize();
		Vector C1C2Middle = firstTurnCenter.vectorSum(C1C2.scalarMult(0.5f));
		
		// Determine the exit points for the first turn
		// There are two possible exit points:
		//	- Inner: this exit point corresponds to the inner tangent
		//	- Outer: this exit point corresponds to the outer tangent
		float angleSign = -1;
		if (firstTurnCenter.toTheRightOf(position, orientation)) {
			angleSign = 1;
		}
		
		float angle = (float)Math.acos(2*R/C1C2Length);
		
		Vector firstExitInner = firstTurnCenter.vectorSum(C1C2.normalizeToLength(R).rotateAroundYAxis(angleSign*angle));
		Vector firstExitOuter = firstTurnCenter.vectorSum(C1C2.normalizeToLength(R).rotateAroundYAxis((float)(angleSign*Math.PI/2f)));
		
		Vector innerTangent = C1C2Middle.vectorDifference(firstExitInner).scalarMult(2);
		Vector outerTangent = C1C2;
		
		// Determine the possible entry points for the second turn
		// There are two possible entry points:
		//	- Inner: this entry point corresponds to the inner tangent
		//	- Outer: this entry point corresponds to the outer tangent
		
		Vector secondEntryInner = firstExitInner.vectorSum(innerTangent);
		Vector secondEntryOuter = firstExitOuter.vectorSum(outerTangent);
		
		// Determine whether the outer or the inner tangent should be selected
		Vector firstExit;
		Vector secondEntry;
		if (secondTurnCenter.toTheRightOf(firstExitOuter, outerTangent) == secondTurnCenter.toTheRightOf(descentStartPosition, descentStartOrientation)) {
			// Select the outer tangent
			firstExit = firstExitOuter;
			secondEntry = secondEntryOuter;
		}else {
			// Select the inner tangent
			firstExit = firstExitInner;
			secondEntry = secondEntryInner;
		}
		
		Vector firstEntry = position;
		Vector secondExit = descentStartPosition;
		
		float firstAngle = firstEntry.vectorDifference(firstTurnCenter).getSignedAngleBetween(firstExit.vectorDifference(firstTurnCenter));
		float secondAngle = secondEntry.vectorDifference(secondTurnCenter).getSignedAngleBetween(secondExit.vectorDifference(secondTurnCenter));
		
		AutopilotTurn firstTurn = new Turn(firstEntry, firstTurnCenter, firstExit, firstAngle, R);
		AutopilotTurn secondTurn = new Turn(secondEntry, secondTurnCenter, secondExit, secondAngle, R);
		
		addTurn(firstTurn);
		addTurn(secondTurn);
	}
	
	public List<AutopilotTurn> getTurns(){
		return this.turns;
	}
	
	private void addTurn(AutopilotTurn turn) {
		this.turns.add(turn);
	}
	
	public AutopilotTurn getNextTurn() {
		
		if (getTurns().size() == 0) {
			return null;
		}
		
		AutopilotTurn nextTurn = getTurns().get(0);
		getTurns().remove(0);
		return nextTurn;
		
	}
	
	private float getStandardRadius() {
		return MINIMUM_RADIUS;
	}
	
	private float getLandingAngle() {
		return LANDING_ANGLE;
	}
	
	private float getMinimumStabilizationDistance() {
		return MINIMUM_STABILIZATION_DISTANCE;
	}
	
	
	private List<AutopilotTurn> turns = new ArrayList<AutopilotTurn>();
	
	private final float MINIMUM_RADIUS = 1000;
	private final float LANDING_ANGLE = (float)Math.PI/18;
	private final float MINIMUM_STABILIZATION_DISTANCE = 200;
	
	
	
	
}
