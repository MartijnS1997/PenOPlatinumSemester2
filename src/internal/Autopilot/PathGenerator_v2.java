package internal.Autopilot;

import internal.Helper.Vector;

/**
 * Class for generating (landing) paths
 * @author Anthony Rathé
 *
 */
public class PathGenerator_v2 {
	
	public PathGenerator_v2() {
		
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
		
		
		// Alfa is the angle of the subsection of circle that is traversed
		// R is the radius of that circle
		// in is the entry point of the circle
		// out is the exit point of the circle
		
		float a = out.distanceBetween(descentStartPosition, descentStartOrientation);
		float b = position.distanceBetween(in, orientation);
		
		if (a >= 0 && b >= 0 && alfa != (float)Math.PI && alfa != 0f) {
			// It is possible to generate a path with only one turn
			System.out.println(in);
			System.out.println(center);
			System.out.println(out);
		}else {
			// Generate an S-path with two turns
			/* KLOPT NIET
			float gamma = orientation.getAngleBetween(descentStartPosition.vectorDifference(position));
			if (!descentStartPosition.toTheRightOf(position, orientation)) gamma = -gamma;
			
			float r  = (R*(1-(float)Math.sin(gamma))/((float)Math.cos(alfa)));
			*/ 
		}
		
		
		
		
		
	}
	
	private float getStandardRadius() {
		return MINIMUM_RADIUS*1.2f;
	}
	
	private float getLandingAngle() {
		return LANDING_ANGLE;
	}
	
	private final float MINIMUM_RADIUS = 1000;
	private final float LANDING_ANGLE = (float)Math.PI/18;
	
	
	
	
}
