package internal.Autopilot;

import internal.Helper.Vector;

public class Turn implements AutopilotTurn{

	private final Vector entryPoint;
	private final Vector centerPoint;
	private final Vector exitPoint;
	private final float turnAngle;
	private final float turnRadius;
	
	public Turn(Vector entryPoint, Vector centerPoint, Vector exitPoint, float turnAngle, float turnRadius) {
		this.entryPoint = entryPoint;
		this.centerPoint = centerPoint;
		this.exitPoint = exitPoint;
		this.turnAngle = turnAngle;
		this.turnRadius = turnRadius;
	}
	
	public Turn(Vector entryPoint, Vector centerPoint, float turnAngle, float turnRadius) {
		this(entryPoint, centerPoint, centerPoint.vectorSum(entryPoint.vectorDifference(centerPoint).rotateAroundYAxis(turnAngle)), turnAngle, turnRadius);
	}
	
	public Vector getTurnCenter() {
		return this.centerPoint;
	}
	
	public Vector getEntryPoint() {
		return this.entryPoint;
	}
	
	public Vector getExitPoint() {
		return this.exitPoint;
	}
	
	public float getTurnRadius() {
		return this.turnRadius;
	}
	
	public float getTurnAngle() {
		return this.turnAngle;
	}
}
