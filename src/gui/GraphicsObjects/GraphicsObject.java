package gui.GraphicsObjects;

import gui.GL.Mesh;
import internal.Helper.Vector;
import math.Vector3f;

public interface GraphicsObject {
	
	Mesh mesh = null;
	Vector3f size = null;
	Vector3f position = null;
	Vector3f orientation = null;
	float[] colours = null;

	public void update(Vector3f position, Vector3f orientation);

	public default void render() {
		GraphicsObject.mesh.render();
	}
	
	public default void delete() {
		GraphicsObject.mesh.delete();
	}

	public default Vector getPosition(){
		return Vector.vector3fToVector(this.getPos());
	}

	public default Vector3f getOrientation(){
		return GraphicsObject.orientation;
	}
	
	public default Vector3f getSize() {
		return GraphicsObject.size;
	}
	
	public default Vector3f getPos() {
		return GraphicsObject.position;
	}
}