package gui.GraphicsObjects;

import math.Matrix3f;
import math.Vector3f;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public interface Polygon extends GraphicsObject {
	
	Vector3f relativePosition = new Vector3f();
	
	public void update(Vector3f displacement, Vector3f orientation);

	public default Vector3f getOrientation() {
		return this.orientation;
	}
	
	public default Vector3f getRelPos() {
		Vector3f pos = this.position;
		Matrix3f transformation = Matrix3f.transformationMatrix(this.orientation).transpose();
		float heading = this.orientation.x;
		float pitch = this.orientation.y;
		float roll = this.orientation.z;
		Matrix3f transform =
				new Matrix3f(
						new Vector3f((float) (cos(heading)*cos(roll) + sin(heading)*sin(pitch)*sin(roll)), (float) (cos(roll)*sin(heading)*sin(pitch)-cos(heading)*sin(roll)), (float) (cos(pitch)*sin(heading))),
						new Vector3f((float) (cos(pitch)*sin(roll)), (float) (cos(pitch)*cos(roll)), (float) -sin(pitch)),
						new Vector3f((float) (cos(heading)*sin(pitch)*sin(roll) - cos(roll)*sin(heading)), (float) (sin(heading)*sin(roll)+cos(heading)*cos(roll)*sin(pitch)), (float) (cos(heading)*cos(pitch)))).transpose();

		Vector3f difference = transformation.multiply(relativePosition);
		pos = pos.add(difference);
		return pos;
	}
}
