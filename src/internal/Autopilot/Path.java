package internal.Autopilot;

public class Path implements Autopilot.Path {

	public Path(float[] x, float[] y, float[] z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	@Override
	public float[] getX() {
		return x;
	}

	@Override
	public float[] getY() {
		return y;
	}

	@Override
	public float[] getZ() {
		return z;
	}

	
	
	float[] x;
	float[] y;
	float[] z;
}
