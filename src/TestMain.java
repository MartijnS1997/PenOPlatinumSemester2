import internal.Autopilot.PathGenerator_v2;
import internal.Helper.Vector;

public class TestMain {
	public static void main(String[] args) {
		PathGenerator_v2 pathGen = new PathGenerator_v2();
		
		Vector position = new Vector(0,0,0);
		Vector orientation = new Vector(1,0,1);
		Vector landingPosition = new Vector(10,0,0);
		Vector landingOrientation = new Vector(0,0,-1);
		pathGen.updateVariables(position, orientation, landingPosition, landingOrientation);
		pathGen.generatePath();
	}
}
