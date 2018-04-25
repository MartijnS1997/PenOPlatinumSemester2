import internal.Autopilot.AutopilotTurn;
import internal.Autopilot.PathGenerator_v2;
import internal.Helper.Vector;

public class TestMain {
	public static void main(String[] args) {
		// Initialize an empty Path Generator
		PathGenerator_v2 pathGen = new PathGenerator_v2();
		
		// Initialize all required variables for path generation
		// Position of the drone at the moment of generation
		Vector position = new Vector(0,0,0);
		// Orientation of the drone at the moment of generation (only x- and z- components will be considered)
		Vector orientation = new Vector(1,0,1);
		// Position on which the drone should be touching down
		Vector landingPosition = new Vector(5000,0,0);
		// Orientation in which the drone should be landing (only x- and z- components will be considered)
		Vector landingOrientation = new Vector(0,0,-1);
		
		// Insert required variables for path generation
		pathGen.updateVariables(position, orientation, landingPosition, landingOrientation);
		
		// Generate a path based on those variables
		pathGen.generatePath();
		
		// Catch first turn
		AutopilotTurn firstTurn = pathGen.getNextTurn();
		// Catch second turn
		AutopilotTurn secondTurn = pathGen.getNextTurn();
		// Catch third turn (null as it indicates 'end of path'
		AutopilotTurn thirdTurn = pathGen.getNextTurn();
		
		System.out.println(firstTurn.getTurnCenter());
		System.out.println(secondTurn.getTurnCenter());
		System.out.println(thirdTurn);
	}
}
