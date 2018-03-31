package internal.Testbed;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import TestbedAutopilotInterface.AirportGuiState;
import TestbedAutopilotInterface.MapAirport;
import internal.Helper.Vector;

//TODO make aiports NON world objects and pull the world object interface from the general testbed (only drones are
// entities that need to be advanced & checked)

public class WorldAirport extends MapAirport {


	/**
	 * Constructor for an airport located in the world, used for testbed purposes
	 * @param location the location of the airport (y component needs to be 0)
	 * @param heading the heading of runway zero
	 * @param runwayWidth the width of the runways of the airport
	 * @param runwayLength the length of the runways of the airport
	 * @param airportID the id of the newly created airport
	 */
	public WorldAirport(Vector location, Vector heading, float runwayWidth, float runwayLength, int airportID){
		super(location, heading, runwayWidth, runwayLength, airportID);
	}

	/**
	 * Checks if the given position is within the airport
	 * @param position the position to check
	 * @return true if and only if the position is located onto airport
	 */
	public boolean isWithinAirport(Vector position){
		//first get the heading vector of the airport
		Vector heading = this.getHeadingVector();
		Vector location = this.getLocation();
		//also grab the length and the width of the runway
		float length = this.getRunwayLength();
		float width = this.getRunwayWidth();
		//project the position upon the heading vector
		//first we need to subtract the position of the airport from the provided position
		//to get the relative position (account for translation of the projection)
		position = position.vectorDifference(location);
		Vector projection = position.projectOn(heading);
		//check if the size is smaller than the length of the airport
		if(projection.getSize() > length) {
			return false;
		}
		//check if the difference fits within the width of the airport (is orthogonal to the heading)
		Vector projDiff = projection.vectorDifference(position);
		if(projDiff.getSize() > width){
			return false;
		}
		//all checks passed, return true
		return true;
	}


	/**
	 * Getter for the state of the Airport for the GUI
	 * @return an airport gui state of the airport
	 */
	public AirportGuiState getGuiState(){
		Vector location = this.getLocation();
		Vector headingVector = this.getHeadingVector();

		return new AirportGuiState() {
			@Override
			public Vector getPosition() {
				return location;
			}

			@Override
			public Vector getPrimaryRunWay() {
				return headingVector;
			}
		};
	}

}