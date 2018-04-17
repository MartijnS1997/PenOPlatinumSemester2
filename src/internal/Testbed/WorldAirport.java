package internal.Testbed;

import TestbedAutopilotInterface.GUI.AirportGuiState;
import TestbedAutopilotInterface.Overseer.MapAirport;
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