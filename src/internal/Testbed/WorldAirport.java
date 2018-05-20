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
	 * Checks if the given position is withing the boundary of the gates
	 * @param position the position to check
	 * @return true if and only if the position is within the borders of the gate
	 */
	protected boolean isAtGateZone(Vector position){
		//get the center of the airport and the width of the runways
		Vector airportLocation = this.getLocation();
		Vector airportHeading = this.getHeadingVector();
		float runwayWidth = this.getRunwayWidth(); //width is only 1/2 of the real width for some reason

		//get the right (or left) orthogonal, needed to project onto
		Vector rightOrthogonal = this.getRightOrthogonal(airportHeading);

		//then take the difference vector between the position (@ground) of the drone and the airport (ptr from ap to drone)
		Vector droneGroundPos = position.projectOntoXZplane();
		Vector diff = droneGroundPos.vectorDifference(airportLocation);

		//now project onto the heading vector and the right orthogonal
		Vector projectOnOrth = diff.projectOn(rightOrthogonal);
		Vector projectOnHeading = diff.projectOn(airportHeading);

		//check if the projection or the orthogonal is within airport borders (and same for the heading vector)
		float orthSize = projectOnOrth.getSize();
		float headingSize = projectOnHeading.getSize();

		boolean isWithinOrth = isBetween(orthSize, 0,runwayWidth);
		boolean isWithinHeading = isBetween(headingSize, 0, runwayWidth/2);
//		if(droneID.equals("0")){
//			System.out.println("isWithinOrth: " + isWithinOrth + ", isWithinHeading: " + isWithinHeading);
//			System.out.println("headingSize: " + headingSize);
//			System.out.println("orthSize: " + orthSize);
//			System.out.println("Heading: " + airportHeading);
//			System.out.println("Airportlocation: "+  airportLocation);
//			System.out.println("drone location: " + droneGroundPos);
//		}
		return isWithinOrth && isWithinHeading;
	}

	/**
	 * Checks if the the given float lies in the interval between the lower and upper border
	 * @param toCheck the float to check
	 * @param lowerBorder the lower border of the interval
	 * @param upperBorder the upper border of the interval
	 * @return true if and only if the toCheck is in range [lower, upper]
	 */
	private static boolean isBetween(float toCheck, float lowerBorder, float upperBorder){
		return toCheck >= lowerBorder && toCheck <= upperBorder;
	}

	/**
	 * Getter for the state of the Airport for the GUI
	 * @return an airport gui state of the airport
	 */
	public AirportGuiState getGuiState(){
		Vector location = this.getLocation();
		Vector headingVector = this.getHeadingVector();
		int airportID = WorldAirport.this.getAirportID();

		return new AirportGuiState() {
			@Override
			public Vector getPosition() {
				return location;
			}

			@Override
			public Vector getPrimaryRunWay() {
				return headingVector;
			}

			@Override
			public int getAirportID(){
				return airportID;
			}
		};
	}

}