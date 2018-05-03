package TestbedAutopilotInterface.Overseer;

import internal.Helper.Vector;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by Martijn on 29/03/2018.
 * A class of airports to be used as map elements, may be further extended to fit the needs for a world airport
 */
public class MapAirport {

    /**
     * Constructor for an airport
     * @param location the location of the airport
     * @param headingVector the heading vector of the first runway
     * @param runwayWidth the width of the runway
     * @param runwayLength the length of the runway
     * @param airportID the id of the airport
     */
    public MapAirport(Vector location, Vector headingVector, float runwayWidth, float runwayLength, int airportID) {
        this.location = location;
        this.headingVector = headingVector.normalizeVector(); // normalize the heading vector
        this.airportID = airportID;
        this.runwayWidth = runwayWidth;
        this.runwayLength = runwayLength;
        this.setGateLocations();
    }

    /**
     * Getter for the borders of the airport (the outer corners are given)
     * @return an array of four vectors, each containing one corner of the airport in the following order
     * upper left, upper right, lower left, lower right (upper defined as the direction of the heading vector
     * of runway zero)
     */
    public Vector[] getAirportBorders(){
        //get all the corners first
        Vector upperLeft = this.getUpperLeftCorner();
        Vector upperRight = this.getUpperRightCorner();
        Vector lowerLeft = this.getLowerLeftCorner();
        Vector lowerRight = this.getLowerRightCorner();

        //put them in an array and return
        return new Vector[]{upperLeft, upperRight, lowerLeft, lowerRight};
    }

    /**
     * Calculates the upper left corner of an airport (left to the heading vector of runway zero)
     * @return a vector containing the location of the upper left corner
     */
    private Vector getUpperLeftCorner(){
        //first get the needed parameters (the heading and the upper end point)
        Vector headingVector = this.getHeadingVector();
        Vector runwayZeroEdge = this.getRunwayZeroEnd();
        //based on the heading calculate the left orthogonal
        Vector leftOrthogonal = this.getLeftOrthogonal(headingVector);
        //sum the edge and the orthogonal to attain the upper left corner
        //but fist scale to width
        float runwayWidth = this.getRunwayWidth();
        Vector scaledLeft = leftOrthogonal.scalarMult(runwayWidth);
        return runwayZeroEdge.vectorSum(scaledLeft);
    }


    /**
     * Calculates the upper right corner of an airport (right to the heading vector of runway zero)
     * @return a vector containing the location of the upper right corner
     */
    private Vector getUpperRightCorner(){
        //first get the needed parameters (the heading and the upper end point)
        Vector headingVector = this.getHeadingVector();
        Vector runwayZeroEdge = this.getRunwayZeroEnd();
        //based on the heading calculate the left orthogonal
        Vector rightOrthogonal = this.getRightOrthogonal(headingVector);
        //sum the edge and the orthogonal to attain the upper left corner
        //but fist scale to width
        float runwayWidth = this.getRunwayWidth();
        Vector scaledRight = rightOrthogonal.scalarMult(runwayWidth);
        return runwayZeroEdge.vectorSum(scaledRight);
    }

    /**
     * Calculates the lower left corner of an airport (left to the heading vector of runway zero)
     * @return a vector containing the location of the lower left corner
     */
    private Vector getLowerLeftCorner(){
        //first get the needed parameters (the heading and the upper end point)
        Vector headingVector = this.getHeadingVector();
        Vector runwayOneEdge = this.getRunwayOneEnd();
        //based on the heading calculate the left orthogonal
        Vector leftOrthogonal = this.getLeftOrthogonal(headingVector);
        //sum the edge and the orthogonal to attain the upper left corner
        //but fist scale to width
        float runwayWidth = this.getRunwayWidth();
        Vector scaledLeft = leftOrthogonal.scalarMult(runwayWidth);
        return runwayOneEdge.vectorSum(scaledLeft);
    }

    /**
     * Calculates the lower right corner of an airport (right to the heading vector of runway zero)
     * @return a vector containing the location of the upper right corner
     */
    private Vector getLowerRightCorner(){
        //first get the needed parameters (the heading and the upper end point)
        Vector headingVector = this.getHeadingVector();
        Vector runwayOneEdge = this.getRunwayOneEnd();
        //based on the heading calculate the left orthogonal
        Vector rightOrthogonal = this.getRightOrthogonal(headingVector);
        //sum the edge and the orthogonal to attain the upper left corner
        //but fist scale to width
        float runwayWidth = this.getRunwayWidth();
        Vector scaledRight = rightOrthogonal.scalarMult(runwayWidth);
        return runwayOneEdge.vectorSum(scaledRight);
    }


    /**
     * Setter for the locations of the gates, gate zero and one are located to the left and right
     * of the heading vector of the first runway respectively (and the vectors pointing from the origin to
     * the gate are orthogonal to the heading vector of runway zero)
     */
    private void setGateLocations(){
        //position of gate one:
        Vector airportLocation = this.getLocation();
        float runwayWidth = this.getRunwayWidth();
        Vector headingVector = this.getHeadingVector();
        //get both orthogonal vectors (for the gates)
        //if heading is (x1, 0, z1) then scalar product with (z1,0,-x1) will result in a zero
        Vector leftOrthogonal = this.getLeftOrthogonal(headingVector);
        //if heading is (x1, 0, z1) then scalar product with (-z1, 0, x1) will give zero
        Vector rightOrthogonal = this.getRightOrthogonal(headingVector);
        //set the length of the orthogonals to 1/2 Width
        leftOrthogonal = leftOrthogonal.normalizeToLength(runwayWidth/2);
        rightOrthogonal = rightOrthogonal.normalizeToLength(runwayWidth/2);
        //gate 0 should be located left from the heading, gate 1 to the right
        Map<Integer, Vector> gateMap = this.getGates();
        //the first orthogonal will always be located on the left of the heading vector (see drawings)
        //first we need to add the center of the airport to get the absolute position
        Vector gateZeroPos = airportLocation.vectorSum(leftOrthogonal);
        gateMap.put(0, gateZeroPos);
        //the second orthogonal will always be located to the right of the heading vector
        //first add the position of the airport to get an absolute position
        Vector gateOnePos = airportLocation.vectorSum(rightOrthogonal);
        gateMap.put(1, gateOnePos);
    }

    /**
     * Gets the vector orthogonal and left to the heading vector of the airport
     * @param headingVector  the heading vector of the airport
     * @return a normalized vector orthogonal and to the left of the heading vector of the airport
     */
    private Vector getLeftOrthogonal(Vector headingVector){
        //if heading is (x1, 0, z1) then scalar product with (z1,0,-x1) will result in a zero
        //the orthogonal will always be located on the left of the heading vector (see drawings)
        Vector leftOrthogonal = new Vector(headingVector.getzValue(),0, -headingVector.getxValue());
        return leftOrthogonal.normalizeVector();
    }

    /**
     * Gets the vector orthogonal and right to the heading vector of the airport
     * @param headingVector  the heading vector of the airport
     * @return a normalized vector orthogonal and to the right of the heading vector of the airport
     */
    private Vector getRightOrthogonal(Vector headingVector){
        //if heading is (x1, 0, z1) then scalar product with (-z1, 0, x1) will give zero
        //the orthogonal will always be located to the right of the heading vector
        Vector secondOrthogonal = new Vector(-headingVector.getzValue(),0, headingVector.getxValue());
        return secondOrthogonal.normalizeVector();
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
        //only keep the x and z components of the position
        position = new Vector(position.getxValue(), 0f, position.getzValue());
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
     * Get the takeoff direction of the runway with the matching runway ID
     * @param runwayID the ID of the runway the takeoff direction is requested for
     * @return the takeoff direction of the runway with the same ID, if the id
     *          does not match the available runways return (0,0,0)
     */
    public Vector getTakeoffDirection(int runwayID){
        switch(runwayID){
            case 0:
                return getRunwayZeroTakeoffDirection();
            case 1:
                return getRunwayOneTakeoffDirection();
            default:
                return new Vector();
        }
    }

    /**
     * Getter for the landing direction for runway zero, this is the direction
     * in which an autopilot has to approach the runway if it wants to attempt a landing
     * @return a vector containing the direction in which to approach runway zero (x,0,z)
     */
    public Vector getRunwayZeroLandingDirection(){
        return this.getHeadingVector().scalarMult(-1f);
    }

    /**
     * Getter for the takeoff director for runway zero, this is the direction
     * in which to takeoff
     * @return a vector containing the direction to takeoff at runway zero
     */
    public Vector getRunwayZeroTakeoffDirection(){
        return this.getHeadingVector();
    }

    /**
     * Getter for the landing direction for the runway one, this is the direction
     * in which an autopilot has to approach the runway if it wants to attempt a landing
     * @return a vector containing the direction in which to approach runway one (x,0,z)
     */
    public Vector getRunwayOneLandingDirection(){
        return this.getHeadingVector();
    }

    /**
     * Get the takeoff direction of airport one, this is the direction in which to fly to takeoff
     * @return the takeoff direction of runway one
     */
    public Vector getRunwayOneTakeoffDirection(){
        return this.getHeadingVector().scalarMult(-1);
    }

    /**
     * Get the outer most point of runway zero in relative terms located at (0, Length)
     * @return a vector containing the endpoint of runway number zero in the world axis system
     */
    public Vector getRunwayZeroEnd(){
        //first get the heading vector
        //then scale it to the length of the runway and add the location of the airport to get an absolute
        Vector heading = this.getHeadingVector();
        Vector airportLocation = this.getLocation();
        float runwayLength = this.getRunwayLength();
        //we may simply multiply because the heading is already in the right direction
        Vector endOfRunwayRelative = heading.normalizeToLength(runwayLength);
        Vector endOfRunway = endOfRunwayRelative.vectorSum(airportLocation);

        return endOfRunway;
    }
    /**
     * Get the outer most point of runway one in relative terms located at (0, -Length)
     * @return a vector containing the endpoint of runway number one given in the world axis system
     */
    public Vector getRunwayOneEnd(){
        //first get the heading vector
        //then scale it to the length of the runway and add the location of the airport to get an absolute
        Vector heading = this.getHeadingVector();
        Vector airportLocation = this.getLocation();
        float runwayLength = this.getRunwayLength();
        //we first have to reverse the heading to get it pointing in the direction of runway number one
        //afterwards we scale it to the end of the runway (relative)
        Vector reverseHeading = heading.scalarMult(-1f);
        Vector endOfRunwayRelative = reverseHeading.normalizeToLength(runwayLength);
        Vector endOfRunway = endOfRunwayRelative.vectorSum(airportLocation);

        return endOfRunway;
    }

    /**
     * Calculates the distance between the id of the gate and the position that was specified
     * @param gate the gate to measure the distance to
     * @param position the current location to measure the distance to the gate with
     * @return the distance between the position of the gate and the provided position
     */
    public float distanceToGate(int gate, Vector position){
        //get the gates
        Map<Integer, Vector> gates = this.getGates();
        Vector gateLocation = gates.get(gate);
        //check for null pointer, throw error if
        if(gateLocation == null){
            throw new IllegalArgumentException("Gate not defined");
        }
        //if not null return the position between the gate and the provided position
        return gateLocation.distanceBetween(position);
    }

    /**
     * Getter for the width of the runway
     * @return a float containing the width of the runway
     */
    public float getRunwayWidth() {
        return runwayWidth;
    }

    /**
     * Getter for the length of the runway
     * @return a float containing the length of the runway
     */
    public float getRunwayLength() {
        return runwayLength;
    }

    /**
     * Getter for the location of the airport itself
     * @return the location of the airport
     */
    public Vector getLocation() {
        return location;
    }

    /**
     * Getter for the heading vector of the first runway of the airport
     * @return the heading of runway number one (normalized)
     */
    public Vector getHeadingVector() {
        return headingVector;
    }

    /**
     * The ID of the airport
     * @return the airport ID (a unique identifier)
     */
    public int getAirportID() {
        return airportID;
    }

    /**
     * Getter for the position of gate zero
     * @return the position of gate zero
     */
    public Vector getGateZeroPosition(){
        Map<Integer, Vector> gates = this.getGates();
        return gates.get(0);
    }

    /**
     * Getter for the position of gate one
     * @return the position of gate one (absolute)
     */
    public Vector getGateOnePosition(){
        Map<Integer, Vector> gates = this.getGates();
        return gates.get(1);
    }

    /**
     * Getter for the gate position with the corresponding gate index
     * @param gateIndex the index of the gate to get the location for
     * @return a vector containing the location of the gate in world axis system
     * note: the gates have number 0 to 1, if index out of range an exception will be thrown
     */
    public Vector getGateLocation(int gateIndex){
        return gates.get(gateIndex);
    }

    /**
     * The width of the runways, the same of all runways on the airport
     */
    private float runwayWidth;

    /**
     * The length of the runways of the airport (same for all the runways)
     */
    private float runwayLength;

    /**
     * Getter for the map containing all the gates of the airport
     * @return a map with the gate ID as key and the location as value (absolute location)
     */
    private  Map<Integer, Vector> getGates() {
        return gates;
    }

    /**
     * The location of the airport in the world
     */
    private Vector location;

    /**
     * The orientation of the airport in the world (the heading vector)
     * unit vector, points from the center of the airport to the runway with id 0
     */
    private Vector headingVector;

    /**
     * the unique identifier of the airport
     */
    private int airportID;

    /**
     * a map containing the id of the specified gate and its corresponding location
     */
    private Map<Integer, Vector> gates = new HashMap<>();

    @Override
    public String toString() {
        return "MapAirport " +
                "location: " + location +
                ", airportID: " + airportID;
    }
}

