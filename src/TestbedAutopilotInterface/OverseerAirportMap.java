package TestbedAutopilotInterface;

import internal.Helper.Vector;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by Martijn on 28/03/2018.
 * A class of maps of the world for the autopilot overseer
 */
public class OverseerAirportMap {
    /**
     * Constructor for an airport map
     * @param runwayWidth the width of the runway
     * @param runwayLength the length of the runway
     */
    public OverseerAirportMap(float runwayWidth, float runwayLength) {
        this.runwayWidth = runwayWidth;
        this.runwayLength = runwayLength;
    }

    /**
     * Getter for the airport corresponding to the given ID, returns null if no airport
     * is associated with the given ID
     * @param airportID the airport id used to find the requested airport
     * @return an airport object containing all the info needed to navigate
     */
    public MapAirport getAirport(int airportID){
        Map<Integer, MapAirport> airports = this.getAirportMap();
        return airports.get(airportID);
    }

    /**
     * Creates an airport at the given center and with the given heading for the first runway
     * and adds it to the airport map
     * @param center the center of the airport
     * @param headingVector the heading of runway zero
     * @return the ID given to the newly created airport
     */
    public int addAirport(Vector center, Vector headingVector){
        int airportID = this.getNextAvailableAirportID();
        float runwayWidth = this.getRunwayWidth();
        float runwayLength = this.getRunwayLength();
        Map<Integer, MapAirport> airportMap = this.getAirportMap();
        //create the new airport
        MapAirport airport = new MapAirport(center, headingVector, runwayWidth, runwayLength, airportID);
        //and add it to the map
        airportMap.put(airportID, airport);
        return airportID;
    }

    /**
     * Getter for the distance between two airports with the given ID
     * @param airportOne the first airport
     * @param airportTwo the second airport
     * @return the distance between the center of the airports
     */
    private float getDistanceBetweenAirports(int airportOne, int airportTwo){
        Map<Integer, MapAirport> airports = this.getAirportMap();
        MapAirport first = airports.get(airportOne);
        MapAirport second = airports.get(airportTwo);
        //try to get their location, catch for nullPointer en throw exception
        try{
            Vector firstLocation = first.getLocation();
            Vector secondLocation = second.getLocation();
            return firstLocation.distanceBetween(secondLocation);
        }catch(NullPointerException e){
            throw new IllegalArgumentException("The queried airport does not exist");
        }
    }

    /**
     * Returns the next available airport ID to be handed out, every ID will only
     * be handed out once (invoking this method will automatically change the value
     * of airport id)
     * @return the next available airport ID
     */
    private int getNextAvailableAirportID(){
        return nextAvailableAirportID++;
    }

    /**
     * Getter for the width of all runways on the map (only one possible)
     * @return the width of all the airports
     */
    private float getRunwayWidth() {
        return runwayWidth;
    }

    /**
     * Setter for the runway width of the airport (needed because the airports are only known after init)
     * @param runwayWidth the runway width
     */
    public void setRunwayWidth(float runwayWidth) {
        this.runwayWidth = runwayWidth;
    }

    /**
     * Setter for all the runway lengths on the map (only one possible)
     * @return the length of the runways
     */
    private float getRunwayLength() {
        return runwayLength;
    }

    /**
     * Setter for the runway length (needed because the airports are initialized only after the simulation started)
     * @param runwayLength the length of the runway
     */
    public void setRunwayLength(float runwayLength) {
        this.runwayLength = runwayLength;
    }

    /**
     * Getter for the entire map that contains all the airports currently present in the world
     * @return a map containing integers (ID) as the keys and an airport (with the corresponding ID) as value
     */
    private Map<Integer, MapAirport> getAirportMap() {
        return airportMap;
    }

    /**
     * The width of all the runways of all the airports on the map (as specified in the assignment)
     */
    private float runwayWidth;

    /**
     * The length for all the runways of all airports on the map (as specified in the assignment)
     */
    private float runwayLength;

    /**
     * The map containing all the airports currently present in the world (and that the overseer knows of)
     */
    private Map<Integer, MapAirport> airportMap = new HashMap<>();

    /**
     * The next available airport ID, given to the airport when added to the Map
     */
    private int nextAvailableAirportID = 0;

}
