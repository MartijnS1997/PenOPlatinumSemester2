package internal.Autopilot;

/**
 * Created by Martijn on 13/03/2018.
 * an enumeration of flight states for an autopilot
 * TAKEOFF: the autopilot objective is to takeoff
 * FLIGHT: the autopilot objective is passing by all the cubes in the world
 * WAY_POINT: the autopilot objective is following way points for flying back to the takeoff point
 * LANDING: the autopilot objective is getting the drone safely to ground again
 * TAXIINg: the autopilot objective is taxiing to a specified point on the ground
 */
public enum AutopilotState {
    INIT_FLIGHT, TAKEOFF, FLIGHT, LANDING, TAXIING_TO_GATE, TAXIING_TO_RUNWAY;

    public static String getString(AutopilotState state){
        switch(state){
            case INIT_FLIGHT:
                return INIT_FLIGHT_STRING;
            case TAKEOFF:
                return TAKEOFF_STRING;
            case FLIGHT:
                return FLIGHT_STRING;
            case LANDING:
                return LANDING_STRING;
            case TAXIING_TO_GATE:
                return TAXIING_TO_GATE_STRING;
            case TAXIING_TO_RUNWAY:
                return TAXIING_TO_RUNWAY_STRING;
            default:
                return INVALID_STATE_STRING;
        }
    }


    private final static String INIT_FLIGHT_STRING = "initializing flight" ;
    private final static String TAKEOFF_STRING = "takeoff" ;
    private final static String FLIGHT_STRING = "flight" ;
    private final static String LANDING_STRING = "landing" ;
    private final static String TAXIING_TO_GATE_STRING = "taxiing to gate" ;
    private final static String TAXIING_TO_RUNWAY_STRING = "taxiing to runway" ;
    private final static String INVALID_STATE_STRING = "invalid state";
}
