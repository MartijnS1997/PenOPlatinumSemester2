package internal.Autopilot;

import internal.Testbed.FlightRecorder;

import static java.lang.Math.*;

/**
 * Created by Martijn on 30/10/2017.
 * Appended and edited by Anthony Rath� on 6/11/2017
 * A class of Autopilot Controllers
 */
public abstract class AutoPilotFlightController extends Controller{


	/**
     * Constructor for the autopilotController
     * @param autopilot
     */
    public AutoPilotFlightController(AutoPilot autopilot){
        super(autopilot);
        
    }



    private void logControlActions(ControlOutputs outputs, String controlString){

        controlString += "Left wing inclination: "  + outputs.getLeftWingInclination()*RAD2DEGREE + "\n";
        controlString += "Right wing inclination: " + outputs.getRightWingInclination()*RAD2DEGREE + "\n";
        controlString += "Horizontal stabilizer inclination: " + outputs.getHorStabInclination()*RAD2DEGREE + "\n";
        controlString += "Vertical wing inclination" + outputs.getVerStabInclination()*RAD2DEGREE + "\n";

        // write the controls to the recorder
        FlightRecorder recorder = this.getFlightRecorder();
        recorder.appendControlLog(controlString);
    }

    /**
     * determines the largest value of both entries, if both are negative an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the largest entry of both
     * @throws IllegalArgumentException thrown if both entries are negative
     */
    private float getMaxPos(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 >= entry2 && entry1 >= 0){
            return entry1;
        }else if(entry2 >= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /**
     * determines the smallest value of both entries, if both are positive an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the smallest entry of both
     * @throws IllegalArgumentException thrown if both entries are positive
     */
    private float getMinNeg(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 <= entry2 && entry1 <= 0){
            return entry1;
        }else if(entry2 <= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /*
    Getters and setters
     */


    /**
     * get the flight recorder of the drone
     * @return
     */
    public FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    /**
     * Setter for the flight recorder
     * @param flightRecorder
     */
    public void setFlightRecorder(FlightRecorder flightRecorder) {
        this.flightRecorder = flightRecorder;
    }

    

    /*
    Getters and setters
     */

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }

    /**
     * Get the PID used for the corrections on the x-coordinate inputs
     * @return the pid controller
     */
    public PIDController getxPID() {
        return xPID;
    }

    /**
     * Get the PID used for the corrections on the y-coordinate inputs
     * @return
     */
    public PIDController getyPID() {
        return yPID;
    }

    
    private FlightRecorder flightRecorder;
    private PIDController xPID = new PIDController(1.f, 0.2f, 0.2f);
    private PIDController yPID = new PIDController(1.f, 0.2f, 0.2f);


    private static final float STANDARD_THRUST = 32.859283f *2;
    private static final float RAD2DEGREE = (float) (180f/ PI);


    



    /**
     * A class of PID controllers
     */


}
