import TestbedAutopilotInterface.*;
import internal.Helper.Vector;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Created by Martijn on 31/03/2018.
 * The main loop for the final phase of the project
 */
public class PhaseFourMain {

    public static void main(String[] args){
        //create the server (will automatically initialize the gui
        TestbedServer testBedServer = new TestbedServer(timeStep, stepsPerCycle, nbDrones, tcpPort, getAirportSpecs());
        AutopilotInitializer initializer = new AutopilotInitializer(host, tcpPort, nbDrones);
        //create the overseer
        AutopilotOverseer overseer = new AutopilotOverseer();
        //run the server
        Thread serverThread = new Thread(testBedServer);
        serverThread.start();
        //run the initializer
        initializer.initialize(overseer);
        //call the overseer (it wil continue on this thread)

        initOverseer(overseer);
    }

    private static void initOverseer(AutopilotOverseer overseer) {
        try {
            //create all the airports
            overseer.defineAirportParams(runwayLength, runwayWidth);
            for(AirportSpec spec: getAirportSpecs()){
                Vector position = spec.getPosition();
                Vector heading = spec.getPrimaryRunWay();
                overseer.defineAirport(position.getxValue(), position.getzValue(),
                        heading.getxValue(), heading.getzValue());
            }
            overseer.call();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static List<AirportSpec> getAirportSpecs(){
        //we will add two airports for now
        AirportSpec airportOne = new AirportSpec() {
            @Override
            public Vector getPosition() {
                return new Vector();
            }

            @Override
            public Vector getPrimaryRunWay() {
                return new Vector(0,0,-1);
            }

            @Override
            public float getRunwayWidth() {
                return runwayWidth;
            }

            @Override
            public float getRunwayLength() {
                return runwayLength;
            }
        };

        AirportSpec airportTwo = new AirportSpec() {
            @Override
            public Vector getPosition() {
                return new Vector(500,0,-1000);
            }

            @Override
            public Vector getPrimaryRunWay() {
                return new Vector(-1,0,-1);
            }
            @Override
            public float getRunwayWidth() {
                return runwayWidth;
            }

            @Override
            public float getRunwayLength() {
                return runwayLength;
            }
        };

        List<AirportSpec> specs = new ArrayList<>();
        specs.add(airportOne);
        specs.add(airportTwo);

        return specs;
    }

    //TODO add the packages
    private static Set<DeliveryPackage> getDeliveries(){
        return null;
    }

    private static int nbDrones = 2;
    private static int stepsPerCycle = 50;
    private static float timeStep = 0.001f;

    private static int tcpPort = 4242;
    private static String host = "localhost";

    private static float runwayLength = 280f;
    private static float runwayWidth = 10f;


}
