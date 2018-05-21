import TestbedAutopilotInterface.*;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import TestbedAutopilotInterface.Overseer.DeliveryPackage;
import TestbedAutopilotInterface.SimulationSetup.AirportSpec;
import TestbedAutopilotInterface.SimulationSetup.DroneSpec;
import TestbedAutopilotInterface.SimulationSetup.SimulationEnvironment;
import TestbedAutopilotInterface.SimulationSetup.SimulationGen;
import internal.Helper.Vector;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static java.lang.Math.PI;
import static java.lang.Math.signum;

/**
 * Created by Martijn on 31/03/2018.
 * The main loop for the final phase of the project
 */
public class PhaseFourMain {

    public static void main(String[] args) throws Exception {
        //clear the logs by writing empty lines
        Files.write(Paths.get("errorLog.txt"), "".getBytes());
        Files.write(Paths.get("trajectoryLog.txt"), "".getBytes());
        Files.write(Paths.get("vectorErrorLog.txt"), "".getBytes());

        //create a simulation generator
        SimulationGen generator = new SimulationGen(worldXSize, worldZSize);
        generator.setAirportRunwayLength(runwayLength);
        generator.setAirportRunwayWidth(runwayWidth);
        //set the seed so we generate the same world
        generator.setRandomSeed(0);
        //create the simulation environment
//        SimulationEnvironment environment = generator.generateBigWorld();
        SimulationEnvironment environment = generator.generate4AirportWorld();
        // generator.generateGridWorld(gridWorldRows, gridWorldColumns);
        //print the environment
        System.out.println(SimulationGen.environmentToString(environment));
        //generate the overseer
        AutopilotOverseer overseer = SimulationGen.generateOverseer(environment);
        //create the server
        TestbedServer testbedServer = generator.generateTestbedServer(environment, overseer);
        testbedServer.setPlaybackSpeed(playbackSpeed);
        //create the autopilot initializer
        AutopilotInitializer initializer = new AutopilotInitializer(host, tcpPort, generator.getNbDrones(), overseer);
        //run server (will also initiate gui
        Thread serverThread = new Thread(testbedServer);
        serverThread.start();
        //run the initializer
        Thread autopilotInitializerThread = new Thread(initializer);
        autopilotInitializerThread.start();
        //call the overseer (it will continue on this thread)
        overseer.call();
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

    /**
     * Gets the specifications for the airports in the world (substitute by generator functions later on)
     * @return a list containing all the airports to add to the world
     */
    private static List<AirportSpec> getAirportSpecs(){
        //we will add two airports for now
        AirportSpec airportOne = new AirportSpec() {
            @Override
            public Vector getPosition() {
                return airportOneLocation;
            }

            @Override
            public Vector getPrimaryRunWay() {
                return airportOneHeading;
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
                return airportTwoLocation;
            }

            @Override
            public Vector getPrimaryRunWay() {
                return airportTwoHeading;
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

    /**
     * Gets the specifications for the drones in the world (substitute implementation by a generator)
     * @return a list containing the specs for the drones to add
     */
    private static List<DroneSpec> getDronesSpecs(){
        DroneSpec drone1 = new DroneSpec() {
            @Override
            public Vector getDronePosition() {
                return droneOneLocation;
            }

            @Override
            public Vector getDroneOrientation() {
                return droneOneOrientation;
            }
        };

        DroneSpec drone2 = new DroneSpec() {
            @Override
            public Vector getDronePosition() {
                return droneTwoLocation;
            }

            @Override
            public Vector getDroneOrientation(){
                return droneTwoOrientation;
            }
        };

        List<DroneSpec> droneSpecs = new ArrayList<>();
        droneSpecs.add(drone1);
        droneSpecs.add(drone2);

        return droneSpecs;
    }

    //specify world parameters
    private static float worldXSize = 2000;
    private static float worldZSize = 2000;
    private static int nbAirports = 8;
    private static int nbPackages = 4;

    //TODO add/implement the packages
    private static Set<DeliveryPackage> getDeliveries(){
        return null;
    }


    private static int stepsPerCycle = 50;
    private static int stepsPerSubCycle = 10;
    private static float timeStep = 0.001f;
    private static int playbackSpeed = 1000;

    private static int tcpPort = 4242;
    private static String host = "localhost";

    private static float runwayLength = 400f;
    private static float runwayWidth = 15f;


    //TODO create a random map generator and drone locator

    //variables used to initialize the airports and the drones
    private static Vector airportOneLocation = new Vector();
    private static Vector airportOneHeading = new Vector(0,0,-1);

    private static Vector airportTwoLocation = new Vector(500,0,-1000);
    private static Vector airportTwoHeading = new Vector(-1,0,-1);

    private static Vector droneOneLocation = new Vector(0,1.20f, -10);
    private static Vector droneOneOrientation = new Vector(0,0,0);

    private static Vector droneTwoLocation = new Vector(500 - 20, 1.20f, -1000-20);
    private static Vector droneTwoOrientation = new Vector((float) (PI/4), 0,0);


}
