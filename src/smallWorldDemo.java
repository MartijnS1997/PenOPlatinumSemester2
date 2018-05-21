import TestbedAutopilotInterface.AutopilotInitializer;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import TestbedAutopilotInterface.SimulationSetup.SimulationEnvironment;
import TestbedAutopilotInterface.SimulationSetup.SimulationGen;
import TestbedAutopilotInterface.TestbedServer;

import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Created by Martijn on 21/05/2018.
 * A class used to demo a small world
 */
public class smallWorldDemo {
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


    //specify world parameters
    private static float worldXSize = 2000;
    private static float worldZSize = 2000;



    private static int stepsPerCycle = 50;
    private static int stepsPerSubCycle = 10;
    private static float timeStep = 0.001f;
    private static int playbackSpeed = 25;

    private static int tcpPort = 4242;
    private static String host = "localhost";

    private static float runwayLength = 400f;
    private static float runwayWidth = 15f;
}
