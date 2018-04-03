package internal.Testbed;

import TestbedAutopilotInterface.AirportSpec;
import TestbedAutopilotInterface.DroneSpec;
import internal.Helper.Vector;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;

/**
 * Created by Martijn on 26/02/2018.
 * A class that build a world with the desired parameters
 */
public class WorldBuilder_v2 {

    public WorldBuilder_v2(){
        this.droneBuilder_v2 = new DroneBuilder_v2();
    }

    /**
     * Creates a world with the reach cube objective
     * @param droneConfig the configuration of the drone
     * @return a world containing the drones
     */
    public World createWorld(Map<Vector, Float> droneConfig){
        List<Drone> droneList = this.getDroneBuilder_v2().createDrones(droneConfig);
        //World world = new WorldGenerator(NB_OF_BLOCKS).createWorld(new Vector(), World.VISIT_ALL_OBJECTIVE);
        World world = new World();
        world.addDrones(droneList);
        droneList.get(0).setVelocity(new Vector(0,0,-50f));
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
        initSurroundings(world);
        return world;
    }


    /**
     * Create a world with no objective that shares the thread pool with the server
     * @param droneConfig the configuration of the drone containing the position and orientation
     * @param threads the executor service that shares the thread pool with the server
     * @return a world containing the drones
     */
    public World createWorld(Map<Vector, Float> droneConfig, ExecutorService threads){
        List<Drone> droneList = this.getDroneBuilder_v2().createDrones(droneConfig);
        World world = new World(threads);
        world.addDrones(droneList);
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
        initSurroundings(world);
        return world;
    }

//
//    public World createTotalFlightWorld(){
//        WorldGenerator generator = new WorldGenerator(NB_OF_BLOCKS);
//        Vector startCubes = new Vector(0, 30f, -700f);
//        World world = generator.createWorld(startCubes, World.FLIGHT_OBJECTIVE);
//        world.addWorldObject(this.getDroneBuilder_v2().createTestBounceDrone());
//
//        initSurroundings(world);
//
//        return world;
//    }

    /**
     * Creates a world with multiple drones (same nb of drones as threads
     * @param droneThreads the threads used to simulate the drones
     * @param drones the specifications for the drones to add to the world (no checks performed)
     * @param airports the airports: the arrays have to contain a position (entry 0) and
     *                 heading (entry 1)
     * @return the world created with the given thread pool and the airports (are converted to WorldAirport)
     */
    public World createMultiDroneWorld(ExecutorService droneThreads, List<DroneSpec> drones, List<AirportSpec> airports){
        World world = new World(droneThreads);
        for(AirportSpec airport: airports){
            //set the width and length of the airports (all calls will, be ignored after the second one
            world.setRunwayWidth(airport.getRunwayWidth());
            world.setRunwayLength(airport.getRunwayLength());
            Vector position = airport.getPosition();
            Vector heading = airport.getPrimaryRunWay();
            world.addAirport(position, heading);
        }
        //create the drones
        DroneBuilder_v2 builder_v2 = this.getDroneBuilder_v2();
        List<Drone> droneList = builder_v2.createDrones(drones);
        world.addDrones(droneList);
        return world;
    }


    /**
     * Initializes the surroundings of the world (ground and runway)
     * @param world the world to initialize
     */
    private static void initSurroundings(World world){
        world.addAirport(new Vector(0,0.5f, 20), new Vector(0,0,-1));
    }


    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;

}
