package internal.Testbed;

import AutopilotInterfaces.Path;
import gui.Cube;
import internal.Autopilot.AutopilotLandingController;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;
import math.Vector3f;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;

import static java.lang.Math.*;

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
        World world = new WorldGenerator(NB_OF_BLOCKS).createWorld(new Vector(), World.VISIT_ALL_OBJECTIVE);
        world.addDrones(droneList);
        droneList.get(0).setVelocity(new Vector(0,0,-50f));
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector(0, 0.5f, 20));
        world.addWorldObject(airport);
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
        World world = new World(World.NO_OBJECTIVE, threads);
        world.addDrones(droneList);
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector(0, 0.5f, 20));
        world.addWorldObject(airport);
        return world;
    }


    public World createTotalFlightWorld(){
        WorldGenerator generator = new WorldGenerator(NB_OF_BLOCKS);
        Vector startCubes = new Vector(0, 30f, -700f);
        World world = generator.createWorld(startCubes, World.FLIGHT_OBJECTIVE);
        world.addWorldObject(this.getDroneBuilder_v2().createTestBounceDrone());

        initSurroundings(world);

        return world;
    }


    /**
     * Initializes the surroundings of the world (ground and runway)
     * @param world the world to initialize
     */
    private static void initSurroundings(World world){
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector(0, 0.5f, 20));
        world.addWorldObject(airport);
    }


    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;

}
