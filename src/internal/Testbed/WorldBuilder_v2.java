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
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
        return world;
    }

    /**
     * Generates a world with only one drone present, used for test flights
     * @return a world only containing one drone
     */
    public World createFlightTestWorld(){
        World world = new World(World.NO_OBJECTIVE);
        Map<Vector, Float> droneConfig = new HashMap<>();
        droneConfig.put(new Vector(0,20f,0), 0f); //drone at 5m height facing forward
        List<Drone> droneList = this.getDroneBuilder_v2().createDrones(droneConfig);
        world.addDrones(droneList);
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
        int nbPoints = 50;
        Random random = new Random(1);
        for(int i = 1; i<nbPoints; i++){
//            float turningRadius = 1500;
//            float x = (float) (-turningRadius*cos(PI/nbPoints * i) + turningRadius);
//            float z = (float) (-sin(PI/nbPoints*i)*turningRadius);


            Vector pos = new Vector(0, 30, -i*60);
            Cube cube = new Cube(pos.convertToVector3f(), new Vector(	60.0f,  1.0f,  1.0f).convertToVector3f(), true );
            cube.setSize(5f);
            Block block = new Block(pos);
            block.setAssocatedCube(cube);
            world.addWorldObject(block);
        }
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
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
        return world;
    }

    /**
     * Creates an empty world with only one drone in it and no cubes (for testing takeoff and tyre physics)
     * @return a world only containing a drone and a floor
     */
    public World createWorld(){
        World world = new World(World.NO_OBJECTIVE);
        world.addWorldObject(this.getDroneBuilder_v2().createTestBounceDrone());
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector(0, 0.5f, 0));
        world.addWorldObject(airport);
        return world;
    }

    /**
     * Create a world to test the takeoff of a drone
     * @return a world suited to test the takeoff of a drone
     */
    public World createTakeoffWorld(){
        World world = new World(World.NO_OBJECTIVE);
        world.addWorldObject(this.getDroneBuilder_v2().createTestBounceDrone());
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
        world.setApproxPath(new Path() {
            @Override
            public float[] getX() {
                return new float[]{0.f};
            }

            @Override
            public float[] getY() {
                return new float[]{40f};
            }

            @Override
            public float[] getZ() {
                return new float[]{-200f};
            }
        });

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

    public World createLandingWorld(){
        Map<Vector, Float> droneConfig = new HashMap<>();
        droneConfig.put(new Vector(0,50f,300), 0f);
        Drone drone = this.getDroneBuilder_v2().createDrones(droneConfig).get(0);
        drone.setOrientation(new Vector(0,(float) 0, (float)0));
        //set the drone velocity
        Vector droneAxisVel = new Vector(0,0,-60);
        Vector worldAxisVel = PhysXEngine.droneOnWorld(droneAxisVel, drone.getOrientation());
        drone.setVelocity(worldAxisVel);
        World world = new World(World.NO_OBJECTIVE);
        world.addWorldObject(drone);
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
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
    }


    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;

}
