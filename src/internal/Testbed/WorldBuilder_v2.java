package internal.Testbed;

import gui.Cube;
import internal.Autopilot.AutopilotLandingController;
import internal.Helper.Vector;
import math.Vector3f;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

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
        World world = new WorldGenerator(NB_OF_BLOCKS).createWorld();
        world.addDrones(droneList);
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
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
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
//        Vector pos1 = new Vector(0,25f, -60f);
//        Vector pos2 = new Vector(0,30f, -120f);
//        Vector pos3 = new Vector(0,20f, -180f);
//        Cube cube1 = new Cube(pos1.convertToVector3f(), new Vector(	60.0f,  1.0f,  1.0f).convertToVector3f(), true );
//        Cube cube2 = new Cube(pos2.convertToVector3f(), new Vector(60.0f, 1.0f, 1.0f).convertToVector3f(), true);
//        Cube cube3 = new Cube(pos3.convertToVector3f(), new Vector(60f, 1.0f, 1.0f).convertToVector3f(), true);
//        Block block1 = new Block(pos1);
//        Block block2 = new Block(pos2);
//        Block block3 = new Block(pos3);
//        block1.setAssocatedCube(cube1);
//        block2.setAssocatedCube(cube2);
//        block3.setAssocatedCube(cube3);
//        world.addWorldObject(block1);
//        world.addWorldObject(block2);
//        world.addWorldObject(block3);
        int nbPoints = 50;
        for(int i = 1; i<nbPoints; i++){
//            float turningRadius = 1500;
//            float x = (float) (-turningRadius*cos(PI/nbPoints * i) + turningRadius);
//            float z = (float) (-sin(PI/nbPoints*i)*turningRadius);
            Vector pos = new Vector(i*5, 20, -i*60);
            Cube cube = new Cube(pos.convertToVector3f(), new Vector(	60.0f,  1.0f,  1.0f).convertToVector3f(), true );
            cube.setSize(5f);
            Block block = new Block(pos);
            block.setAssocatedCube(cube);
            world.addWorldObject(block);
        }

        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        return world;
    }


    /**
     * Generates a world with only one drone present, placed on the ground, used for test flights
     * @return a world only containing one drone
     */
    public World createGroundDroneTestWorld(){
        World world = new World(World.NO_OBJECTIVE);
        Map<Vector, Float> droneConfig = new HashMap<>();
        droneConfig.put(new Vector(0,1.0f,0), 0f); //drone at 5m height facing forward
        List<Drone> droneList = this.getDroneBuilder_v2().createDrones(droneConfig);
        world.addDrones(droneList);
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
//        Vector pos1 = new Vector(0,25f, -60f);
//        Vector pos2 = new Vector(0,30f, -120f);
//        Vector pos3 = new Vector(0,20f, -180f);
//        Cube cube1 = new Cube(pos1.convertToVector3f(), new Vector(	60.0f,  1.0f,  1.0f).convertToVector3f(), true );
//        Cube cube2 = new Cube(pos2.convertToVector3f(), new Vector(60.0f, 1.0f, 1.0f).convertToVector3f(), true);
//        Cube cube3 = new Cube(pos3.convertToVector3f(), new Vector(60f, 1.0f, 1.0f).convertToVector3f(), true);
//        Block block1 = new Block(pos1);
//        Block block2 = new Block(pos2);
//        Block block3 = new Block(pos3);
//        block1.setAssocatedCube(cube1);
//        block2.setAssocatedCube(cube2);
//        block3.setAssocatedCube(cube3);
//        world.addWorldObject(block1);
//        world.addWorldObject(block2);
//        world.addWorldObject(block3);
        int nbPoints = 50;
        for(int i = 1; i<nbPoints; i++){
//            float turningRadius = 1500;
//            float x = (float) (-turningRadius*cos(PI/nbPoints * i) + turningRadius);
//            float z = (float) (-sin(PI/nbPoints*i)*turningRadius);
            Vector pos = new Vector(i*5, 20, -i*60);
            Cube cube = new Cube(pos.convertToVector3f(), new Vector(	60.0f,  1.0f,  1.0f).convertToVector3f(), true );
            cube.setSize(5f);
            Block block = new Block(pos);
            block.setAssocatedCube(cube);
            world.addWorldObject(block);
        }

        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
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
//        Floor floor = new Floor(new Vector());
//        world.addWorldObject(floor);
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
        Airport airport = new Airport(new Vector());
        world.addWorldObject(airport);
        return world;
    }

    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;
}
