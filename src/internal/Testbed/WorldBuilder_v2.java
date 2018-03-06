package internal.Testbed;

import internal.Autopilot.PathGenerator;
import internal.Helper.Vector;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;

import gui.Cube;

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
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
        Airport airport = new Airport(new Vector(0, 0.01f, -0));
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
     * Creates an empty world with only one drone in it and no cubes (for testing takeoff and tire physics)
     * @return a world only containing a drone and a floor
     */
    public World createWorld(){
    	
    	
        World world = new World(World.NO_OBJECTIVE);
        world.addWorldObject(this.getDroneBuilder_v2().createTestBounceDrone());
        Floor floor = new Floor(new Vector());
        world.addWorldObject(floor);
//        Airport airport = new Airport(new Vector());
//        world.addWorldObject(airport);

        
        // For pathgenerator testing purposes
        /*
        PathGenerator pathGenerator = new PathGenerator();
        Vector position = new Vector(150f, 10f, 0f);
        Vector destination = new Vector(0f, 0f, 0f);
        Vector velocity = new Vector(0f, 0f, -30f);
        velocity = velocity.rotateAroundYAxis(-(float)Math.PI/8);
        pathGenerator.generateLandingPath(position, velocity, destination);
        
        Vector firstColor = new Vector(80f, 80f, 80f);
        Block firstBlock = new Block(position);
        Cube firstCube = new Cube(position.convertToVector3f(), firstColor.convertToVector3f(), true);
        firstCube.setSize(5f);
        firstBlock.setAssocatedCube(firstCube);
        world.addWorldObject(firstBlock);
        
        Vector secondPosition = position.vectorSum(velocity);
        Vector secondColor = new Vector(50f, 50f, 50f);
        Block secondBlock = new Block(secondPosition);
        Cube secondCube = new Cube(secondPosition.convertToVector3f(), secondColor.convertToVector3f(), true);
        secondCube.setSize(5f);
        secondBlock.setAssocatedCube(secondCube);
        world.addWorldObject(secondBlock);
        
        for (Vector blockPos : pathGenerator.getPath()) {
        	Vector color = new Vector(10f, 10f, 10f);
            Block block = new Block(blockPos);
            Cube cube = new Cube(blockPos.convertToVector3f(), color.convertToVector3f(), true);
            cube.setSize(2f);
            block.setAssocatedCube(cube);
            world.addWorldObject(block);
        }
        */
        // END for pathgenerator testing purposes
        return world;
    }

    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;
}
