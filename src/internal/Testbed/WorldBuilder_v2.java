package internal.Testbed;

import internal.Helper.Vector;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;

/**
 * Created by Martijn on 26/02/2018.
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
        return world;
    }

    public DroneBuilder_v2 getDroneBuilder_v2() {
        return droneBuilder_v2;
    }

    private DroneBuilder_v2 droneBuilder_v2;
    private final int NB_OF_BLOCKS = 5;
}
