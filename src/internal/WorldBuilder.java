package internal;


import java.io.IOException;
import java.util.List;

import gui.Cube;
import gui.Tile;
import math.Vector3f;
/*
*//**
 * Created by Martijn on 26/10/2017.
 * a class to build a world, set the standard configuration here.
 */
public class WorldBuilder {

    public WorldBuilder() {
        //do nothing
    }

    public World createWorld(String config) throws IOException{
        World world = new World(World.VISIT_ALL_OBJECTIVE);
    	if (!isPredefWorld()) {
    		world = this.getWorldGenerator().createWorld();
    	}
    	else {
    	    Parser parser = new Parser(this.getPredefDirectory());
    		//List<Vector> positions = parser.getCoordinates();
    		//List<Vector> colors = this.getWorldGenerator().colorGenerator(positions.size());
            List<Vector> data = parser.getBlockData();
            for (int i = 0 ; i < data.size(); i += 2) {
                Vector position = data.get(i);
                Vector color = data.get(i+1);
                Block block = new Block(position);
                Cube cube = new Cube(position.convertToVector3f(), color.convertToVector3f(), true);
                cube.setSize(1f);
                block.setAssociatedGraphicsObject(cube);
                world.addWorldObject(block);
    		}
            
            int n = 101; // n moet oneven zijn
            float nx = 10;
            float nz = 10;
            Floor floor = new Floor(new Vector(-n*nx/2, 0, -n*nz + 50f));
            for (int i = 0; i < n*n; i++) {
            	Vector delta = new Vector(nx*(i%n), 0, nz*(i/n));
            	Vector position = delta.vectorSum(floor.getPosition());
                Vector color = new Vector((float) (60.0+(i%2)*60), 1, 0.6f);
            	Tile tile = new Tile(position.convertToVector3f(), color.convertToVector3f());
            	tile.setSize(new Vector(nx, 0, nz));
            	floor.setAssociatedGraphicsObject(tile);
            }
            world.addWorldObject(floor);
    	}
        
        //world.addWorldObject(block1);
        this.setDrone(new DroneBuilder(true).createDrone(config));
        world.addWorldObject(this.getDrone());

        return world;
    }


    public boolean isPredefWorld() {
        return predefWorld;
    }

    public void setPredefWorld(boolean predefWorld) {
        this.predefWorld = predefWorld;
    }

    public String getPredefDirectory() {
        return predefDirectory;
    }

    public void setPredefDirectory(String predefDirectory) {
        this.predefDirectory = predefDirectory;
    }

    public Drone getDrone() {
        return drone;
    }

    public void setDrone(Drone DRONE) {
        this.drone = DRONE;
    }

    public WorldGenerator getWorldGenerator() {
        return worldGenerator;
    }

    private final static int numberOfBlocks = 5;
    private boolean predefWorld = false;
    private String predefDirectory = "src/internal/blockCoordinates.txt";
    private final WorldGenerator worldGenerator = new WorldGenerator(numberOfBlocks);
    private Drone drone;


}