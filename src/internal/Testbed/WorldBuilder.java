package internal.Testbed;


import java.io.IOException;
import java.util.List;


import internal.Helper.Parser;
import internal.Helper.Vector;
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
        World world = new World();
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


    		}
    	}
        
        //world.addWorldObject(block1);
        this.setDrone(new DroneBuilder(true).createDrone(config));
        world.addDrone(this.getDrone());

        
        WorldAirport airport = new WorldAirport(new Vector(0, 0.01f, -0));
        //world.addAirport(airport);

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