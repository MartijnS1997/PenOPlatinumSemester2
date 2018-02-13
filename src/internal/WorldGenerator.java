package internal;
import java.util.ArrayList;
import java.util.Random;
import gui.Cube;

/**
 * Generates a world with N cubes which all have a different HSV value
 * @author Jonathan
 * appended by Anthony
 */
public class WorldGenerator {
	
	/**
	 * Constructor 
	 * @param nbOfCubes amount of cubes the world will have
	 */
	public WorldGenerator(int nbOfCubes){
		if (! isValidNbOfCubes(nbOfCubes)){
			throw new IllegalArgumentException("The number of cubes is not > 0");
		}
		this.nbOfCubes = nbOfCubes;
	}
	
	/**
	 * Returns the amount of cubes the world will have
	 */
	public int getNbOfCubes(){
		return nbOfCubes;
	}

	/**
	 * A world needs to have at least 1 cube
	 * @param cubes amount of cubes the world has
	 * @return true if cubes > 0, false otherwise
	 */
	public boolean isValidNbOfCubes(int cubes){
		return cubes > 0;
	}
	
	/**
	 * Returns the initial position of the drone (0,0,0)
	 */
	public Vector getInitialPosition(){
		return this.initialPosition;
	}
	
	/**
	 * Creates nbOfCubes different colors for the cubes
	 * @return Arraylist of Vectors which contain HSV values
	 */
	public ArrayList<Vector> colorGenerator(){
		ArrayList<Vector> allColors = new ArrayList<Vector>();
		int n = getNbOfCubes();
		float h;
		float s ;
		float v = 1f;
		
		
		for (int i = 0; i < n; i++){
			//For a small amount of cubes, s is constant and h gets a smaller range (340)
			if (n <= 12){
				h = (float) 340/(i);
				s = 1f;
			}
			//For a large amount of cubes, s = 1 if i is even and s = 0.5 otherwise and h has its full range (360)
			else{
				h = (float) 359/(i);
				if (i % 2 == 0){
					s = 1f;
				}
				else{
					s = 0.5f;
				}
			}
			Vector color = new Vector(h,s,v);
			allColors.add(color);
		}
		return allColors;		
	}
	
	/**
	 * Creates nbOfCubes different colors for the cubes
	 * @return Arraylist of Vectors which contain HSV values
	 */
	public ArrayList<Vector> colorGenerator(int n){
		ArrayList<Vector> allColors = new ArrayList<Vector>();
		float h;
		float s ;
		float v = 1f;
		
		
		for (int i = 0; i < n; i++){
			//For a small amount of cubes, s is constant and h gets a smaller range (340)
			if (n <= 12){
				h = (float) 340/(i);
				s = 1f;
			}
			//For a large amount of cubes, s = 1 if i is even and s = 0.5 otherwise and h has its full range (360)
			else{
				h = (float) 359/(i);
				if (i % 2 == 0){
					s = 1f;
				}
				else{
					s = 0.5f;
				}
			}
			Vector color = new Vector(h,s,v);
			allColors.add(color);
		}
		return allColors;		
	}
	
	/**
	 * Generates a list with only red cubes (mostly for testing purpose)
	 */
	public ArrayList<Vector> redGenerator(){
		ArrayList<Vector> allColors = new ArrayList<Vector>();
		int n = getNbOfCubes();
		float h = 360;
		float s = 1;
		float v = 1;
		for (int i = 0; i < n; i++){
			Vector color = new Vector(h,s,v);
			allColors.add(color);
		}
		return allColors;	
	}
	
	
	/**
	 * Generates a position in the following range: [-10,10] following a Gaussian distribution
	 * with 0 as mean and 99% of the values laying in [-9,9]
	 */
	public float xPosGen(){
		Random r = new Random();	
		float val = (float) r.nextGaussian() * 3;
		return val;
	}
	
	/**
	 * Generates a position in the following range: [0,10] following a Gaussian distribution
	 * with 5 as mean and 99% of the values laying in [0.5,9.5]
	 */
	public float yPosGen(){
		Random r = new Random();	
		float val = (float) r.nextGaussian() * 1.5f + 5f;
		return val;
	}
	
	/**
	 * Generates a position in the following range: [-100,-10] following a Gaussian distribution
	 * with -55 as mean and 99% of the values laying in [-13,-97]
	 */
	public float zPosGen(){
		Random r = new Random();	
		float val = (float) r.nextGaussian() * 14 - 55;
		return val;
	}

	/**
	 * Generates a radius in the following range: [0,10] following a uniform distribution
	 */
	public float radiusGen(){
		Random r = new Random();
		float val = r.nextFloat() * 10;
		return val;
	}


	/**
	 * Generates an angle in the following range: [0,2PI] following a uniform distribution
	 */
	public float angleGen(){
		Random r = new Random();
		float val = r.nextFloat() * (float) (2*Math.PI);
		return val;
	}
	
	/**
	 * Generates a position vector using x-,y- and zPosGen
	 * When an x or y or z value is lower or higher than the boundary of the interval defined in those functions, 
	 * the x or y or z (respectively) value is set to lowest or highest (respectively) value allowed 
	 */
	public Vector positionGenerator(){;

		float r =  radiusGen();
		float a = angleGen();

		float x = (float) Math.cos(a) * r;
		float y = (float) Math.sin(a) * r;
		float z = -40f;
		
		if (x > 10){
			x = 10f;
		}
		if (x < -10){
			x = -10f;
		}
		if (y > 10){
			y = 10f;
		}
		if (y < 0){
			y = 0f;
		}
		if (z > -10){ 
			z = -10f;
		}
		if (z < -100){
			z = -100f;
		}
		
		Vector position = new Vector(x,y,z);
		return position;
	}
	
	/**
	 * Generates all positions for all cubes in the world 
	 * @return Arraylist with nbOfCubes position Vectors
	 */
	public ArrayList<Vector> allPositionsGenerator(){
		ArrayList<Vector> allPositions = new ArrayList<>();
		Vector initPos = getInitialPosition();
		Vector first = positionGenerator();
		int n = getNbOfCubes();
				
		//Position of the first cube
		Vector firstCubePos = new Vector(initPos.getxValue() + first.getxValue(),
				initPos.getyValue() + first.getyValue(), initPos.getzValue() + first.getzValue());
		allPositions.add(firstCubePos);
		
		//Position of cubes 2..n
		for (int i = 1; i < n; i++){
			Vector prevPos = allPositions.get(i-1);
			Vector newPos = positionGenerator();
			
			Vector cubePos = new Vector(prevPos.getxValue() + newPos.getxValue(),
					prevPos.getyValue() + newPos.getyValue(), prevPos.getzValue() + newPos.getzValue());
			allPositions.add(cubePos);
		}
		return allPositions;
	}
	
	/**
	 * Create a world with nbOfCubes cubes who all have a different HSV combination and a different position
	 * @return World with nbOfCubes different cubes
	 */
	public World createWorld(){
		int n = getNbOfCubes();
		ArrayList<Vector> allPositions = allPositionsGenerator();
		
		//uncomment the line below for cubes with different colors
		ArrayList<Vector> allColors = colorGenerator();
		
		//uncomment the line below for red cubes only
//		ArrayList<Vector> allColors = redGenerator();

		//the current objective is visit all
		World world = new World(World.VISIT_ALL_OBJECTIVE);
		Random r = new Random();	
		
		for (int i = 0; i < n; i++){
			int range = n-i;
			int index = r.nextInt(range);
			
			Vector pos = allPositions.get(i);
			Vector clr = allColors.get(index);
			allColors.remove(index);
			
			Block block = new Block(pos);
			Cube cube = new Cube(pos.convertToVector3f(), clr.convertToVector3f());
			cube.setSize(1f);
			block.setAssocatedCube(cube);
			
			world.addWorldObject(block);
			
		}
		return world;
	}
	
	
	/**
	 * The initial position of the drone
	 */
	private Vector initialPosition = new Vector(0,0,0);
	/**
	 * Variable for the amount of cubes in the world
	 */
	private int nbOfCubes;
	
	
	
}