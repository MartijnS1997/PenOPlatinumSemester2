package internal.Testbed;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import gui.Cube;
import internal.Autopilot.Path;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

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
	 * Generates a position in the following range: [minX,maxX] following a Gaussian distribution
	 * with meanXPos as mean and 99% of the values laying in [meanXPos-3*stdDevX,meanXPos+3*stdDevX]
	 */
	public float xPosGen(){
		Random r = new Random();
		float val = (float) r.nextGaussian() * getStdDevX() + getMeanX();
		return val;
	}

	/**
	 * Generates a position in the following range: [minY,maxY] following a Gaussian distribution
	 * with meanYPos as mean and 99% of the values laying in [meanYPos-3*stdDevY,meanYPos+3*stdDevY]
	 */
	public float yPosGen(){
		Random r = new Random();
		float val = (float) r.nextGaussian() * getStdDevY() + getMeanY();
		return val;
	}

	/**
	 * Generates a position in the following range: [minZ,maxZ] following a Gaussian distribution
	 * with meanZPos as mean and 99% of the values laying in [meanZPos-3*stdDevZ,meanZPos+3*stdDevZ]
	 */
	public float zPosGen(){
		Random r = new Random();
		float val = (float) r.nextGaussian() *  getStdDevZ() + getMeanZ();
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
		float val = r.nextFloat() * (float) (2* PI);
		return val;
	}

	/**
	 * Generates a position vector using x-,y- and zPosGen
	 * When an x or y or z value is lower or higher than the boundary of the interval defined in those functions,
	 * the x or y or z (respectively) value is set to lowest or highest (respectively) value allowed
	 */
	public Vector positionGenerator(){;

//		float r =  radiusGen();
//		float a = angleGen();
//
//		float x = (float) Math.cos(a) * r;
//		float y = (float) Math.sin(a) * r;
//		float z = -40f;

		float x = xPosGen();
		float y = yPosGen();
		float z = zPosGen();


		if (x > getMaxX()){
			x = getMaxX();
		}
		if (x < getMinX()){
			x = getMinX();
		}
		if (y > getMaxY()){
			y = getMaxY();
		}

		//cube can't go through the ground, so minY has to be >= 2.5m (+2m so they aren't on the ground)
		if (y < getMinY()){
			y = getMinY();
		}
		if (z > getMaxZ()){
			z = getMaxZ();
		}
		if (z < getMinZ()){
			z = getMinZ();
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
	 * Generates an error in the following range: [minError,maxError] following a Gaussian distribution
	 * with meanError as mean and 99% of the values laying in [meanError-3*stdDevError,meanError+3*stdDevError]
	 */
	public float errorGen(){
		Random r = new Random();
		//float val = (float) r.nextGaussian() *  getStdDevError() + getMeanError();
		float sgn = r.nextFloat();
		float val = 0;
		if (sgn > 0.5)
			val = r.nextFloat() * getMaxError();
		else
			val = r.nextFloat() * getMinError();
		return val;
	}



	/**
	 * Add an error on the x,y,z values of every cube position
	 * @param positions all cube positions in the world
	 * @return all cube positions with an added error
	 */
	public ArrayList<Vector> generateErrorOnCubePositions(ArrayList<Vector> positions){

		ArrayList<Vector> newPositions = new ArrayList<Vector>();
		int n = positions.size();


		for (int i = 0; i < n; i++){

			float eX = errorGen();
			float eY = errorGen();
			float eZ = errorGen();

			if (eX > getMaxError()) eX = getMaxError();
			if (eX < getMinError()) eX = getMinError();
			if (eY > getMaxError()) eY = getMaxError();
			if (eY < getMinError()) eY = getMinError();
			if (eZ > getMaxError()) eZ = getMaxError();
			if (eZ < getMinError()) eZ = getMinError();


			Vector newPos = new Vector(positions.get(i).getxValue()+eX,positions.get(i).getyValue()+eY,
					positions.get(i).getzValue()+eZ);

			newPositions.add(newPos);
		}

		return newPositions;
	}


	/**
	 * Create a world with nbOfCubes cubes who all have a different HSV combination and a different position
	 * @return World with nbOfCubes different cubes
	 */
	public World createWorld(){
		int n = getNbOfCubes();
		ArrayList<Vector> allPositions = allPositionsGenerator();
		return createWorldWithPath(allPositions);
	}

	public List<Vector> createPath(){
		return allPositionsGenerator();
	}

	private  AutopilotInterfaces.Path createPathObject(List<Vector> path){
		int nbOfCubes = getNbOfCubes();
		float xPos[] = new float[nbOfCubes];
		float yPos[] = new float[nbOfCubes];
		float zPos[] = new float[nbOfCubes];
		int index = 0;
		for(Vector vector: path){
			xPos[index] = vector.getxValue();
			yPos[index] = vector.getyValue();
			zPos[index] = vector.getzValue();
		}

		return new AutopilotInterfaces.Path(){
			@Override
			public float[] getX() {
				return xPos;
			}

			@Override
			public float[] getY() {
				return yPos;
			}

			@Override
			public float[] getZ() {
				return zPos;
			}
		};
	}

	/**
	 * Creates a world on the given path, the path that is inputted is randomized
	 * randomization means: placed within a probability sphere of 5m radius with a random heading and roll
	 * @param path the path to be randomized
	 * @return a world containing blocks that are placed random among the provided path
	 */
	public World createWorldWithPath(List<Vector> path){
		int n = getNbOfCubes();
		ArrayList<Vector> allPositions = pathRandomizer(path) ;
		ArrayList<Vector> errorPosition = generateErrorOnCubePositions(allPositions);
		setErroredPositions(errorPosition);

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
			Cube cube = new Cube(pos.convertToVector3f(), clr.convertToVector3f(), true);
			cube.setSize(5f);
			block.setAssocatedCube(cube);

			world.addWorldObject(block);

		}
		AutopilotInterfaces.Path approxPath = this.createPathObject(path);
		world.setApproxPath(approxPath);

		return world;
	}

	/**
	 * Randomizes the given path such that the cubes are within a 5m radius (gaussian distribution capped on 5)
	 * @param path the path to be randomized
	 * @return a list of path positions with each entry within a 5meter radius of the initial position
	 */
	public static ArrayList<Vector> pathRandomizer(List<Vector> path){
		float maxRadius = 5f;
		float maxAngle = (float) (2*PI);
		float randomOffset = 0.5f;
		//generate a vector with modulus 5 and a random pitch and heading
		//transform it back to the world later
		Random random = new Random(System.currentTimeMillis()%25785);
		//initialize the return path
		ArrayList<Vector> randomizedPath = new ArrayList<>();

		for(Vector pathElem: path) {
			//get the random variables
			float modulus = (float) min(abs(maxRadius * (random.nextGaussian()-randomOffset)), 1.0f);
			float heading = (float) ((random.nextFloat() - randomOffset) * maxAngle);
			float pitch = (float) ((random.nextFloat() - randomOffset) * maxAngle);
			//the modulus of the sphere
			Vector modulusVector = new Vector(0, 0, modulus);
			//get a random position within the sphere
			Vector orientationVector = new Vector(heading, pitch, 0);
			//create the random position by mapping the angle on the world
			Vector randomPos = PhysXEngine.droneOnWorld(modulusVector, orientationVector);

			randomizedPath.add(randomPos);

		}

		return randomizedPath;

	}



	/**
	 * The initial position of the drone
	 */
	private Vector initialPosition = new Vector(0,0,0);
	/**
	 * Variable for the amount of cubes in the world
	 */
	private int nbOfCubes;

	/**
	 * max and min difference in coordinates between two cubes
	 */
	private float maxX = 10;
	private float minX = -10;
	//cube can't go through the ground, so minY has to be >= 2.5m (+2m so they aren't on the ground)
	private float maxY = 14.5f;
	private float minY = 4.5f;
	private float maxZ = -40;
	private float minZ = -400;


	/**
	 * the means and standard deviations
	 */
	private float meanX = 0;
	private float stdDevX = 3;
	private float meanY = 9.5f;
	private float stdDevY = 1.5f;
	private float meanZ = -220;
	private float stdDevZ = 52;


	/**
	 * min/man/mean/std dev error applied to cube positions
	 */
	private float minError = -5.0f;
	private float maxError = 5.0f;
	private float meanError = 0f;
	private float stdDevError = 1.5f;

	/**
	 * getters for max/min X,Y,Z
	 */
	public float getMaxX() {
		return maxX;
	}

	public float getMinX() {
		return minX;
	}

	public float getMaxY() {
		return maxY;
	}

	public float getMinY() {
		return minY;
	}

	public float getMaxZ() {
		return maxZ;
	}

	public float getMinZ() {
		return minZ;
	}



	/**
	 * getters for the means and standard deviations
	 */
	public float getMeanX(){
		return meanX;
	}

	public float getMeanY(){
		return meanY;
	}

	public float getMeanZ(){
		return meanZ;
	}

	public float getStdDevX(){
		return stdDevX;
	}

	public float getStdDevY(){
		return stdDevY;
	}

	public float getStdDevZ(){
		return stdDevZ;
	}

	/**
	 * getters for the max/min/mean/std dev error applied to cube positions
	 */

	public float getMinError() { return minError; }

	public float getMaxError() {
		return maxError;
	}

	public float getMeanError() {
		return meanError;
	}

	public float getStdDevError() {
		return stdDevError;
	}

	/**
	 * array list that stores the modified positions of the cubes (error added)
	 */
	private ArrayList<Vector> erroredPositions;

	/**
	 * getter for the errored positions of the cubes
	 * @return the errored positions of the cubes
	 */
	public ArrayList<Vector> getErroredPositions() {
		return erroredPositions;
	}

	/**
	 * setter for the errored positions of the cubes
	 * @param erroredPositions the errored positions of the cubes
	 */
	public void setErroredPositions(ArrayList<Vector> erroredPositions) {
		this.erroredPositions = erroredPositions;
	}

	/**
	 * method only implemented to test functionality (only for testing purposes)
	 */
	public void testFunction(ArrayList<Vector> pos){
		ArrayList<Vector> errorPosition = generateErrorOnCubePositions(pos);

		for (int i = 0; i < pos.size(); i++){
				//System.out.println(pos.get(i));
				//System.out.println(errorPosition.get(i));
			float diffX = pos.get(i).getxValue() - errorPosition.get(i).getxValue();
			float diffY = pos.get(i).getyValue() - errorPosition.get(i).getyValue();
			float diffZ = pos.get(i).getzValue() - errorPosition.get(i).getzValue();
			if (diffX <= 5 && diffX >= -5)
				System.out.println("diffX correct");
			else
				System.out.println("incorrect");
			if (diffY <= 5 && diffY >= -5)
				System.out.println("diffY correct");
			else
				System.out.println("incorrect");
			if (diffZ <= 5 && diffZ >= -5)
				System.out.println("difZ correct");
			else
				System.out.println("incorrect");
		}
	}

}
