package internal.Testbed;
import AutopilotInterfaces.Path;
import TestbedAutopilotInterface.GUI.*;
import TestbedAutopilotInterface.Overseer.DeliveryPackage;
import TestbedAutopilotInterface.Overseer.PackageService;
import TestbedAutopilotInterface.Overseer.WorldDelivery;
import internal.Exceptions.AngleOfAttackException;
import internal.Exceptions.SimulationEndedException;
import internal.Helper.Vector;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

/**
 * Class for creating a World object.
 * @author anthonyrathe & Martijn Sauwens
 */

//TODO check if the drone is on the tarmac or not
//TODO use more resource friendly query for the packages --> now every time a shallow copy of the main set is made
//TODO only access the package service when new deliveries are available (use flags to communicate)
//TODO implement the GUIdeliveryStates into the gui frame that is sent to the GUI
//TODO soften the restrictions on the distance needed to load the package (in first iter we try only to arrive at a gate
//TODO if we're at a gate we should receive a package destined for either gate one or zero)
public class World {
	
	public World(){
		this(1);

	}

	public World(int nbOfDrones){
		this.droneThreads = Executors.newFixedThreadPool(nbOfDrones);
	}

	/**
	 * Constructor used for a shared thread pool with the testbed server (since the amount of communication threads are
	 * equal to the amount of drones. The threads are never active at the same time so we may share them)
	 * @param threadPool the thread pool assigned to the world for simulating drones
	 * @param packageService the package service used by the world to assign packages to the drone
	 *                       (only makes queries to it and sets if packages have been successfully delivered)
	 */
	public World(ExecutorService threadPool, PackageService packageService){
		this.droneThreads = threadPool;
		this.packageService = packageService;
	}

	/**
	 * Advances the world state with a given time interval
	 * if the goal has been reached, the world will stop advancing
	 * @param timeInterval the time interval
	 * @throws IllegalArgumentException thrown if the provided time interval is invalid
	 * @author Martijn Sauwens
	 * @throws IOException
	 */
	public void advanceWorldState(float timeInterval, int nbTotalIterations, int nbSubIterations) throws IllegalArgumentException, IOException, InterruptedException {
		if(!isValidTimeInterval(timeInterval))
			throw new IllegalArgumentException(INVALID_TIME_INTERVAL);

//		Set<Block> blockSet = this.getBlockSet();
//		System.out.println("Drone velocity before :" + this.getDroneSet());
		//System.out.println("nb Intervals: " + nbIntervals);

        //keep iterating until we reach zero iterations to do (no for loop to fit the nb sub iterations
        int iterationsTodo = nbTotalIterations;
		while(iterationsTodo != 0) {

            //needs to refresh the drone set on each iteration
            Set<Drone> droneSet = this.getDroneSet();

            //unload/load all the packages
            movePackages();
            //check if all the packages are delivered
            if (allPackagesDelivered()) {
                throw new SimulationEndedException();
            }

		    //check if we can still do the nb of sub iterations requested (while still staying in sync with the testbed)
		    if(iterationsTodo >= nbSubIterations) {

                //advance all the drones:
                this.advanceAllDrones(droneSet, timeInterval, nbSubIterations);

                //subtract the number of sub iterations
                iterationsTodo -= nbSubIterations;
            }else{
		        //the iterations to do are smaller than the requested sub iterations, only do the leftover iterations
                this.advanceAllDrones(droneSet, timeInterval, iterationsTodo);
                //no iterations left
                iterationsTodo = 0;
            }

            //TODO uncomment if ready to handle crashes properly
            //now check for a crash
            //checkForCrashes(droneSet);
			collisionDetection(droneSet);

        }
//		System.out.println("Drone velocity after" + this.getDroneSet());
		if(allPackagesDelivered()){
			throw new SimulationEndedException();
		}
	}

	/**
	 * Method to advance all the drones in the drone set for exactly one iteration step
	 * @param droneSet the set of drones to be advanced
	 * @param deltaTime the time for the next state step
	 * @param nbIterations the number of iterations to in one call
	 * @throws InterruptedException if the futures times out (nothing you can do about)
	 */
	private void advanceAllDrones(Set<Drone> droneSet, float deltaTime, int nbIterations) throws InterruptedException {
		//first set the time interval for all the drones
		for(Drone drone: droneSet){
		    drone.setNbIterations(nbIterations);
			drone.setDeltaTime(deltaTime);
		}
		//get the execution pool
		ExecutorService droneThreads = this.getDroneThreads();
		//System.out.println("used ThreadPool: " + this.getDroneThreads());
		//first invoke all the next states
		List<Future<Void>> unfinishedThreads = droneThreads.invokeAll(droneSet);
		List<Future<Void>> finishedThreads = new ArrayList<>();
		//then wait for all the futures to finish
		boolean allFinished = false;
		//keeps looping until all drones are advanced to the next state
		while(!allFinished){
			//first get the first thread
			Future<Void> droneFuture = unfinishedThreads.get(0);

			//wait until the first thread finishes
			try {
				droneFuture.get();
				//check if there was an exception
			} catch (ExecutionException e) {
				if(e.getCause() instanceof AngleOfAttackException){
					System.out.println("angle of attack exception");
				}else{
					System.out.println("An error occurred: " + e.getCause().toString());
				}
			}

			//get all the finished elements
			finishedThreads = unfinishedThreads.stream()
					.filter(future -> future.isDone())
					.collect(Collectors.toList());

			//check if any of the finished threads got into trouble
			for(Future<Void> finishedFuture: finishedThreads){
				try {
					finishedFuture.get();
					//check if there occurred an error
				} catch (ExecutionException e) {
					if(e.getCause() instanceof AngleOfAttackException){
						throw (AngleOfAttackException) e.getCause(); //rethrow, info for the main loop
					}else{
						e.printStackTrace();
					}
				}
			}

			//filter out all the elements that are finished
			unfinishedThreads = unfinishedThreads.stream()
					.filter(future -> !future.isDone())
					.collect(Collectors.toList());
			//check if drone futures is empty ot not
			if(unfinishedThreads.size() == 0){
				allFinished = true;
			}
		}
		//we may exit, all the drones have been set to state k+1
	}

	/**
	 * delivers and loads all the packages that can be loaded onto the drones
	 * and can be delivered to the gates
	 */
	private void movePackages(){
		//get the package set
		Set<Drone> droneSet = this.getDroneSet();
		PackageService packageService = this.getPackageService();
		//get all the packages that need to be delivered and are assigned a drone (these are the only
		//ones that need to be loaded and unloaded
		Set<WorldDelivery> packages = packageService.getAllUndeliveredAssignedWorldDeliveries();
		//first check if all the drones can unload their packages
		unloadPackages(droneSet);
		//then load all the packages
		loadPackages(droneSet, packages);

	}

	/**
	 * unloads all the packages from the drones that can deliver their packages
	 * @param drones the drones that need to be checked for possible delivery
	 */
	private void unloadPackages(Set<Drone> drones){
		//loop trough all the drones, check if they carry any packages
		for(Drone drone: drones){
			if(drone.isDelivering()){
				//check if the drone can deliver the package
				if(canDeliverPackage(drone)){
				//unload the package
				drone.unloadPackage();
				}
			}
		}

		//all the packages are unloaded
	}

	/**
	 * Loads all the packages onto the drones that can accept a new package and meet the requirements
	 * for loading a package
	 * @param drones the map of drones to check for
	 * @param packages the set of packages to check for
	 */
	private void loadPackages(Set<Drone> drones, Set<WorldDelivery> packages){
		//cycle trough all the packages, filter for all the packages that are not
		//delivered yet
		Set<WorldDelivery> unDelivered = packages.stream().filter(p -> !p.isDelivered()).collect(Collectors.toSet());
		for(Drone drone: drones){
			//first check if the drone has the right state to pick up a package
			if(!readyToPickUp(drone)){
				//if the drone is going to fast or is already delivering a package, continue no further calculation is needed
				continue;
			}
			String droneID = drone.getDroneID();
			//now get the next package to be delivered by the drone
			WorldDelivery delivery = getNextDelivery(droneID, unDelivered);
			//check if this particular package can be delivered
			if(delivery != null && canPickUpPackage(drone, delivery)){
//				System.out.println("\npicked up delivery: " + delivery + "\n");
				drone.loadPackage(delivery);
			}
		}

	}

//	for(WorldDelivery delivery: unDelivered){
//			//now check for the drone with the corresponding ID
//			String deliveryDroneID = delivery.getDeliveryDroneID();
//			//check if null, if so continue:
//			if(deliveryDroneID == null){
//				continue;
//			}
//			//if not, check if the drone can be loaded
//			Drone drone = drones.get(deliveryDroneID);
//			//check if the drone can be loaded with the package
//			if(canPickUpPackage(drone, delivery)){
//				System.out.println("delivery loading: " + delivery);
//				drone.loadPackage(delivery);
//			}
//		}

		//all packages are checked

	/**
	 * Checks if the drone is in the right state to pick up a package from the airport
	 * --> check is used as an easy (non calculation intensive) first check to see if the drone can pick up a package
	 * @param drone the drone to check
	 * @return false if the drone is already delivering a package or if the drone is going faster than the
	 * 		   package transfer velocity
	 */
	private static boolean readyToPickUp(Drone drone){
		//check first if the drone is already delivering a package
		if(drone.isDelivering()){
			return false;
		}
		//check if the velocity is low enough
		Vector droneVelocity = drone.getVelocity();
		if(droneVelocity.getSize()  > MAX_TRANSFER_VELOCITY){
			return false;
		}

		return true;
	}

	/**
	 * Gets the package with the lowest delivery sequence number, as documented in the overseer the undelivered
	 * package with the lowest sequence number must be delivered first by the drone
	 * @param droneID the id of the drone to get the next delivery for
	 * @param undeliveredDeliveries the delivery packages that are not yet delivered by the drone package system
	 * @return the next delivery to be done by the drone if available, if not, returns null
	 */
	private WorldDelivery getNextDelivery(String droneID, Set<WorldDelivery> undeliveredDeliveries){

		WorldDelivery lowestSeqNbDelivery = null;
		long lowestSeqNb = Long.MAX_VALUE;
		for(WorldDelivery delivery: undeliveredDeliveries){
			//check if this package is destined for the currently selected drone
			if(!droneID.equals(delivery.getDeliveryDroneID())){
				continue;
			}
			long currentSeqNb = delivery.getSequenceNumber();

			//and also if the sequence number is lower
			if( currentSeqNb < lowestSeqNb){
				lowestSeqNb = currentSeqNb;
				lowestSeqNbDelivery = delivery;
			}
		}

		return lowestSeqNbDelivery;
	}

	/**
	 * Method that returns a set containing all the drones in the world
	 * @author anthonyrathe
	 */
	public Set<Drone> getDroneSet(){
		return new HashSet<Drone>(this.drones.values());
	}

	/**
	 * Getter for the map of drones currently active in the world (key = drone id and value = drone with id)
	 * @return a map with drone id's as key and the corresponding drone as value
	 */
	private Map<String, Drone> getDroneMap(){
		return this.drones;
	}

	/**
	 * Getter for the map that contains all the airports in the world
	 * @return a map with as keys the id of the airports and value the corresponding airport (with the same ID)
	 */
	private Map<Integer, WorldAirport> getAirportMap(){
		return this.airports;
	}

	/**
	 * Getter for the set consisting of all airports in the world
	 * @return the set with all airports in the world
	 */
	private Set<WorldAirport> getAirportSet(){
		return new HashSet<WorldAirport>(this.getAirportMap().values());
	}
	
	/**
	 * Method that returns the drone in the world
	 * @author Anthony Rathe
	 * @throws IOException 
	 */
	public Drone getDrone() throws IOException {
		for (Drone drone : this.getDroneSet()) {
			return drone;
		}
		throw new IOException("No drone was found");
	}

	/**
	 * Add all the drones in the collection to the world
	 * @param drones the drones to be added
	 */
	public void addDrones(Collection<Drone> drones){
		Map<String, Drone> droneMap = this.getDroneMap();
		//add all the drones
		for(Drone drone: drones){
			String ID =drone.getDroneID();
			droneMap.put(ID, drone);
		}
	}

    /**
     * Adds a single drone to the set that contains all drones
     * @param drone the drone to add
     */
	public void addDrone(Drone drone){
	    Set<Drone> droneSet = this.getDroneSet();
	    droneSet.add(drone);
    }

	/**
	 * Adds an airport to the world, airports are only defined by their location and orientation
	 * the world assigns the airport an unique identifier to check packet transportation
	 * @param location the location of the airport
	 * @param heading the heading of runway zero of the airport
	 * @return the ID assigned to the airport by the world
	 * note: it is mandatory that the airport ID assigned by the world matches the ID assigned by the
	 * overseer for correct package delivery
	 */
	public int addAirport(Vector location, Vector heading){
		//get the parameters needed for airport generation
		Map<Integer, WorldAirport> airports = this.getAirportMap();
		//first check if the width and length are already defined, if not define them now
		int newID = this.getNextAirportID();
		//generate the airport
		WorldAirport airport = new WorldAirport(location, heading, this.getRunwayWidth(), this.getRunwayLength(), newID);
		airports.put(newID, airport);
		return newID;
	}

	/**
	 * Getter for the next ID to be given to a newly generated airport
	 * @return the next ID available to assign to an airport
	 * note: automatically increments the ID when called, basically acts as counter that
	 * never returns the same value (unless overflow occurs)
	 */
	private int getNextAirportID(){
		return this.airportIDCounter++;
	}

	/**
	 * Getter for the length of the runways of the airport
	 * @return the length of the runway
	 */
	private float getRunwayLength() {
		return runwayLength;
	}

	/**
	 * Setter for the length of the runways, it can be only set once, after that all different calls are ignored
	 * @param runwayLength the length of the runway to be set
	 */
	public void setRunwayLength(float runwayLength) {
		if(this.getRunwayLength() != 0f){
			return;
		}
		this.runwayLength = runwayLength;
	}

	/**
	 * Getter for the width of the runways of the airports in this world
	 * @return the width of the runways
	 */
	private float getRunwayWidth() {
		return runwayWidth;
	}

	/**
	 * Setter for the witdh of the runway, can only be set once, all calls after the initial setting will be ignored
	 * @param runwayWidth the width of the runway to be set
	 */
	public void setRunwayWidth(float runwayWidth) {
		if(this.runwayWidth != 0f){
			return;
		}
		this.runwayWidth = runwayWidth;
	}

	/**
	 * The length of the airports in the world
	 */
	private float runwayLength = 0f;

	/**
	 * The width of the airports in the world
	 */
	private float runwayWidth = 0f;

	/**
	 * Checks if the current drone can deliver its specified package
	 * @param drone the drone to check
	 * @return true if the drone has the required velocity to deliver a package and is close enough to
	 * the gate
	 */
	private boolean canDeliverPackage(Drone drone){
		//check first if drone has the required velocity, if not skip the rest of the calc
		Vector droneVelocity = drone.getVelocity();

		if(droneVelocity.getSize()  > MAX_LANDING_VELOCITY){
			return false;
		}
		//check if the drone is currently at the airport where the package needs to be delivered
		WorldDelivery delivery = drone.getDeliveryPackage();
		//first get the package
		int destinationAirportID = delivery.getDestinationAirport();
//		int destinationGateNumber = delivery.getDestinationAirportGate();
		//get the position of the gate
		Map<Integer, WorldAirport> airports = this.getAirportMap();
		WorldAirport airport = airports.get(destinationAirportID);
		return airport.isAtGateZone(drone.getPosition());
//		Vector dronePos = drone.getPosition();
//		float distanceToGate = airport.distanceToGate(destinationGateNumber, dronePos);
//		return distanceToGate <= MAX_TRANSFER_DISTANCE;
	}

	/**
	 * Checks if the drone can pick up the specified package
	 * @param drone the drone tested to pick up the package
	 * @param delivery the package to deliver and to be tested against the drone
	 * @return true if the package delivery drone ID and the drone ID match and the drone has met
	 * the required position and landing velocity to deliver the package
	 */
	private boolean canPickUpPackage(Drone drone, WorldDelivery delivery){
		//check if the drone ID and the delivery drone ID match
		String droneID = drone.getDroneID();
		String deliveryDroneID = delivery.getDeliveryDroneID();
		//if not return false
		if(!droneID.equals(deliveryDroneID)){
			return false;
		}
		//get the package source airport and position
		int sourceAirportID = delivery.getSourceAirport();
//		int sourceGateNumber = delivery.getSourceAirportGate();
		//get the position of the gate
		Map<Integer, WorldAirport> airports = this.getAirportMap();
		WorldAirport airport = airports.get(sourceAirportID);
//		Vector dronePos = drone.getPosition();
//		float distanceToGate = airport.distanceToGate(sourceGateNumber, dronePos);
//
//		return distanceToGate <= MAX_TRANSFER_DISTANCE;
		return airport.isAtGateZone(drone.getPosition());
	}


	//the maximum velocity the drone may have before it can transfer a package
	private final static float MAX_TRANSFER_VELOCITY = 1.0f;

	//the maximum distance between the target gate and the drone before a package can be transferred
	private final static float MAX_TRANSFER_DISTANCE = 25.0f;

	/**
	 * Checks if there are any drones that have crashed during the time step simulated
	 * @param droneSet the set of drones to be checked
	 */
	private void checkForCrashes(Set<Drone> droneSet){
		//first check if the drones have crashed with the ground
		checkForGroundCollisions(droneSet);
		//then if they have collided with each other
		checkForDroneCollisions(droneSet);

	}

	/**
	 * Checks if the drones have crashed with the ground if so it removes the drones from the world object set
	 * and removes it from the provided drone set (need for integrity)
	 * @param droneSet the set of drones to check
	 * for definition of crash see the check crash implementation
	 */
	private void checkForGroundCollisions(Set<Drone> droneSet) {
		for(Drone drone: droneSet){
			//if so, delete the drone from the set
			if(drone.checkCrash()){
				this.removeDrone(drone);
				droneSet.remove(drone);
			}
		}
	}

	/**
	 * Checks the drone set for colliding drones a drone collides if the centers of two drones are within
	 * the specified crash distance of 5 meters (see static variable CRASH_DISTANCE)
	 * @param droneSet the set of drones to check for crashes
	 * @return the set of drones that have collided
	 */
	private Set<Drone> collisionDetection(Set<Drone> droneSet){
		//convert the set of drones to an array list
		List<Drone> droneList = new ArrayList<>(droneSet);
		Set<Drone> crashedDrones = new HashSet<>();

		int nbDrones = droneList.size();
		//cycle trough all the drones and check if they have crashed
		for(int i = 0; i != nbDrones; i++){
			//get the current drone
			Drone drone1 = droneList.get(i);
			//check if the drone hasn't already crashed
			if(crashedDrones.contains(drone1)){
				//if so, we do not need to check it
				continue;
			}
			Vector posDrone1 = drone1.getPosition();
			for(int j = i+1; j < nbDrones; j++){
				Drone drone2 = droneList.get(j);
				Vector posDrone2 = drone2.getPosition();
				float distance = posDrone2.distanceBetween(posDrone1);
				if(distance <= CRASH_DISTANCE){
					crashedDrones.add(drone1);
					crashedDrones.add(drone2);
					//because the upper drone has crashed so we do not need to check
					break;
				}
			}
		}

		return crashedDrones;
	}

	/**
	 * Checks for drones that have collided with each other and removes the collided drones from the world object set
	 * as well from the provided drone set
	 * @param droneSet the set of drones that needs to be checked for collisions
	 */
	private void checkForDroneCollisions(Set<Drone> droneSet) {
		//then check if any drone is in a 5m vicinity of another
		//first cast the set to a list
		List<Drone> droneList = new ArrayList<>(droneSet);
		//create a list to store the crashed indices
		Set<Integer> crashedDroneIndices = new HashSet<>();
		//get the size of the list
		int listSize = droneList.size();

		//Outer loop, check every drone once
		for(int i = 0; i != listSize; i++){
			//inner loop, only the following drones need to be checked, all the previous have already passed the outer loop
			for(int j = i + 1; j  < listSize; j++){
				//first get the positions of the drone
				Vector pos1 = droneList.get(i).getPosition();
				Vector pos2 = droneList.get(j).getPosition();

				//then get the distance between the two drones
				float distance = pos1.distanceBetween(pos2);
				if(distance <= CRASH_DISTANCE){
					crashedDroneIndices.add(i);
					crashedDroneIndices.add(j);
				}
			}
		}

		//all the drones that have collided with eachother need to be removed
		for(Integer droneIndex: crashedDroneIndices){
			//first get the drone
			Drone currDrone = droneList.get(droneIndex);
			//then remove it from the world
			this.removeDrone(currDrone);
			//remove it from the drone set (for consistency and performance improvement)
			droneSet.remove(currDrone);
		}
	}

	/**
	 * Remove a given drone from the world object set
	 * @param drone the drone to be removed from the world
	 */
	private void removeDrone(Drone drone){
		this.getDroneSet().remove(drone);
	}


	/**
	 * Checks if all the packages are delivered
	 * @return true if and only if all the packages have the delivered flag
	 */
	private boolean allPackagesDelivered(){
		PackageService service = this.getPackageService();
		Set<WorldDelivery> packages = service.getAllUndeliveredWorldDeliveries();
		for(WorldDelivery delivery : packages){
			if(!delivery.isDelivered()){
				return false;
			}
		}

		//TODO uncomment if packages are implemented
		return false;
	}


	/**
	 * Getter for the approximated cube path, each cube lies within a probability sphere of
	 * 5 meters from the indicated location
	 * @return the path approximating the cube positions
	 */
	public Path getApproxPath() {
		return approxPath;
	}

	/**
	 * Setter for the approximated cube path (see getter for more info)
	 * @param approxPath the approximated path
	 */
	protected void setApproxPath(Path approxPath) {
		this.approxPath = approxPath;
	}

	/**
	 * Getter for all the threads used to simulate a world
	 * @return an ExecutorService containing the threads
	 */
	private ExecutorService getDroneThreads() {
		return droneThreads;
	}

	/**
	 * Checks if the provided time interval is valid
	 * @param timeInterval the time interval to be checked
	 * @return true if and only if the time interval is valid
	 * @author Martijn Sauwens
	 */
	private boolean isValidTimeInterval(float timeInterval){
		return timeInterval > 0.0f;
	}

//	/**
//	 * Getter for the list of packages that need to be delivered
//	 * @return the list of all packages that need to be delivered in this world
//	 */
//	private Set<DeliveryPackage> getPackages() {
//		return packages;
//	}

//	/**
//	 * Adds a single package to the world that has to be delivered
//	 * only adds the package if it is a non null reference, otherwise this method does not have any effect
//	 * @param deliveryPackage the package to deliver
//	 */
//	public void addPackage(DeliveryPackage deliveryPackage){
//		//get the current package set
//		Set<DeliveryPackage> packages = this.getPackages();
//		//and add the package
//		if(isValidPackage(deliveryPackage)) {
//			packages.add(deliveryPackage);
//		}
//	}

//	/**
//	 * Adds all the supplied packages to the world, if the packages are not valid they will not be added
//	 * to the world (no notification is given, these invalid packages are simply ignored)
//	 * @param newPackages the packages to add
//	 */
//	public void addPackages(Collection<DeliveryPackage> newPackages){
//		//get the current package set
//		Set<DeliveryPackage> packageSet = this.getPackages();
//		//filter for all the valid packages in the new packages
//		Set<DeliveryPackage> validPackages = newPackages.stream().filter(p -> isValidPackage(p)).collect(Collectors.toSet());
//		//add all the packages that are valid
//		packageSet.addAll(validPackages);
//	}

	/**
	 * Checks if the package can be added to the world
	 * @param deliveryPackage the package to add
	 * @return true if the package is a non null reference
	 */
	private static boolean isValidPackage(DeliveryPackage deliveryPackage){
		return deliveryPackage != null;
	}

	/**
	 * Getter for the package service used in the world
	 * the service is responsible for assigning packages to the drones and notifying the world
	 * about which packages should be given to which drone
	 * @return a package service used by the world
	 */
	private PackageService getPackageService() {
		return packageService;
	}

	/**
	 * The approximated path of the cubes
	 */
	private AutopilotInterfaces.Path approxPath;

	/**
	 * An execution pool created for faster simulation of the drones
	 */
	private ExecutorService droneThreads;


	/**
	 * The map that contains all the airports currently present in the world
	 * the keys are the ID's of the airports and the values the airports itself
	 */
	private Map<Integer, WorldAirport> airports = new HashMap<>();


	/**
	 * The map of drones currently active in the world (key=droneID, value=drone with ID)
	 */
	private Map<String, Drone> drones = new HashMap<>();


	/**
	 * The package service used to deliver the packages, is used to check which packages need to be passed to which drone
	 */
	private PackageService packageService;

	/**
	 * A counter that keeps track of the ID's that were already distributed to the drones
	 */
	private int airportIDCounter = 0;


	
	// Error strings
	private final static String ADD_WORLD_OBJECT_ERROR = "The object couldn't be added to the world";
	private final static String WORLD_OBJECT_404 = "The object couldn't be found";
	private final static String INVALID_TIME_INTERVAL = "The time interval is <= 0, please provide a strictly positive number";
	private final static String INVALID_OBJECTIVE = "The specified objective is not possible";



	/**
	 * Constants
	 */
	public final static float CRASH_DISTANCE = 5.0f;
	public final static float MAX_LANDING_VELOCITY = 1.0f;


	/**
	 * Creates a new gui queue element, these elements are used to be put in the rendering queue for the
	 * GUI. This allows us to decouple the testbed from the renderer (we render on a different thread)
	 * The renderer reads the queue elements one by one (every 1/20 s) and renders the produced frame
	 * @return a GuiQueue element to insert into the renderer queue
	 */
	public GUIQueueElement getGuiElements(){
		//retrieve the needed sets
		Set<Drone> drones = this.getDroneSet();
//		Set<Block> blocks = this.getBlockSet(); //no blocks in the world
		Set<WorldAirport> airports = this.getAirportSet();
		Set<WorldDelivery> deliveries = this.getPackageService().getSubmittedWorldDeliveries();

		//generate the necessary data
		Map<String, DroneGuiState> droneGuiQueueElem = getDroneGuiStates(drones);
		Set<CubeGuiState> cubeGuiQueueElem = new HashSet<>(); //there are no blocks anymore
		Set<AirportGuiState> airportGuiQueueElem = getAirportGuiStates(airports);
		Set<DeliveryGuiState> deliveryGuiQueueElem = getDeliveryGuiStates(deliveries);

		//create a new entry for the queue
		return new GUIQueueElement() {
			@Override
			public Map<String, DroneGuiState> getDroneStates() {
				return droneGuiQueueElem;
			}

			@Override
			public Set<CubeGuiState> getCubePositions() {
				return cubeGuiQueueElem;
			}

			@Override
			public Set<AirportGuiState> getAirports() {
				return airportGuiQueueElem;
			}

			@Override
			public Set<DeliveryGuiState> getDeliveries(){
				return deliveryGuiQueueElem;
			}
		};

	}

	/**
	 * Creates a map of drone gui states with as the key the drone ID and value the
	 * state of the drone needed to render for the GUI
	 * @param drones the drones extract the necessary data from
	 * @return a Map with as keys the drone ID's and the corresponding state of the drones
	 */
	private Map<String, DroneGuiState> getDroneGuiStates(Set<Drone> drones)
	{
		//generate the map containing the gui states
		Map<String, DroneGuiState> stateMap = new HashMap<>();
		//fill the map
		String key;
		DroneGuiState value;
		for(Drone drone: drones){
			//get the ID, position and orientation of the drone
			key = drone.getDroneID();
			Vector position = drone.getPosition().deepCopy();
			Vector velocity = drone.getVelocity().deepCopy();
			Vector orientation = drone.getOrientation().deepCopy();
			value = new DroneGuiState() {
				@Override
				public Vector getPosition() {
					return position;
				}

				@Override
				public Vector getVelocity() {
					return velocity;
				}

				@Override
				public Vector getOrientation() {
					return orientation;
				}
			};

			//add the key value pair
			stateMap.put(key, value);
		}

		//return the newly created hashMap
		return stateMap;
	}


	/**
	 * Generates a set of airport states for the GUI, used for rendering the airports
	 * present within the world
	 * @param airports the airports to convert
	 * @return a set of AirportGuiStates to be put into the GuiQueue
	 */
	private Set<AirportGuiState> getAirportGuiStates(Set<WorldAirport> airports){
		//create the set to store all the states
		Set<AirportGuiState> stateSet = new HashSet<>();
		for(WorldAirport airport: airports){
			//create the entry
			AirportGuiState airportState = airport.getGuiState();

			//add the entry
			stateSet.add(airportState);
		}

		return stateSet;
	}

	/**
	 * Creates a set of delivery state objects containing all the states for the deliveries, will be sent over to the
	 * GUI for rendering the data about the packages.
	 * @param deliveries the deliveries to create the state set from
	 * @return a set of deliveryGuiStates to describe the state of all packages in the world
	 */
	private Set<DeliveryGuiState> getDeliveryGuiStates(Set<WorldDelivery> deliveries){
		Set<DeliveryGuiState> guiDeliveries = new HashSet<>();
		for(WorldDelivery delivery : deliveries){
			//grab the needed parameters
			int sourceAirport = delivery.getSourceAirport();
			int sourceGate = delivery.getSourceAirportGate();
			int destinationAirport = delivery.getDestinationAirport();
			int destinationGate = delivery.getDestinationAirportGate();
			String deliveryDrone = delivery.getDeliveryDroneID();
			boolean isPickedUp = delivery.isPickedUp();
			boolean isDelivered = delivery.isDelivered();
			DeliveryGuiState deliveryState = new DeliveryGuiState() {
				@Override
				public int getSourceAirport() {
					return sourceAirport;
				}

				@Override
				public int getSourceGate() {
					return sourceGate;
				}

				@Override
				public int getDestinationAirport() {
					return destinationAirport;
				}

				@Override
				public int getDestinationGate() {
					return destinationGate;
				}

				@Override
				public String getDeliveryDrone() {
					return deliveryDrone;
				}

				@Override
				public boolean isPickedUp() {
					return isPickedUp;
				}

				@Override
				public boolean isDelivered() {
					return isDelivered;
				}
			};

			guiDeliveries.add(deliveryState);
		}

		return guiDeliveries;
	}

	@Override
	public String toString() {
		return "World{" +
				", airports=" + airports +
				", drones=" + drones +
				'}';
	}
}


	/*
	Code graveyard
	 */

//	/**
//	 * Method that returns a set containing all the blocks in the world
//	 * @author anthonyrathe
//	 */
//	public Set<Block> getBlockSet(){
//		return this.getSet(Block.class);
//	}
//	/**
//	 * Method that returns a set of all the objects in the world
//	 * @author anthonyrathe
//	 */
//	public Set<WorldObject> getObjectSet(){
//		return this.objects;
//	}
//
//	/**
//	 * Method that adds a given worldobject to the world
//	 * @param object the object to be added
//	 * @author anthonyrathe
//	 */
//	public void addWorldObject(WorldObject object) throws IllegalArgumentException{
//		if (this.canHaveAsObject(object)){
//			this.objects.add(object);
//		}else{
//			throw new IllegalArgumentException(ADD_WORLD_OBJECT_ERROR);
//		}
//	}
//	/**
//	 * Generates a set of cube states for the GUI
//	 * @param blocks the blocks to convert
//	 * @return a set of gui cube states
//	 */
//	private Set<CubeGuiState> getCubeGuiSates(Set<Block> blocks){
//		//create the set to store all the states
//		Set<CubeGuiState> stateSet = new HashSet<>();
//		for(Block block: blocks){
//			//create the entry
//			CubeGuiState cubeState = new CubeGuiState() {
//				@Override
//				public Vector getCurrentPosition() {
//					return block.getCurrentPosition();
//				}
//			};
//
//			//add the entry
//			stateSet.add(cubeState);
//		}
//
//		return stateSet;
//	}


/*
Code graveyard
 */

//
//	/**
//	 * Method that checks if an object can be added to this world
//	 * @param object object to perform the check on
//	 * @return true if the world can accept the object
//	 * @author anthonyrathe
//	 */
//	public boolean canHaveAsObject(WorldObject object){
//		return WorldObject.canHaveAsWorld(this);
//	}
//
//	/**
//	 * Method that removes a given worldobject from the world
//	 * @param object the object to be added
//	 * @author anthonyrathe
//	 */
//	public void removeWorldObject(WorldObject object) throws IllegalArgumentException{
//		if (this.hasWorldObject(object)){
//			this.objects.remove(object);
//		}else{
//			throw new IllegalArgumentException(WORLD_OBJECT_404);
//		}
//	}
//	/**
//	 * Objectives
//	 */
//	public final static String REACH_CUBE_OBJECTIVE = "reach cube";
//	public final static String VISIT_ALL_OBJECTIVE = "visit all the cubes";
//	public final static String NO_OBJECTIVE = "no objective";
//	public final static String FLIGHT_OBJECTIVE = "do a complete flight";
//	/**
//	 * Calculates the height of the drone when landed
//	 * @param drone the drone to calculate for
//	 * @return the height of the tyre  + the height of the chassis
//	 */
//	private float landedDroneYPos(Drone drone){
//		AutopilotConfig config = drone.getAutopilotConfig();
//		float tyreRadius = config.getTyreRadius();
//		float chassisHeight = config.getWheelY();
//		return tyreRadius + chassisHeight;
//	}

//	/**
//	 * Variable containing the current objective (atm only vist all cubes and reach cube)
//	 */
//	private String objective;
//
//		boolean withinFourMeters = block.getCurrentPosition().distanceBetween(drone.getCurrentPosition()) <=4.0f;
//		switch (this.getObjective()){
//			case REACH_CUBE_OBJECTIVE:
//				return withinFourMeters;
//			case VISIT_ALL_OBJECTIVE:
//				//first check if the current cube has been reached, if not we cannot have visited them all
//				if(!withinFourMeters)
//					return false;
//
//				block.setVisited();
//
//				for(Block block1: this.getBlockSet()){
//					if(block1.isVisited()) {
//						this.removeWorldObject(block1);
//						this.getBlockSet().remove(block);
//						//System.out.print("Visited block: " + block1);
//					}
//				}
//
//				//check if all the cubes are visited
//				for(Block currentBlock: this.getBlockSet()){
//					// if not return false
//					if(!currentBlock.isVisited())
//						return false;
//				}
//				//only if all the cubes are visited the for loop will terminate
//				return true;
//			case NO_OBJECTIVE:
//				return false; // no objective was set
//			case FLIGHT_OBJECTIVE:
//				return flightObjectiveReached(block, drone);
//		}
//		return false;
//	}

//	private boolean flightObjectiveReached(Block block, Drone drone) {
//		boolean withinFourMeters = block.getCurrentPosition().distanceBetween(drone.getCurrentPosition()) <=4.0f;
//		if(!withinFourMeters)
//            return false;
//
//		block.setVisited();
//
//		for(Block block1: this.getBlockSet()){
//            if(block1.isVisited()) {
//                this.removeWorldObject(block1);
//                this.getBlockSet().remove(block);
//                //System.out.print("Visited block: " + block1);
//            }
//        }
//
//		//check if all the cubes are visited
//		for(Block currentBlock: this.getBlockSet()){
//            // if not return false
//            if(!currentBlock.isVisited())
//                return false;
//        }
//
//        return droneVisitedAllCubesAndOnGround(drone);
//	}

//	/**
//	 * Checks if the drone is on the ground, has reached all the cubes and came to a standstill
//	 * @param drone the drone to check
//	 * @return true if the drone is on the ground, had visited all the cubes and has velocity < 1m/s
//	 */
//	private boolean droneVisitedAllCubesAndOnGround(Drone drone){
//		//first check if all the blocks are visited
//		boolean ready = this.getBlockSet().size() == 0;
//		//check if more or les on the ground
//		ready = ready && (drone.getCurrentPosition().getyValue() <= landedDroneYPos(drone));
//		//check if moving fast
//		ready = ready && drone.getPreviousPosition().getSize() < MAX_LANDING_VELOCITY;
//		return ready;
//
//	}
//	/**
//	 * Getter for the currenr objective
//	 * @return a string with the current objective
//	 * @author Martijn Sauwens
//	 */
//	private String getObjective() {
//		return objective;
//	}
//
//	/**
//	 * Setter for the current objective
//	 * @param objective the current objective
//	 * @author Martijn Sauwens
//	 */
//	private void setObjective(String objective) {
//		if(!canHaveAsObjective(objective))
//			throw new IllegalArgumentException(INVALID_OBJECTIVE);
//		this.objective = objective;
//	}
//	/**
//	 * Checker if the provided objective is valid
//	 * @param objective the objective to check
//	 * @author Martijn Sauwens
//	 */
//	public static boolean canHaveAsObjective(String objective){
//		switch(objective){
//			case REACH_CUBE_OBJECTIVE:
//				return true;
//			case VISIT_ALL_OBJECTIVE:
//				return true;
//			case NO_OBJECTIVE:
//				return true;
//			case FLIGHT_OBJECTIVE:
//				return true;
//			default:
//				return false;
//		}
//	}
//
//	/**
//	 * @return an array containing all the possible objectives
//	 * @author Martijn Sauwens
//	 */
//	public String[] getAllPossibleObjectives(){
//		return new String[]{VISIT_ALL_OBJECTIVE, REACH_CUBE_OBJECTIVE};
//	}
//	private List<Drone> getDisplayDronePathsList(){
//		return this.displayDronePathsList;
//	}
//
//	private boolean displayDronePathsListContains(Drone drone) {
//		return getDisplayDronePathsList().contains(drone);
//	}
//
//	private void addToDisplayDronePathsList(Drone drone) {
//		this.displayDronePathsList.add(drone);
//	}
//
//	private void removeFromoDisplayDronePathsList(Drone drone) {
//		this.displayDronePathsList.remove(drone);
//	}
//	/**
//	 * Adds all the world objects in the list to the world
//	 * @param worldObjects the world objects to be added
//	 */
//	public void addWorldObjects(Collection<WorldObject> worldObjects){
//		for(WorldObject object: worldObjects){
//			this.addWorldObject(object);
//		}
//	}
//	/**
//	 * Method that removes all blocks from the world
//	 * @author Anthony Rathe
//	 */
//	public void removeBlocks() {
//		for (Block block : this.getBlockSet()) {
//			removeWorldObject(block);
//		}
//	}
//
//	/**
//	 * Method that returns a block from the world
//	 * @return
//	 */
//	public Block getRandomBlock() {
//		for (Block block : this.getBlockSet()) {
//			return block;
//		}
//		return null;
//	}
//
//	/**
//	 * Method that checks whether the world contains a given object or not
//	 * @param object the object to be found in the world
//	 * @author anthonyrathe
//	 */
//	public boolean hasWorldObject(WorldObject object){
//		return this.getObjectSet().contains(object);
//	}
//
//	/**
//	 * Method that returns a set of all the objects in the world that belong to a given subclass
//	 * @param type the class to which the requested objects should belong
//	 * @author anthonyrathe
//	 */
//	public <type> Set<type> getSet(Class<? extends WorldObject> type){
//		Set<type> objects = new HashSet<type>();
//		for (WorldObject object : this.getObjectSet()) {
//			if (type.isInstance(object)){
//				objects.add((type)object);
//			}
//		}
//		return objects;
//	}
//public int getXsize(){
//	return Xsize;
//}
//	public int getYsize() {
//		return Ysize;
//	}
//	public int getZsize(){
//		return Zsize;
//	}