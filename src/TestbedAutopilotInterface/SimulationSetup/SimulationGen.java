package TestbedAutopilotInterface.SimulationSetup;

import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import TestbedAutopilotInterface.Overseer.PackageService;
import TestbedAutopilotInterface.TestbedServer;
import gui.WorldObjects.Airport;
import internal.Helper.Vector;

import java.util.*;

import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

/**
 * Created by Martijn on 7/04/2018.
 * A class to generate an entire simulation environment
 * --> the airports
 * --> the drones
 * --> the packages
 * Used to manage the complexity of creating a world
 *
 * total surface where the airports can be located is specified as (2*WorldXSize)*(2*WorldZSize)
 * boundaries of the world are [-xSize, xSize] for x and [-zSize, zSize] for z
 * the created world is centered in (0,0,0)
 */
public class SimulationGen {



    /**
     * Default constructor for a simulation generator
     * @param worldXSize the x-boundary for the world size
     * @param worldZSize the z-boundary for the world size
     * @param nbDrones   the number of drones to be added to the simulation environment
     * @param nbAirports the number of airports to be added to the simulation environment
     * @param nbPackages the number of packages to be added to the simulation environment
     *
     */
    public SimulationGen(float worldXSize, float worldZSize, int nbDrones, int nbAirports, int nbPackages){
        this.worldXSize = worldXSize;
        this.worldZSize = worldZSize;
        this.nbDrones = nbDrones;
        this.nbAirports = nbAirports;
        this.nbPackages = nbPackages;
    }

    /**
     * Constructor for when a standard world format is generated
     * @param worldXSize the size of the world in the x-direction
     * @param worldZSize the size of the world in the z-direction
     */
    public SimulationGen(float worldXSize, float worldZSize){
        this(worldXSize, worldZSize, 0,0,0);
    }

    /**
     * warning, overwrites the previous configuration
     * @return a big world for the demo
     */
    public SimulationEnvironment generateBigWorld(){
        //first overwrite the provided parameters
        this.worldXSize = 4E4f;
        this.worldZSize = 4E4f;
        this.nbDrones = 25;
        this.nbAirports = 25;
        this.nbPackages = 40;

        int nbRows = 5;
        int nbColumns = 5;
        return  generateGridWorld(nbRows, nbColumns);
    }

    /**
     * Generates a world where the airports are located in a grid like fashion with only a random orientation of
     * the heading vector of the airports
     * @return a similation environment object containing the specifications for the simulation environment
     */
    private SimulationEnvironment generateGridWorld(int nbRows, int nbColumns){
        if(!canGenerateGrid(nbRows, nbColumns)){
            throw new IllegalArgumentException(GRID_SIZE_ERROR);

        }
        List<AirportSpec> airportSpecs = generateAirportsGrid(nbRows, nbColumns);
        //check if the generated grid is valid
        if(airportsCanOverlap(airportSpecs)) {
            throw new IllegalStateException(AIRPORT_FIT_EXCEPTION);
        }
        //now generate the drones
        List<DroneSpec> droneSpecs = generateDrones(airportSpecs);

        //generate the packages
        Set<DeliverySpec> deliverySpecs = new HashSet<>(generateDeliveries(airportSpecs));

        //generate the simulation environment
        SimulationEnvironment simulationEnvironment = new SimulationEnvironment() {
            @Override
            public List<DroneSpec> getDroneSpecifications() {
                return droneSpecs;
            }

            @Override
            public List<AirportSpec> getAirportSpecifications() {
                return airportSpecs;
            }

            @Override
            public Set<DeliverySpec> getDeliveryPackages() {
                return deliverySpecs;
            }
        };

        return simulationEnvironment;
    }

    public SimulationEnvironment generate2AirportWorld(){
        float xPosAirport1 = this.getWorldXSize();
        float xPosAirport2 = - xPosAirport1;
        float zPosAirport1 = -this.getWorldZSize();
        float zPosAirport2 = -zPosAirport1;

        Vector airport1Pos = new Vector(xPosAirport1, 0, zPosAirport1);
        Vector airport2Pos = new Vector(xPosAirport2, 0, zPosAirport2);

        //the first airport in the upper right corner faces to the left with the main runway
        float airportRunwayLength = this.getAirportRunwayLength();
        float airportRunwayWidth = this.getAirportRunwayWidth();
        Vector heading1 = new Vector(-airportRunwayLength, 0,0);
        //the second airport in the lower left corner faces upward
        Vector heading2 = new Vector(0,0, -airportRunwayLength);

        //generate the airport specs
        List<AirportSpec> airports = new ArrayList<>();
        AirportSpec airport1 = generateAirportAt(airport1Pos, heading1);

        AirportSpec airport2 = generateAirportAt(airport2Pos, heading2);
        airports.add(airport1);
        airports.add(airport2);

        //place the package & the drone: dest airport 2 & drone airport = 1
        Vector droneOrientation = new Vector(getHeadingAngle(heading1),0,0);
        Vector dronePos = airport1Pos.vectorSum(getDroneYPos());
        DroneSpec drone = new DroneSpec() {
            @Override
            public Vector getDronePosition() {
                return dronePos;
            }

            @Override
            public Vector getDroneOrientation(){
                return droneOrientation;
            }
        };

        DeliverySpec delivery = generateDelivery(1,0,0,0);

        //generate the lists
        List<DroneSpec> drones = new ArrayList<>();
        drones.add(drone);
        Set<DeliverySpec> deliveries = new HashSet<>();
        deliveries.add(delivery);

        return new SimulationEnvironment() {
            @Override
            public List<DroneSpec> getDroneSpecifications() {
                return drones;
            }

            @Override
            public List<AirportSpec> getAirportSpecifications() {
                return airports;
            }

            @Override
            public Set<DeliverySpec> getDeliveryPackages() {
                return deliveries;
            }
        };
    }

    public SimulationEnvironment generate4AirportWorld(){
        float worldXSize = this.getWorldXSize();
        float worldZSize = this.getWorldZSize();

        //airport 1: located at upper left
        //airport 2: located at upper right
        //airport 3: located at lower left
        //airport 4: located at lower right
        Vector airport1Pos = new Vector(-worldXSize, 0, -worldZSize);
        Vector airport2Pos = new Vector(worldXSize, 0, -worldZSize);
        Vector airport3Pos = new Vector(-worldXSize, 0, worldZSize);
        Vector airprot4Pos = new Vector(worldXSize, 0, worldZSize);

        //the first airport in the upper right corner faces to the left with the main runway
        float airportRunwayLength = this.getAirportRunwayLength();
        float airportRunwayWidth = this.getAirportRunwayWidth();
        //generate all runways heading to the center of the world
        Vector heading1 = new Vector(1, 0,1);
        Vector heading2 = new Vector(-1,0, 1);
        Vector heading3 = new Vector(1,0,-1);
        Vector heading4 = new Vector(-1, 0,-1);


        //generate the airport specs
        List<AirportSpec> airports = new ArrayList<>();
        AirportSpec airport1 = generateAirportAt(airport1Pos, heading1);
        AirportSpec airport2 = generateAirportAt(airport2Pos, heading2);
        AirportSpec airport3 = generateAirportAt(airport3Pos, heading3);
        AirportSpec airport4 = generateAirportAt(airprot4Pos, heading4);
        airports.add(airport1);
        airports.add(airport2);
        airports.add(airport3);
        airports.add(airport4);

        //place the package & the drone: destination airport 2 & drone airport = 1
        Vector drone1Orientation = new Vector(getHeadingAngle(heading1),0,0);
        Vector drone1Pos = airport1Pos.vectorSum(getDroneYPos());
        Vector drone2Orientation = new Vector(getHeadingAngle(heading3), 0,0);
        Vector drone2Pos = airport3Pos.vectorSum(getDroneYPos());


        DroneSpec drone1 = generateDrone(drone1Pos, drone1Orientation);
        DroneSpec drone2 = generateDrone(drone2Pos, drone2Orientation);

        //make a delivery for every airport starting at the first airport
        DeliverySpec delivery1 = generateDelivery(0,0,1,0);
        DeliverySpec delivery2 = generateDelivery(0,0,2,0);
        DeliverySpec delivery3 = generateDelivery(0,0,3,0);
        DeliverySpec delivery4 = generateDelivery(3, 0,0,1);


        //generate the lists
        List<DroneSpec> drones = new ArrayList<>();
        drones.add(drone1);
        drones.add(drone2);
        Set<DeliverySpec> deliveries = new HashSet<>();
        deliveries.add(delivery1);
        deliveries.add(delivery2);
        deliveries.add(delivery3);
        deliveries.add(delivery4);

        return new SimulationEnvironment() {
            @Override
            public List<DroneSpec> getDroneSpecifications() {
                return drones;
            }

            @Override
            public List<AirportSpec> getAirportSpecifications() {
                return airports;
            }

            @Override
            public Set<DeliverySpec> getDeliveryPackages() {
                return deliveries;
            }
        };
    }

    /**
     * Generates an airport at the specified location with the given heading for runway zero
     * @param airportPos the position of the airport
     * @param airportHeading the orientation of runway zero
     * @return an airport at the given location with the given heading
     */
    private AirportSpec generateAirportAt(Vector airportPos, Vector airportHeading){
        float runwayWidth = this.getAirportRunwayWidth();
        float runwayLength = this.getAirportRunwayLength();
        AirportSpec airport = new AirportSpec() {
            @Override
            public Vector getPosition() {
                return airportPos;
            }

            @Override
            public Vector getPrimaryRunWay() {
                return airportHeading;
            }

            @Override
            public float getRunwayWidth() {
                return runwayWidth;
            }

            @Override
            public float getRunwayLength() {
                return runwayLength;
            }
        };

        return airport;
    }

    /**
     * Generates the delivery specification with the desired sources and destinations
     * @param sourceAirport the id of the airport where to pick up the package
     * @param sourceGate the id of the gate where the package must be picked up
     * @param destinationAirport the id of the airport where the package is destined to
     * @param destinationGate the id of the gate where the package must be dropped of
     * @return a delivery spec matching the specifications of the parameters defined above
     */
    private DeliverySpec generateDelivery(int sourceAirport, int sourceGate, int destinationAirport, int destinationGate){
        DeliverySpec delivery = new DeliverySpec() {
            @Override
            public int getSourceAirport() {
                return sourceAirport;
            }

            @Override
            public int getSourceAirportGate() {
                return sourceGate;
            }

            @Override
            public int getDestinationAirport() {
                return destinationAirport;
            }

            @Override
            public int getDestinationAirportGate() {
                return destinationGate;
            }
        };

        return delivery;
    }

    /**
     * Generates the drone spec with the given specifications
     * @param dronePos the position of the drone in the world axis system
     * @param droneOrientation the orientation of the drone (in heading pitch and roll)
     * @return the droneSpec with the given specifications
     */
    private DroneSpec generateDrone(Vector dronePos, Vector droneOrientation){
        DroneSpec droneSpec = new DroneSpec() {
            @Override
            public Vector getDronePosition() {
                return dronePos;
            }

            @Override
            public Vector getDroneOrientation() {
                return droneOrientation;
            }
        };

        return droneSpec;
    }
    /**
     * Generates the deliveries for the simulation environment based on the airports that are present in the world
     * The deliveries are distributed randomly across all the airports in the world
     * @param airports the airports that are present in the world
     * @return a list of deliveries
     */
    private List<DeliverySpec> generateDeliveries(List<AirportSpec> airports){
        //get the number of packages to deliver
        int nbPackages = this.getNbPackages();

        //generate the list to store the specifications
        List<DeliverySpec> deliverySpecs = new ArrayList<>();

        //iterate trough the packages, assign randomly to an airport, if an airport already has
        //its maximum nb of packages, delete it from the subset
        //create new random number generator
        Random random = new Random(getRandomSeed());

        for(int assignedPackages = 0; assignedPackages != nbPackages; assignedPackages++){
            DeliverySpec newDelivery = generateDelivery(airports, random);
            deliverySpecs.add(newDelivery);
        }

        //if we exit all the packages should have been assigned

        return deliverySpecs;
    }

    /**
     * Generates a single delivery, the delivery contains a random generated destination and source airport
     * with a random source and destination gate
     * @param airports the airports to add the packages to
     * @param random the random number generator used to generate the lay of the map
     */
    private DeliverySpec generateDelivery(List<AirportSpec> airports, Random random) {
        int nbAirports = airports.size();
        //get source and destination airport
        int sourceIndex = random.nextInt(nbAirports);
        int destIndex = random.nextInt(nbAirports);
        //the destination index must be different from the source index
        while(sourceIndex == destIndex){
            destIndex = random.nextInt(nbAirports);
        }

        //get source and destination gates
        int sourceGateID = random.nextInt(maxGateID);
        int destGateID = random.nextInt(maxGateID);
        //get an airport at random
        //then set the airport specs to the package
        int sourceAirportID = sourceIndex;
        int destAirportID = destIndex;
        //generate the specs for the package
        DeliverySpec delivery = new DeliverySpec() {
            @Override
            public int getSourceAirport() {
                return sourceAirportID;
            }

            @Override
            public int getSourceAirportGate() {
                return sourceGateID;
            }

            @Override
            public int getDestinationAirport() {
                return destAirportID;
            }

            @Override
            public int getDestinationAirportGate() {
                return destGateID;
            }
        };

        return delivery;
    }

    /**
     * Randomly generates drones located at the airports specified in the provided list
     * @param airports the airport to generate the drones at
     * @return a list containing the specifications of the drones that were generated
     */
    private List<DroneSpec> generateDrones(List<AirportSpec> airports){

        //get for every drone an airport
        int nbDrones = this.getNbDrones();
        int nbAirports = airports.size();
        if(nbDronesPerAirport*nbAirports < nbDrones){
            throw new IllegalArgumentException("not all drones could be fit on the airports");
        }
        List<AirportSpec> droneAirports = getRandomListElements(airports, nbDrones);
        //now fit the drones on the airports, if there are more drones than airports we first need to fit
        //two drones on each airport until the amount of drones is equal to the amount of airports left to assign
        int dronesToAdd = nbDrones;
        int nbAssignedAirports =  droneAirports.size();
        List<DroneSpec> drones = new ArrayList<>();
        for(int i = 0; i != nbAssignedAirports; i++){
            AirportSpec airport = droneAirports.get(i);
            //check if we need to add one or two drones to the airport
            if(dronesToAdd > nbAssignedAirports){
                addTwoDronesToAirport(drones, airport);
                //decrement drones to add by two
                dronesToAdd -= 2;

            }
            else{
                addSingleDroneToAirport(drones, airport);
                //decrement the drones to add by one
                dronesToAdd --;
            }
        }

        //if we've finished the loop we've placed all the drones to the world
        return drones;
    }

    /**
     * Adds two drones to the specified airport (drones are added as drone specs and they are generated on the specified
     * location) along the two specified runways, both with a given offset to the center (see generation offset)
     * and oriented along the heading of the primary runway & are both placed in gate 1 and zero respectively
     * @param drones the list of drones to add the result to
     * @param airport the airport to add the two drones at (the locations)
     */
    private void addTwoDronesToAirport(List<DroneSpec> drones, AirportSpec airport) {
        //select the runway one and zero heading
        Vector runwayZeroHeading =  airport.getPrimaryRunWay();
        Vector runwayOneHeading = airport.getPrimaryRunWay().scalarMult(-1f);

        //get the center of the airport
        Vector airportCenter = airport.getPosition();

        //we'll also need the width of the airport
        float runwayWidth = this.getAirportRunwayWidth();

        //get the position of the gates
        Vector gateZero = this.getGateZeroPosition(airportCenter, runwayZeroHeading, runwayWidth);
        Vector gateOne = this.getGateOnePosition(airportCenter, runwayOneHeading, runwayWidth);

        //generate the drone specs by calling addDroneToAirport
        addDroneToAirport(drones, gateZero, runwayZeroHeading);
        addDroneToAirport(drones, gateOne, runwayOneHeading);
    }

    /**
     * Adds a single drone to the specified airport aligned with the primary runway (runway zero)
     * and place it in gate zero (the gate on the left of the primary runway)
     * (we don't randomize the runway itself because the orientation of the airport itself is randomized)
     * @param drones the list of drone specifications to add the drone to
     * @param airport the airport to place the drone at
     */
    private void addSingleDroneToAirport(List<DroneSpec> drones,  AirportSpec airport){
        //first get the location and the heading of the airport
        Vector location = airport.getPosition();
        Vector runwayZeroHeading = airport.getPrimaryRunWay();

        //also get the width of the airport, we'll need it to calculate the position of the gate
        float runwayWidth = this.getAirportRunwayWidth();

        //calculate the position of gate zero
        Vector gateZero = this.getGateZeroPosition(location, runwayZeroHeading, runwayWidth);

        //add the drone based on the runway orientation and the location of the airport
        addDroneToAirport(drones, gateZero, runwayZeroHeading);
    }

    /**
     * Getter for the position of gate zero
     * @param airportLocation the location of the airport
     * @param headingVector the heading vector of the airport, this is the orientation of runway zero
     * @param airportRunwayWidth the width of the airport
     * @return the location of gate zero in ground coordinates (x, 0, z) & absolute
     */
    private Vector getGateZeroPosition(Vector airportLocation, Vector headingVector, float airportRunwayWidth){
        //gate zero is located to the left of the heading of the airport
        Vector leftOrthogonal = this.getLeftOrthogonal(headingVector);
        //scale it to half the runway width, this gives the relative position
        Vector relPosGateZero = leftOrthogonal.scalarMult(airportRunwayWidth/2.0f);
        //sum with the airport location to get the absolute position
        return airportLocation.vectorSum(relPosGateZero);

    }

    /**
     * Getter for the position of gate one
     * @param airportLocation the location of the airport
     * @param headingVector the heading vector of the airport, this is the orientation of runway zero
     * @param airportRunwayWidth the width of the airport
     * @return the location of gate one in ground coordinates (x, 0, z) & absolute
     */
    private Vector getGateOnePosition(Vector airportLocation, Vector headingVector, float airportRunwayWidth){
        //gate one is located to the right of the heading of the airport
        Vector rightOrthogonal = this.getRightOrthogonal(headingVector);
        //scale it to half, the width of the runway, this gives the relative position
        Vector relPosGateOne = rightOrthogonal.scalarMult(airportRunwayWidth/2.0f);
        //sum the relative position with the center of the airport, gives the absolute position
        return airportLocation.vectorSum(relPosGateOne);
    }

    /**
     * Gets the vector orthogonal and left to the heading vector of the airport
     * @param headingVector  the heading vector of the airport
     * @return a normalized vector orthogonal and to the left of the heading vector of the airport
     */
    private Vector getLeftOrthogonal(Vector headingVector){
        //if heading is (x1, 0, z1) then scalar product with (z1,0,-x1) will result in a zero
        //the orthogonal will always be located on the left of the heading vector (see drawings)
        Vector leftOrthogonal = new Vector(headingVector.getzValue(),0, -headingVector.getxValue());
        return leftOrthogonal.normalizeVector();
    }

    /**
     * Gets the vector orthogonal and right to the heading vector of the airport
     * @param headingVector  the heading vector of the airport
     * @return a normalized vector orthogonal and to the right of the heading vector of the airport
     */
    private Vector getRightOrthogonal(Vector headingVector){
        //if heading is (x1, 0, z1) then scalar product with (-z1, 0, x1) will give zero
        //the orthogonal will always be located to the right of the heading vector
        Vector secondOrthogonal = new Vector(-headingVector.getzValue(),0, headingVector.getxValue());
        return secondOrthogonal.normalizeVector();
    }
    /**
     * Adds a drone to at the given airport along the given runway heading
     * (the airport is implicitly defined by its location and the runway is selected by specifying the heading vector)
     * (runway one heading is runwayZeroHeading.scalarMult(-1))
     * the orientation of the drone is aligned with the takeoff direction of the runway
     * @param drones the list to add the result to
     * @param airportLocation the location of the airport
     * @param runwayHeading the heading of the runway to place the drone along at
     */
    private void addDroneToAirport(List<DroneSpec> drones, Vector airportLocation, Vector runwayHeading){

        //get the offset along the runway from the center
        float getOffset = this.getDroneGenerationOffset();

        //calculate the offset
        Vector runwayZeroOffset = runwayHeading.normalizeToLength(getOffset);

        //get the location of the drone
        Vector dronePosition = airportLocation.vectorSum(runwayZeroOffset);
        //also account for the height of the drone
        dronePosition = dronePosition.vectorSum(getDroneYPos());
        //we need to do this because of our favorite autistic child named Java
        Vector dronePos =  dronePosition;

        //get the orientation of the drone
        Vector droneOrientation = new Vector(getHeadingAngle(runwayHeading), 0, 0);

        //create the drone
        DroneSpec droneSpec = new DroneSpec() {
            @Override
            public Vector getDronePosition() {
                return dronePos;
            }

            @Override
            public Vector getDroneOrientation() {
                return droneOrientation;
            }
        };

        drones.add(droneSpec);
    }

    /**
     * Getter for the value of the heading for a 2D vector in the xz-plane
     * a negative angle is heading to the right, positive heading to the left
     * the angle is defined against the negative z-axis in the world axis system
     * @param headingVector the heading vector (x, 0, z) to get the heading angle for
     * @return the heading angle calculated from the given heading vector
     */
    private float getHeadingAngle(Vector headingVector){
        //calculate the angle between the heading vector and the negative Z-axis to get the heading angle
        //for the drone
        Vector negZ = new Vector(0,0,-1);
        float angle = negZ.getAngleBetween(headingVector);
        //also get the sign from the y component, negative sign is right, positive is left (do negZ x HeadingVector)
        float sign =  signum(negZ.crossProduct(headingVector).getyValue());

        //check for NaN in the angle
        if(Float.isNaN(angle)){
            return 0;
        }

        return angle*sign;
    }

    /**
     * Randomly selects a sublist containing the specified number of elements
     * if the supplied number of elements is larger than the actual amount then the whole list will be returned
     * always generates a shallow copy of the list provided (even in the case of the sublistSize > list.size())
     * @param list the list to select the elements from
     * @param sublistSize the size of the list containing the random picked elements
     * @param <T> the Type of the list
     * @return a list of type T with size min(sublistSize, list.size())
     */
    private <T> List<T> getRandomListElements(List<T> list, int sublistSize){
        //first generate the duplicate list
        List<T> result = new ArrayList<>();
        // make a shallow copy
        result.addAll(list);
        int providedListSize = list.size();
        //get the number of elements we need to remove from the result list
        //we'll remove elements until the size of the result is equal to the size of the sublist
        Random random = new Random(this.getRandomSeed());
        int nbToRemove = providedListSize - sublistSize;
        //loop until we've removed the appropriate number of elements
        //loop will be automatically skipped if the sublist size > list.size();
        for(int i = 0; i < nbToRemove; i++){
            //the max index of the list
            int indexBound = providedListSize - i;
            int randomIndex = random.nextInt(indexBound);
            result.remove(randomIndex);
        }

        //return the list after removing the appropriate amount of elements
        return result;
    }

    /**
     * Generates a grid of airports located in the specified number of rows and columns
     * with the centers located within or on the borders of the world (-worldXSize, worldXSize) for x and
     * (-worldZSize, worldZSize) for z.
     * The airports need at least to be max(runwayLength, runwayWidth) apart from each other as the
     * heading of the aiports is determined randomly
     * @param nbRows the number for rows of airports in the world
     * @param nbColumns the number of columns of airports in the world
     */
    private List<AirportSpec> generateAirportsGrid(int nbRows, int nbColumns) {
        //calculate the distance between each airport center that will be needed
        float worldX = this.getWorldXSize();
        float worldZ = this.getWorldZSize();

        float xSpacing = 2*worldX/nbRows;
        float zSpacing = 2*worldZ/nbColumns;


        //set the base to start the allocation at (just the world size, we start at (-worldX, -WorldZ) and go
        //all the way up to (WorldX,WorldZ)

        float worldXBase = - worldX;
        float worldZBase = - worldZ;

        //used to create the airports
        float runwayWidth = this.getAirportRunwayWidth();
        float runwayLength = this.getAirportRunwayLength();
        //also create random to randomly generate the heading of the world
        Random random = new Random(System.currentTimeMillis()%8456);

        List<AirportSpec> airportSpecs = new ArrayList<>();

        for(int rowIndex = 0; rowIndex != nbRows; rowIndex++){
            float xCoord = rowIndex*xSpacing + worldXBase;
            for(int columnIndex = 0; columnIndex != nbColumns; columnIndex++){
                float zCoord = columnIndex*zSpacing + worldZBase;
                float headingX = random.nextFloat() - 0.5f;
                float headingZ = random.nextFloat() - 0.5f;



                AirportSpec airportSpec = new AirportSpec() {
                    @Override
                    public Vector getPosition() {
                        return new Vector(xCoord, 0, zCoord);
                    }

                    @Override
                    public Vector getPrimaryRunWay() {
                        return new Vector(headingX, 0, headingZ).normalizeVector();
                    }

                    @Override
                    public float getRunwayWidth() {
                        return runwayWidth;
                    }

                    @Override
                    public float getRunwayLength() {
                        return runwayLength;
                    }
                };

                airportSpecs.add(airportSpec);
            }
        }

        //return the resulting list
        return airportSpecs;
    }

    /**
     * Checks if the airports in the given airport specifications can overlap
     * the "can overlap" criterion is defined as following:
     * take the distance from the center of the airport to a corner of the runways (done by pythagoras)
     * and check if the distance between all the airports is larger than 2 times the maximum disance
     * (all airports are defined with an "airport radius")
     * @param airportSpecs the specifications of the airports to check
     * @return true if there is at least one set of airports that overlap
     */
    private boolean airportsCanOverlap(List<AirportSpec> airportSpecs){
        //checks if any of the airports can overlap in the given sequence of airport specs
        int nbAirports = airportSpecs.size();
        //runway width and length
        float runwayLength = this.getAirportRunwayLength();
        float runwayWidth = this.getAirportRunwayWidth();
        //get the radius of the airport, this is defined as the max distance between the center of the airport
        //and the outer corner of the runways (distance is given by pythagorean theorem
        float airportRadius = (float) sqrt(runwayLength*runwayLength + runwayWidth*runwayWidth);

        for(int i = 0; i != nbAirports; i++){
            //get the location of the airport
            Vector location1 = airportSpecs.get(i).getPosition();
            //now check against all the airports with index i + 1... nbAirports
            //i+1 so we do not check against ourselves
            for(int j = i+1; j < nbAirports; j++){
                Vector location2 = airportSpecs.get(j).getPosition();
                //get the distance between both locations
                float distanceBetween = location1.distanceBetween(location2);
                //if they can overlap (we wont actually check, the world would look horrible
                //if the airports are all fitted together tightly
                if(distanceBetween <= 2*airportRadius){
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Checks if the requested grid world can be generated with the given amount of airports that may be generated
     * @param nbRows the number of rows in the grid to generate
     * @param nbColumns the number of columns in the grid to generate
     * @return true if and only if the product of the nb of rows and columns equals the number of airports
     *         that have to be added to the simulation
     */
    private boolean canGenerateGrid(int nbRows, int nbColumns){
        return this.getNbAirports() == nbColumns*nbRows;
    }

    /**
     * Generates the testbed server based on the given package service and the generated simulation environment
     * @param simulationEnvironment the environment that the testbed has to simulate
     * @param packageService the package service used by the world to coordinate the package transport
     * @return the server that was generated
     */
    public TestbedServer generateTestbedServer(SimulationEnvironment simulationEnvironment, PackageService packageService){
        List<AirportSpec> airportSpecs = simulationEnvironment.getAirportSpecifications();
        List<DroneSpec> droneSpecs = simulationEnvironment.getDroneSpecifications();
        int tcpPort = this.getTcpPort();
        int nbStepsPerCycle = this.getStepsPerCycle();
        int nbStepsPerSubCycle = this.getNbOfSubSteps();
        float timeStep = this.getTimeStep();
        int maxNbThreads = droneSpecs.size();
        TestbedServer.TestbedServerConfig  serverConfig = new TestbedServer.TestbedServerConfig() {
            @Override
            public int getTcpPort() {
                return tcpPort;
            }

            @Override
            public int getStepsPerCycle() {
                return nbStepsPerCycle;
            }

            @Override
            public int getStepsPerSubCycle() {
                return nbStepsPerSubCycle;
            }

            @Override
            public float getTimeStep() {
                return timeStep;
            }

            @Override
            public int getMaxNbThreads() {
                return maxNbThreads;
            }

            @Override
            public PackageService getPackageService() {
                return packageService;
            }

            @Override
            public List<DroneSpec> getDroneSpecifications() {
                return droneSpecs;
            }

            @Override
            public List<AirportSpec> getAirportSpecifications() {
                return airportSpecs;
            }
        };

        TestbedServer testbedServer = new TestbedServer(serverConfig);

        return testbedServer;
    }

    /**
     * Generator for the overseer based on the simulation environment object
     * @param simulationEnvironment the simulation environment to cofigure the overseer with
     * @return an autopilot overseer configured for the given simulation environment
     */
    public static AutopilotOverseer generateOverseer(SimulationEnvironment simulationEnvironment){
        List<AirportSpec> airports = simulationEnvironment.getAirportSpecifications();
        List<DroneSpec> drones = simulationEnvironment.getDroneSpecifications();
        Set<DeliverySpec> deliveryPackages = simulationEnvironment.getDeliveryPackages();

        int nbDrones = drones.size();
        System.out.println("nbDrones specified upon creation: " + nbDrones);

        AutopilotOverseer overseer = new AutopilotOverseer(nbDrones);
        //add the airports to the overseer
        configureOverseerAirports(airports, overseer);
        configureOverseerPackages(deliveryPackages, overseer);

        return overseer;
    }

    /**
     * Configures the overseer by adding all the packages in the set to the specified overseer
     * (these packages will be added to the buffer)
     * @param packages the packages to add to the overseer (and to be delivered)
     * @param overseer the overseer to add the packages to
     */
    private static void configureOverseerPackages(Set<DeliverySpec> packages, AutopilotOverseer overseer){
        //add all the packages to the overseer, no need for special ordering, only use the api provided by
        //the didactic team
        for(DeliverySpec delivery: packages){
            int fromAirport = delivery.getSourceAirport();
            int fromGate = delivery.getSourceAirportGate();
            int toAirport = delivery.getDestinationAirport();
            int toGate = delivery.getDestinationAirportGate();
            overseer.deliverPackage(fromAirport, fromGate, toAirport, toGate);
        }
    }

    /**
     * Configures the overseer with the given airport specs
     * 1. sets the airport parameters as specified in the overseer API
     * 2. adds the airports in the same order as they were defined
     * @param airports the airports to add to the overseer
     * @param overseer the overseer to add the airports to
     */
    private static void configureOverseerAirports(List<AirportSpec> airports, AutopilotOverseer overseer) {
        for(int index = 0; index != airports.size(); index++) {
            AirportSpec airport = airports.get(index);

            //if first iterations configure airport
            if (index == 0) {
                float length = airport.getRunwayLength();
                float width = airport.getRunwayWidth();
                overseer.defineAirportParams(length, width);
            }

            //define the airport itself
            Vector position = airport.getPosition();
            Vector heading = airport.getPrimaryRunWay();
            overseer.defineAirport(position.getxValue(), position.getzValue(), heading.getxValue(), heading.getzValue());
        }
    }

    /**
     * Converts the provided simulation environment into a string summarizing the data of the simulation
     * the string contains:
     * -> id, location and heading of the airports
     * -> location and orientation of the drones
     * -> source gate & airport and destination airport & gate of the deliveries
     * @param simulationEnvironment the simulation environment that needs to be converted
     * @return a string summarizing the properties of the simulation
     */
    public static String environmentToString(SimulationEnvironment simulationEnvironment){

        //get the lists that need to be converted to a string
        List<DroneSpec> droneSpecs = simulationEnvironment.getDroneSpecifications();
        List<AirportSpec> airportSpecs = simulationEnvironment.getAirportSpecifications();
        Set<DeliverySpec> deliverySpecs = simulationEnvironment.getDeliveryPackages();

        //generate the string builder to create the sting
        StringBuilder stringBuilder = new StringBuilder();

        //message the user
        stringBuilder.append("Simulation data:\n\n\n");

        //get the airport string data and add to the builder
        stringBuilder.append("Airports in simulation:\n\n");
        String airportString = airportSpecToString(airportSpecs);
        stringBuilder.append(airportString);

        //get drone string data and add to the builder
        stringBuilder.append("Drones in simulation: \n\n");
        String droneString = droneSpecToString(droneSpecs);
        stringBuilder.append(droneString);

        //get delivery string and add to the builder
        stringBuilder.append("Delivery data:\n\n");
        String deliveryString = deliverySpecToString(deliverySpecs);
        stringBuilder.append(deliveryString);

        return stringBuilder.toString();
    }

    /**
     * Converts the privided sat of delivery specifications into a string
     * that summarizes the data
     * @param deliverySpecs the specifications of the deliveries to convert
     * @return a string containing useful data about the deliveries
     */
    private static String deliverySpecToString(Set<DeliverySpec> deliverySpecs){

        StringBuilder stringBuilder = new StringBuilder();

        for(DeliverySpec delivery: deliverySpecs){
            int sourceAirport = delivery.getSourceAirport();
            int sourceGate = delivery.getSourceAirportGate();

            int destAirport = delivery.getDestinationAirport();
            int destGate = delivery.getDestinationAirportGate();

            String source = "Source airport id: " + sourceAirport + ", gate id: " + sourceGate + "\n";
            String dest = "Destination airport id: " + destAirport + ", gate id: " + destGate + "\n\n";

            stringBuilder.append(source);
            stringBuilder.append(dest);
        }

        return stringBuilder.toString();
    }

    /**
     * Converts the given drone specifications to a string summarizing the drones
     * @param droneSpecs the specifications to convert to a string
     * @return a string containing useful info about the drones provided
     */
    private static String droneSpecToString(List<DroneSpec> droneSpecs) {

        StringBuilder stringBuilder = new StringBuilder();


        for(DroneSpec drone: droneSpecs){
            Vector position = drone.getDronePosition();
            Vector orientation = drone.getDroneOrientation();

            String dronePos = "Drone position: " + position + "\n";
            String droneOrient = "Drone orientation: " + orientation + "\n\n";

            stringBuilder.append(dronePos);
            stringBuilder.append(droneOrient);
        }

        return stringBuilder.toString();
    }

    /**
     * Converts the airport specifications in the given list to a string containing the data
     * @param airportSpecs the airport specs to convert to a string
     * @return a string containing useful data about the airports provided
     */
    private static String airportSpecToString(List<AirportSpec> airportSpecs) {
        StringBuilder stringBuilder = new StringBuilder();

        int index = 0;
        for(AirportSpec airportSpec:airportSpecs){
            Vector location = airportSpec.getPosition();
            Vector heading = airportSpec.getPrimaryRunWay();

            String airportIndex = "Airport ID: " + index + "\n";
            String airportLocation = "Airport location: " + location + "\n";
            String airportHeading = "Airport heading" + heading + "\n\n";

            stringBuilder.append(airportIndex);
            stringBuilder.append(airportLocation);
            stringBuilder.append(airportHeading);

            index++;
        }

        return stringBuilder.toString();
    }

    /*
    Instances with predetermined values (advanced options, so we can add extra features later)
     */

    /**
     * Getter for the runway width of the airports to be created within the simulation
     * @return a floating point containing the width of the runways
     */
    private float getAirportRunwayWidth() {
        return airportRunwayWidth;
    }

    /**
     * Setter for the width of the runways of the airports created in the simulation
     * @param airportRunwayWidth the width to be assigned to the airports
     */
    public void setAirportRunwayWidth(float airportRunwayWidth) {
        if(airportRunwayWidth <= 0) {
            throw new IllegalArgumentException("negative value");
        }
        this.airportRunwayWidth = airportRunwayWidth;
    }

    /**
     * Getter for the length of the runways of the airports in the simulation
     * @return a floating point containing the length of the runways
     */
    private float getAirportRunwayLength() {
        return airportRunwayLength;
    }

    /**
     * Setter for the length of the runways of the airports created in the simulation
     * @param airportRunwayLength the desired length of the airports
     */
    public void setAirportRunwayLength(float airportRunwayLength) {
        if(airportRunwayLength <= 0) {
            throw new IllegalArgumentException("negative value");
        }
        this.airportRunwayLength = airportRunwayLength;
    }

    /**
     * Getter for the number of simulation steps there are taken for each simulation cycle
     */
    private int getStepsPerCycle() {
        return stepsPerCycle;
    }

    /**
     * Setter for the number of simulation steps there are taken for each simulation cycle
     * @param stepsPerCycle the steps that are taken per simulation cycle
     */
    public void setStepsPerCycle(int stepsPerCycle) {
        this.stepsPerCycle = stepsPerCycle;
    }

    /**
     * Getter for the number of sub steps there are taken during the simulation cycle
     * only after n sub steps there is made a check if the drones have collided/crashed or if they
     * can deliver a package
     */
    private int getNbOfSubSteps() {
        return nbOfSubSteps;
    }

    /**
     * Setter for the number of sub steps used in the simulation (see getter for more info)
     * @param nbOfSubSteps the number of sub steps taken
     */
    public void setNbOfSubSteps(int nbOfSubSteps) {
        this.nbOfSubSteps = nbOfSubSteps;
    }

    /**
     * Getter for the time step used in the simulation
     */
    private float getTimeStep() {
        return timeStep;
    }

    /**
     * Setter for the time step used in the simulation
     * @param timeStep the simulation step to be used
     */
    public void setTimeStep(float timeStep) {
        this.timeStep = timeStep;
    }

    /**
     * Getter for the TCP port where the server will be listening for connections
     * @return the tcp port used by the server to listen for connections
     */
    private int getTcpPort() {
        return tcpPort;
    }

    /**
     * Setter for the TCP port used by the server
     * @param tcpPort the tcp port to be used
     */
    public void setTcpPort(int tcpPort) {
        this.tcpPort = tcpPort;
    }

    /**
     * Getter for the hostname of the server
     * @return string containing the hostname of the server
     */
    private String getHost() {
        return host;
    }

    /**
     * Setter for the hostname that will be used by the server
     * (normally this should always be localhost... but who knows)
     * @param host
     */
    public void setHost(String host) {
        this.host = host;
    }

    /**
     * Getter for the offset from the center (along the heading of the runway) where
     * the drones are generated
     * @return the offset used from the center to generate the drones
     */
    private float getDroneGenerationOffset() {
        return droneGenerationOffset;
    }

    /**
     * Setter for the generation distance from the center of the airports (along the heading of the runway)
     * @param droneGenerationOffset the offset to set for the drones
     */
    public void setDroneGenerationOffset(float droneGenerationOffset) {
        this.droneGenerationOffset = droneGenerationOffset;
    }

    /**
     * Getter for the seed used for the random number generators, used to generate the same simulation
     * for testing (just set a fixed seed, the world will be the same on every init)
     * @return the seed to be used by the random number generator
     */
    private int getRandomSeed() {
        return randomSeed;
    }

    /**
     * Setter for the seed of the random number generator (used to get the same simulation for testing)
     * @param randomSeed the seed to set
     */
    public void setRandomSeed(int randomSeed) {
        this.randomSeed = randomSeed;
    }

    /**
     * Getter for the y position of the drone, used to account for the chassis
     * @return the y position of the drone in vector format (0,y,0)
     * note: returned as a vector for ease of use
     */
    private Vector getDroneYPos() {
        return droneYPos;
    }

    /**
     * Setter for the y position of the drone, used to account for the size of the chassis
     * @param droneYPos the y position of the drone
     */
    public void setDroneYPos(float droneYPos) {
        this.droneYPos = new Vector(0, droneYPos, 0);
    }

    /**
     * The width of the runway of the airports (same value for all airports)
     */
    private float airportRunwayWidth = 15f;

    /**
     * The length of the runway of the airports (same values for all the airports within a world
     */
    private float airportRunwayLength = 400f;

    /**
     * The number of steps that should be taken by the testbed in one simulation cycle
     * a cycle is generating the outputs for the autopilot and receiving commands from it
     */
    private int stepsPerCycle = 50;

    /**
     * The number of simulation steps there have to elapse before the drones are checked for a crash
     * and/or package delivery
     */
    private int nbOfSubSteps = 10;
    /**
     * The in simulation timestep used
     */
    private float timeStep = 0.001f;

    /**
     * The port on which the server is listening for connections
     */
    private int tcpPort = 4242;

    /**
     * The hostname where the server is listening for connections
     */
    private String host = "localhost";

    /**
     * The offset from the drone to the center of the airport
     */
    private float droneGenerationOffset = 10f;

    /**
     * The seed used for the random number generators, used to test on the same world
     * if seed is set to a fixed value, default: random seed
     */
    private int randomSeed = new Random().nextInt();

    /**
     * The y position of the drone, may be modified by the user, used to account for the offset of the drone
     * such that the wheels almost touch the ground
     */
    private Vector droneYPos = new Vector(0, 1.20f, 0);



    /*
    Instances with no predetermined standard value
     */

    /**
     * Getter for the x-size of the world used to specify the range along the x-axis in which airports can be generated
     * @return float containing the width of the world
     */
    private float getWorldXSize() {
        return worldXSize;
    }

    /**
     * Getter for the breadth of the world used as the breadth range in which airports can be generated
     * @return float containing the breadth of the world
     */
    private float getWorldZSize() {
        return worldZSize;
    }

    /**
     * Getter for the nb of drones to be added to the simulation
     * @return the number of drones to add
     */
    public int getNbDrones() {
        return nbDrones;
    }

    /**
     * Getter for the number of airports to be added to the simulation
     * @return the number of airports to add to the simulation
     */
    private int getNbAirports() {
        return nbAirports;
    }

    /**
     * Getter for the number of packages that initially need to be delivered by the drones
     * in the simulation (there may be added packages to the simulation later on
     * @return the number of packages to deliver
     */
    private int getNbPackages() {
        return nbPackages;
    }

    /**
     * The size of the world measured along the x-axis
     */
    private float worldXSize;

    /**
     * The size of the world measured along the z-axis
     */
    private float worldZSize;

    /**
     * The number of drones to be added to the simulation
     */
    private int nbDrones;

    /**
     * The number of airports to be added to the simulation
     */
    private int nbAirports;

    /**
     * The number of packages that need to be added initially to the simulation
     * (more can be submitted later)
     */
    private int nbPackages;

    /**
     * Constant defining how many drones can fit on a single airport when the simulation is generated
     */
    private final static int nbDronesPerAirport = 2;

    /**
     * Constant defining the upper gate id for airports
     */
    private final static int maxGateID = 1;

    /*
    Error messages
     */

    private static final String AIRPORT_FIT_EXCEPTION = "The world is too small to fit all the airports please try again with another size.";
    private static final String GRID_SIZE_ERROR = "The product of nbRows and nbColumns does not equal the nb of airports to add";


}
