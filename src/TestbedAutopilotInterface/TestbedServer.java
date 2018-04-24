package TestbedAutopilotInterface;


import TestbedAutopilotInterface.GUI.GUIQueueElement;
import TestbedAutopilotInterface.GUI.TestbedGUI;
import TestbedAutopilotInterface.Overseer.PackageService;
import TestbedAutopilotInterface.SimulationSetup.AirportSpec;
import TestbedAutopilotInterface.SimulationSetup.DroneSpec;
import internal.Exceptions.AngleOfAttackException;
import internal.Exceptions.SimulationEndedException;
import internal.Helper.SquareMatrix;
import internal.Testbed.Drone;
import internal.Testbed.World;
import internal.Testbed.WorldBuilder_v2;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 23/02/2018.
 * A class to host many requests for testbed connections by autopilot (acts as a server with multiple connections
 */

// Todo implement the following scheme for the server side:
//1. build a new world (blocks, airports etc, no drones)
//2. add the drones to the world via drone builder
//3. start the GUI
//4. start the server and listen for connections
//5. for every new connection create a new thread by assigning each thread a drone
//6. simulate the first step (config and state are sent via the threads)
//7. wait for the threads to send their data and pass the commands of the autopilot to their corresponding drones
//8. if the threads are finished setting the parameters for the next iteration start a new loop in the simulation
//note on threads: at the start of the connection all the Testbed threads are created and a thread pool to accompany them
//they are used for communication only and are re-used each execution cycle
//note: the server operates on port 4242 (TCP) on "localhost" (IP)

public class TestbedServer implements Runnable {

    /**
     * Constructor for a testbed server, used for communication with the autopilots
     * @param timeStep the time step in seconds
     * @param stepsPerCycle the amount of steps that are taken within a single execution cycle
     * @param maxNbThreads the max number of threads the server can handle in the executor service
     * @param tcpPort the port the server operates on
     * @param airports the airports to be set in the world, the first entry of the vector is the location
     *                 and the second is the heading vector of runway zero
     */
    public TestbedServer(float timeStep, int stepsPerCycle, int stepsPerSubCycle, int maxNbThreads, int tcpPort,
                         PackageService packageService, List<AirportSpec> airports, List<DroneSpec> drones) {
        this.timeStep = timeStep;
        this.stepsPerCycle = stepsPerCycle;
        this.stepsPerSubCycle = stepsPerSubCycle;
        this.maxNbOfThreads = maxNbThreads;
        this.tcpPort = tcpPort;
        this.packageService = packageService;
        this.airportSpecs = airports;
        this.droneSpecs = drones;
    }

    /**
     * Constructor for a testbed server used for compact initialization, has exactly the same effect
     * as constructing with the expanded testbed server
     * @param config the configuration for the server (see interface documentation for more)
     */
    public TestbedServer(TestbedServerConfig config){
        this.timeStep = config.getTimeStep();
        this.stepsPerCycle = config.getStepsPerCycle();
        this.stepsPerSubCycle = config.getStepsPerSubCycle();
        this.maxNbOfThreads = config.getMaxNbThreads();
        this.tcpPort = config.getTcpPort();
        this.packageService = config.getPackageService();
        this.airportSpecs = config.getAirportSpecifications();
        this.droneSpecs = config.getDroneSpecifications();
    }

    /**
     * Run method, gets called upon thread creation for this class (own main loop)
     */
    @Override
    public void run() {
        //first initialize the testbed server
        this.initTestbedServer();
        //then start simulating the server
        try {
            this.serverMainLoop();
        }catch(Exception e){
            //check for any exception during the execution of the main loop
            e.printStackTrace();
        }
    }

    /**
     * The main loop of the server
     */
    private void serverMainLoop(){
        boolean simulationActive = true;
        while(simulationActive){
            try{
                //simulate the next step
                this.simulateStep();
            }catch(java.io.EOFException | SimulationEndedException e) {
                //the connection at the other side has closed or the simulation has ended
                //terminate the testbed
                System.out.println("terminating testbed");
                this.terminateServer();
                simulationActive = false;

            }catch(AngleOfAttackException angleOffAttackException){
                //angle of attack exception occurred
                this.terminateServer();
                System.out.println("terminating testbed");
                throw angleOffAttackException;

            }catch(IOException | InterruptedException e){
                //no clue what happened
                System.out.println("Interrupted");
                e.printStackTrace();
            }

//            System.out.println("next iteration");

        }
    }

    /*
    Methods for the simulation cycle
     */
    private void simulateStep() throws IOException, InterruptedException {

        //first open up the connection and communicate the state to the autopilot
        //wait for a response
        autopilotCommunication();

        //now that all the communication is done, we can simulate the next step
        //(all the drones have received their commands for the next step)
        long start = System.currentTimeMillis();
        advanceWorld();
        long end = System.currentTimeMillis();
//        System.out.println("Elapsed time: " + (end - start) + "millis");
        //once the world is advanced, we are finished here
    }

    /**
     * Sends the current state to all the connections which in turn send the state to the connected autopilots
     * Post: the commands needed to simulate the next step are in place for each drone (the autopilotOutputs)
     * @throws InterruptedException i have no idea why (something with interrupted threads, but we dit not put timers in place)
     */
    private void autopilotCommunication() throws InterruptedException {
        //first send all the connections to the thread pool
        ExecutorService threadPool = this.getThreadPool();
        Set<TestbedConnection> connections = this.getServerTestbedConnections();
        //invoke all the connections to get a response from their connected autopilots
        List<Future<Void>> busyConnectionList = threadPool.invokeAll(connections);
        //now we wait for all the connections to finish
        boolean allConnectionsFinished = false;
        while(!allConnectionsFinished){
            //wait for the first element to finish
            busyConnectionList.get(0);
            //then filter for futures that are not yet complete (keep em)
            busyConnectionList = busyConnectionList.stream()
                    .filter(connection -> !connection.isDone())
                    .collect(Collectors.toList());
            //if all the elements are finished then all the communication is done
            if(busyConnectionList.size() == 0){
                allConnectionsFinished = true;
            }
        }
        // if all communication is done, return
    }

    /**
     * Advances the world state n times for a given time step t
     */
    private void advanceWorld() throws InterruptedException, IOException {
        World world = this.getWorld();
        float timeStep = this.getTimeStep();
        int stepsPerCycle = this.getStepsPerCycle();
        int stepsPerSubCycle = this.getStepsPerSubCycle();
        //advance the state of the world for n steps of time delta t
        world.advanceWorldState(timeStep, stepsPerCycle, stepsPerSubCycle);
        //increment the time that was simulated
        this.incrementElapsedTime();
//        System.out.println(world.toString());
        //communicate the new world state with the GUI
        addFrameToGuiQueue();
    }

    /**
     * Called for proper termination of the server (closing all sockets and streams etc)
     */
    private void terminateServer(){
        for(TestbedConnection connection: this.getServerTestbedConnections()){
            connection.terminateConnection();
        }
        try {
            this.getServerSocket().close();
        } catch (IOException e) {
            //doesn't matter if we're closing down
            e.printStackTrace();
        }
    }

    /**
     * Used to convey the current state of the testbed to the gui, creating the
     * necessary data and putting it inside the queue, also deals with the frame rate control
     * so the testbed wont go too far ahead of the renderer (if even possible)
     */
    private void addFrameToGuiQueue(){
        //add the element to the queue
        ConcurrentLinkedQueue<GUIQueueElement> renderQueue = this.getRendererQueue();
        //extract the queue element from the world
        World world = this.getWorld();
        GUIQueueElement newFrame = world.getGuiElements();
//        System.out.println("drone states: " + newFrame.getDroneStates());
        //insert the frame into the queue
        renderQueue.add(newFrame);
        //check if the queue size is smaller than the maximum allowed, if not, wait
        while(renderQueue.size() > MAX_FRAMES_AHEAD){
            if(!guiBNsignaled){
                System.out.println("buffer full");
                guiBNsignaled = true;
            }
//
            //if not, sleep for the time one frame needs to be rendered
            try {
                float sleepTime = this.getTimeStep()*this.getStepsPerCycle()*1000f; // the times 1000 for converting to millis
                Thread.sleep((long) sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    private boolean guiBNsignaled = false;

    /*
    Initialization methods for the testbed server
     */

    /**
     * Initializes the testbed (see implementations and comments along the lines for further explanation
     */
    private void initTestbedServer() {

        this.initGui();

        this.initThreads();

//
//        // initialize the windows
//        this.initWindows();
        // initialize the world that needs to be simulated
        this.initWorld();
        // initialize the server
        this.initServer();

    }

    /**
     * Initializes the thread pool needed to simulate the world
     */
    private void initThreads() {
        ExecutorService threadPool = Executors.newFixedThreadPool(this.getMaxNbOfThreads());
        //set the thread pool
//        System.out.println(threadPool);
        this.setThreadPool(threadPool);
    }

    private void initGui() {
        ConcurrentLinkedQueue<GUIQueueElement> guiQueue = this.getRendererQueue();
        //create the gui
        TestbedGUI gui = new TestbedGUI(guiQueue);
        Thread guiThread = new Thread(gui);
        guiThread.start();

    }

    /**
     * Initializer for the world that needs to be simulated by the server
     */
    // note: the amount of created drones is equal to the number of connections we can make
    private void initWorld(){
        List<AirportSpec> airportSpecs = this.getAirportSpecs();
        System.out.println("nb Airports: " + airportSpecs.size());
        List<DroneSpec> droneSpecs = this.getDroneSpecs();
        PackageService packageService = this.getPackageService();
        WorldBuilder_v2 builder = new WorldBuilder_v2();
        World world = builder.createMultiDroneWorld(this.getThreadPool(), packageService, droneSpecs, airportSpecs);
        //add the airport
        this.setWorld(world);
        //send the newly created world to the GUI
        addFrameToGuiQueue();
    }

    /**
     * Initializes the server and creates the objects needed for the connections
     */
    private void initServer(){
        //create the server socket
        ServerSocket serverSocket = null;
        try {
            serverSocket = new ServerSocket(getTcpPort());
        } catch (IOException e) {
            //the socket could not be created... I have no idea why, so lets print the issue
            e.printStackTrace();
        }
        //we need not to worry about null references, if server socket could not be created, the program terminates
        //abnormally
        this.setServerSocket(serverSocket);
        initConnections();

    }

    /**
     * Creates the connections needed for communication with the autopilots using the drones that
     * are contained within the world (adds the connections to the connection set)
     */
    private void initConnections() {
        //start listening for connections, creating connections until all the available connection spots are filled
        //get the world that is simulated and grab all the drones in a set
        World world = this.getWorld();
        Set<Drone> droneSet = world.getDroneSet();
        for(Drone drone: droneSet){
            try {
                //listening is a thread blocking method, just listen
                Socket socket = this.getServerSocket().accept();
                //if the socket is created hook a drone up to the connection
                TestbedConnection connection = new TestbedConnection(socket, drone, this);
                //add the connection to the connection set
                this.getServerTestbedConnections().add(connection);
            } catch (IOException e) {
                // again not a clue what went wrong, better print some stack trace
                e.printStackTrace();
            }
        }
        //if all the drones are assigned a connection we may exit
    }

    /*
    Getter and setters for physics stuff #############################
     */

    /**
     * Getter for the world simulated by the server, containing all
     * the drones and cubes
     * @return the world that is simulated
     */
    private World getWorld() {
        return world;
    }

    /**
     * Setter for the world that is simuated by the server, contains all the drones and cubes
     * @param world the world to simulate
     */
    private void setWorld(World world) {
        this.world = world;
    }

    /**
     * Getter for the package service to be used by the world that is simulated by the testbed
     * @return a package service used by the world
     */
    private PackageService getPackageService() {
        return packageService;
    }

    /**
     * Getter for the specifications of the airports to be added to the world
     * @return a list containing the specifications of the airports to be added
     */
    private List<AirportSpec> getAirportSpecs() {
        return airportSpecs;
    }

    /**
     * Getter for the specifications of the drones to be added to the world
     * @return a lis containing droneSpec objects containing the info needed to generate the drones in the world
     */
    private List<DroneSpec> getDroneSpecs() {
        return droneSpecs;
    }

    /**
     * Getter for the in-simulation flight duration
     * @return the elapsed time
     */
    public float getElapsedTime() {
        return elapsedTime;
    }

    /**
     * Increments the in-simulation elapsed time with the given time step
     * elapsed time += getTimeStep()
     */
    private void incrementElapsedTime(){
        this.elapsedTime += this.getTimeStep()*this.getStepsPerCycle();
    }

    /**
     * Getter for the in-simulation time step in seconds
     * @return the time step in seconds
     */
    private float getTimeStep() {
        return timeStep;
    }

    /**
     * Getter for the number of steps that are taken each simulation cycle (no communication with the autopilot in between)
     * @return the nb op steps per simulation cycle
     */
    private int getStepsPerCycle() {
        return stepsPerCycle;
    }

    /**
     * Getter for the number of cycles that the world simulates before doing another check (used for acceleration)
     * @return an integer containing the number of sub cycles to do while simulating a drone
     */
    public int getStepsPerSubCycle() {
        return stepsPerSubCycle;
    }

    /*
    Physics related instances ########################################
     */

    /**
     * Object that stores the world that will be simulated
     */
    private World world;

    /**
     * The package service to be used by the world
     */
    private PackageService packageService;

    /**
     * The specifications of the airports to be added to the world
     */
    private List<AirportSpec> airportSpecs;

    /**
     * The specifications of the drones to be added to the world
     */
    private List<DroneSpec>  droneSpecs;

    /**
     * Variable that stores the in-simulation duration of the flight
     */
    private float elapsedTime = 0;

    /**
     * Variable that stores the in simulation time between the simulation steps (in seconds)
     */
    private float timeStep;

    /**
     * Variable that stores the number of steps that are simulated within as single execution cycle
     */
    private int stepsPerCycle;

    /**
     * Variable for the number of sub-cycles for every simulation cycle, used to reduce the overhead for the testbed
     * --> number of sub-cycles are the number of simulation steps one drone simulates before a check in the
     *     world class occurs for crashes and package delivery
     */
    private int stepsPerSubCycle;
   /*
   Server related getters and setters #################################
    */

    /**
     * Getter for the server socket, listens for connections with autopilots
     * @return a the server socket used by the testbed server for outside connections
     */
    private ServerSocket getServerSocket() {
        return serverSocket;
    }

    /**
     * Setter for the server socket used for connecting with the autopilot
     * @param serverSocket the server socket used for connections
     */
    private void setServerSocket(ServerSocket serverSocket) {
        if(!this.canHaveAsServerSocket(serverSocket))
            throw new IllegalArgumentException(INVALID_SERVER_SOCKET);
        this.serverSocket = serverSocket;
    }

    private boolean canHaveAsServerSocket(ServerSocket serverSocket){
        return serverSocket != null && this.getServerSocket() == null;
    }

    /**
     * Getter for the port to connect with
     * @return the TCP port used for connections
     */
    public int getTcpPort() {
        return tcpPort;
    }

    /**
     * Getter for the list containing all the connections maintained by the testbed
     * @return the set of all current connections
     */
    private Set<TestbedConnection> getServerTestbedConnections() {
        return serverTestbedConnections;
    }

    /**
     * Getter for the number of threads maintained by the server for the simulation
     * @return the number of threads active in the current thread pool
     */
    private int getMaxNbOfThreads() {
        return maxNbOfThreads;
    }

    /**
     * Getter for the thread pool used by the server to maintain the connections
     * @return the thread pool
     */
    private ExecutorService getThreadPool() {
        return threadPool;
    }

    /**
     * Setter for the thread pool of the server
     * @param threadPool the thread pool for the server
     */
    private void setThreadPool(ExecutorService threadPool) {
        if(!canHaveAsThreadPool(threadPool))
            throw new IllegalArgumentException(INVALID_THREAD_POOL);
        this.threadPool = threadPool;
    }

    /**
     * Checks if the provided thread pool can be assigned as the thread pool of the server
     * @param threadPool the thread pool to be tested
     * @return true if and only if the provided thread pool is a non null reference and the
     *         server doesn't already have an initialized thread pool
     */
    private boolean canHaveAsThreadPool(ExecutorService threadPool){
        return threadPool != null && this.getThreadPool() == null;
    }

    /*
    Server related instances ##########################################
     */
    /**
     * Object that stores the socket of the server that listens for connections
     */
   private ServerSocket serverSocket;

    /**
     * Variable that stores the current used port to listen for TCP connections
     */
    private int tcpPort = 4242;


    /**
     * Object that contains a list that stores all the current server threads
     * choice for set so connections can easily be removed
     */
    private Set<TestbedConnection> serverTestbedConnections = new HashSet<>();


    //Todo maybe use one shared thread pool for the world simulation and
    //Todo and the connections since the execution doesn't overlap (resource sharing)
    //note: the nb of threads may be less than the nb of connections if better for performance (submit more tasks to
    //the thread pool than there are threads available)
    /**
     * Variable that stores the number of threads available in the
     * thread pool of the server
     */
    private int maxNbOfThreads;

    /**
     * Object that stores the thread pool needed for the simulation
     */
    private ExecutorService threadPool;


    /*
    Graphics getters and setters ######################################
     */

    /**
     * Getter for the renderer queue, here we insert the state of the simulation
     * for the GUI to render, this allows to decouple the testbed from the gui
     * @return a concurrent linked queue used to insert the states to render
     */
    private ConcurrentLinkedQueue<GUIQueueElement> getRendererQueue() {
        return rendererQueue;
    }

    /*
    Graphics related instances #######################################
     */

    /**
     * The queue used to communicate with the gui
     */
    private ConcurrentLinkedQueue<GUIQueueElement> rendererQueue = new ConcurrentLinkedQueue<>();

    /**
     * The amount of frames the testbed may go ahead of the renderer before issuing a pause, this
     * prevents the queue from becoming to large
     */
    private final static int MAX_FRAMES_AHEAD = 2700; //1000 ahead gives approx of 100mb data on heap --> gui utilizes about 150mb so seems fair

    /*
    Message strings
     */
    private static final String INVALID_SERVER_SOCKET = "Invalid server socket, provided socket is null reference or the server socket is already initialized";
    private static final String INVALID_THREAD_POOL = "Invalid thread pool, thread pool is already initialized or the provided pool is a null reference";


//    float timeStep, int stepsPerCycle, int stepsPerSubCycle, int maxNbThreads, int tcpPort,
//    PackageService packageService, List<AirportSpec> airports, List<DroneSpec> drones)

    /**
     * An interface used for configuring the server
     */
    public interface TestbedServerConfig{

        /**
         * Getter for the TCP port to be used by the server
         * @return the tcp port to be used by the server
         */
        int getTcpPort();

        /**
         * Getter for the number of steps that will be used for a single simulation cycle
         * one cycle is a full round trip from simulating the next state to receiving the new inputs from the
         * autopilots
         * @return int with the number of steps to be used per cycle
         */
        int getStepsPerCycle();

        /**
         * Getter for the number of steps there are made for a single sub cycle
         * during a sub cycle no checks are performed on the drones to check if they have crashed or
         * delivered a package
         * @return the number of sub steps per cycle (integer)
         */
        int getStepsPerSubCycle();

        /**
         * Getter for the time step to be used by the server while simulating the testbed
         * @return float indicating the simulation step time in seconds
         */
        float getTimeStep();

        /**
         * Getter for the maximum number of threads to be employed by the server
         * @return the max nb of threads
         */
        int getMaxNbThreads();

        /**
         * Getter for the package service to be used by the world that is simulated by the server
         * @return a package service object, will be used by the world
         */
        PackageService getPackageService();

        /**
         * Getter for the drone specifications,
         * these specifications will be used for generating the world
         * @return a list of dronespec objects for configuring the world
         */
        List<DroneSpec> getDroneSpecifications();

        /**
         * Getter for the airport specifications
         * these specifications will be used to generate the airports in the world
         * @return a list of airport specifications to be used while generating the world
         */
        List<AirportSpec> getAirportSpecifications();
    }

}


