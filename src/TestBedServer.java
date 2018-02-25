import gui.Cube;
import gui.Graphics;
import gui.Settings;
import gui.Window;
import internal.Testbed.Drone;
import internal.Testbed.World;
import math.Vector3f;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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

public class TestBedServer implements Runnable {


    public TestBedServer(float timeStep, int maxNbConnections, int maxNbThreads) {
        this.timeStep = timeStep;
        this.maxNbOfConnections = maxNbConnections;
        this.maxNbOfThreads = maxNbThreads;
    }

    //Todo initialize the server threads
    //Todo implement the incrementation of the simulation time

    /**
     * Run method, gets called upon thread creation for this class (own main loop)
     */
    @Override
    public void run() {
        //first initialize the testbed server
        this.initTestbedServer();
        //then start simulating the server

    }

    /*
    Methods for the simulation cycle
     */
    //Todo implement the simulation cycle
    private void simulateStep(){

    }

    /*
    Initialization methods for the testbed server
     */

    /**
     * Initializes the testbed (see implementations and comments along the lines for further explanation
     */
    private void initTestbedServer() {
        // initialize the graphics engine
        this.initGraphics();

        // initialize the world that needs to be simulated
        this.initWorld();

        // initialize the windows
        this.initWindows();

        // initialize the server
        this.initServer();
    }


    //Todo implement the graphics initializer (need for adapted graphics)
    //Todo check if the initializer can be placed after the generation of the world (or why not)
    /**
     * Initializer for the graphics of the server
     * note: no drone cam is supported here, may be added in the future (not part of the assignment)
     */
    private void initGraphics(){
        //create a new graphics object to associate with the server
        this.setGraphics(new Graphics());

        //TODO check if cubes are still needed, with the packet delivery no cubes need to be placed to fly trough
        //provide the graphics for generating cubes
        Cube.setGraphics(this.getGraphics());

        //construct the windows
        this.setDroneView(new Window(960, 510, 0.0f, 0.05f, "Drone view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setTopDownView(new Window(960, 510, 1f, 0.05f, "Top down view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setSideView(new Window(960, 510, 1f, 1f, "Side view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setChaseView(new Window(960, 510, 0f, 1f, "Chase view", new Vector3f(1.0f, 1.0f, 1.0f), true));

        //then add the windows to the graphics engine
        this.getGraphics().addWindow("Drone view", this.getDroneView());
        this.getGraphics().addWindow("Top down view", this.getTopDownView());
        this.getGraphics().addWindow("Side view", this.getSideView());
        this.getGraphics().addWindow("Chase view", this.getChaseView());
    }

    /**
     * Initializer for the world that needs to be simulated by the server
     */
    // Todo implement the world initializer, first cleanup the world and drone builder
    // note: the amount of created drones is equal to the number of connections we can make
    private void initWorld(){

    }

    /**
     * Initializer for the windows
     */
    private void initWindows(){
        this.getDroneView().initWindow(this.getWorld(), Settings.DRONE_CAM);
        this.getTopDownView().initWindow(this.getWorld(), Settings.DRONE_TOP_DOWN_CAM);
        this.getChaseView().initWindow(this.getWorld(), Settings.DRONE_CHASE_CAM);
        this.getSideView().initWindow(this.getWorld(), Settings.DRONE_SIDE_CAM);
    }

    /**
     * Initializes the server and creates the objects needed for the connections
     */
    private void initServer(){
        //create the thread pool
        ExecutorService threadPool = Executors.newFixedThreadPool(this.getMaxNbOfThreads());
        //set the thread pool
        this.setThreadPool(threadPool);

        //create the server socket
        ServerSocket serverSocket = null;
        try {
            serverSocket = new ServerSocket(TestBedServer.getTcpPort());
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
    public World getWorld() {
        return world;
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
        this.elapsedTime += this.getTimeStep();
    }

    /**
     * Getter for the in-simulation time step
     * @return the time step
     */
    private float getTimeStep() {
        return timeStep;
    }

    /*
    Physics related instances ########################################
     */

    /**
     * Object that stores the world that will be simulated
     */
    private World world;

    /**
     * Variable that stores the in-simulation duration of the flight
     */
    private float elapsedTime = 0;

    /**
     * Variable that stores the in simulation time between the simulation steps
     */
    private float timeStep;

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
    public static int getTcpPort() {
        return TCP_PORT;
    }

    /**
     * Getter for the number of connections maintained by the server
     * @return the number of connections
     */
    private int getMaxNbOfConnections() {
        return maxNbOfConnections;
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
        return threadPool != null && this.getThreadPool() != null;
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
    private final static int TCP_PORT = 4242;

    /**
     * Variable that stores the number of connections maintained by the server,
     * is equal to the number of drones and autopilots supported by the server
     */
    private int maxNbOfConnections;

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
     * Getter for the graphics engine, used for generating the objects
     * @return the graphics engine
     */
    private Graphics getGraphics() {
        return graphics;
    }

    /**
     * Setter for the graphics engine
     * @param graphics the graphics engine to be set
     */
    private void setGraphics(Graphics graphics) {
        if(!canHaveAsGraphics(graphics))
            throw new IllegalArgumentException(INVALID_GRAPHICS);
        this.graphics = graphics;
    }

    /**
     * Checks if the provided graphics engine can be set as the graphics engine
     * for the testbed server
     * @param graphics the graphics engine to be tested
     * @return true if and only if graphics is not a null reference and the server
     * hasn't already a graphics engine associated with it
     */
    private boolean canHaveAsGraphics(Graphics graphics){
        return graphics != null && this.getGraphics() == null;
    }

    /**
     * Getter for the drone View
     * @return the drone view
     */
    private Window getDroneView() {
        return droneView;
    }

    /**
     * Setter for the drone view
     * @param droneView the drone view
     */
    private void setDroneView(Window droneView) {
        this.droneView = droneView;
    }

    /**
     * Getter for the top down view
     * @return the top down view
     */
    private Window getTopDownView() {
        return topDownView;
    }

    /**
     * Setter for the top down view
     * @param topDownView the top down view
     */
    private void setTopDownView(Window topDownView) {
        this.topDownView = topDownView;
    }

    /**
     * Getter for the chase view
     * @return the chase view
     */
    private Window getChaseView() {
        return chaseView;
    }

    /**
     * Setter for the chase view
     * @param chaseView the chase view
     */
    private void setChaseView(Window chaseView) {
        this.chaseView = chaseView;
    }

    /**
     * Getter for the side view
     * @return the side view
     */
    private Window getSideView() {
        return sideView;
    }

    /**
     * Setter for the side view
     * @param sideView the side view
     */
    private void setSideView(Window sideView) {
        this.sideView = sideView;
    }

    /*
    Graphics related instances #######################################
     */
    /**
     * Object that stores the graphics engine for the testbed
     */
    private Graphics graphics;

    /**
     * The windows used in the simulation, note no window for the drone camera input
     * because this one is not used for the packet service (may be added later)
     */
    private Window droneView;
    private Window topDownView;
    private Window chaseView;
    private Window sideView;

    /*
    Message strings
     */
    private static final String INVALID_SERVER_SOCKET = "Invalid server socket, provided socket is null reference or the server socket is already initialized";
    private static final String INVALID_THREAD_POOL = "Invalid thread pool, thread pool is already initialized or the provided pool is a null reference";
    private static final String INVALID_GRAPHICS = "Invalid graphics, graphics engine is already initialized or the provided engine is a null reference";
}


