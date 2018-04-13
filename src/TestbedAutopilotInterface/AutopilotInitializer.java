package TestbedAutopilotInterface;

import TestbedAutopilotInterface.Overseer.AutopilotOverseer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Created by Martijn on 8/03/2018.
 * A class that initializes all the autopilot connections used for the simulation
 */
public class AutopilotInitializer implements Runnable {
    public AutopilotInitializer(String host, int tcpPort, int nbOfAutopilots, AutopilotOverseer overseer){
        this.port = tcpPort;
        this.host = host;
        this.nbOfAutopilots = nbOfAutopilots;
        this.autopilotThreads = Executors.newFixedThreadPool(nbOfAutopilots);
        this.overseer = overseer;

    }

    @Override
    public void run() {
        initialize();
    }

    /**
     * Initializes all the autopilot connections for a simulation
     */
    public void initialize(){
        System.out.println("initializing autopilots");
        ExecutorService threadPool = this.getAutopilotThreads();
        String host = this.getHost();
        int tcpPort = this.getPort();
        int nbOfThreads = this.getNbOfAutopilots();
        AutopilotOverseer overseer = this.getOverseer();
        List<AutopilotConnection> connections = new ArrayList<>();

        //create the connection objects
        for(int i  = 0; i != nbOfThreads; i++){
            //set the queue to the connection
            AutopilotConnection connection = new AutopilotConnection(host, tcpPort, overseer);
            connections.add(connection);
        }

        //then submit them all to the executor service
        try {
            System.out.println("invoking autopilots");
            List<Future<Void>> futures = threadPool.invokeAll(connections);
            System.out.println("Autopilots invoked");
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        System.out.println("Initialized autopilots");
    }

    /**
     * Getter for the tcp port used for communication with the tesbed
     * @return
     */
    private int getPort() {
        return port;
    }

    /**
     * Getter for the host name of the testbed (localhost)
     * @return
     */
    private String getHost() {
        return host;
    }


    /**
     * Getter for the number of autopilots maintained in the thread pool
     * @return the number of autopilots created
     */
    private int getNbOfAutopilots() {
        return nbOfAutopilots;
    }

    /**
     * Getter for the autopilot thread pool
     * @return the currently used thread pool
     */
    private ExecutorService getAutopilotThreads() {
        return autopilotThreads;
    }

    /**
     * Getter for the overseer for communication between the autopilots
     * @return an autopilot overseer used in coordinating the drones
     */
    private AutopilotOverseer getOverseer() {
        return overseer;
    }

    /**
     * The port the server is located at
     */
    private int port;

    /**
     * The host name of the server
     */
    private String host;

    /**
     * The number of drones there are present in the testbed and thus how much autopilots to generate
     */
    private int nbOfAutopilots;

    /**
     * The thread pool used to keep all the autopilot connections in different threads
     */
    private ExecutorService autopilotThreads;

    /**
     * The overseer to set to the autopilots
     */
    private AutopilotOverseer overseer;
}
