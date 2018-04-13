package TestbedAutopilotInterface;

import AutopilotInterfaces.*;
import TestbedAutopilotInterface.Overseer.AutopilotOverseer;
import internal.Autopilot.AutoPilot;
import internal.Exceptions.SimulationEndedException;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.concurrent.Callable;

/**
 * Created by Martijn on 8/03/2018.
 * A class of AutopilotInterfaces connections
 * Each connection is linked to a certain autopilot and will send and receive data for it
 */
public class AutopilotConnection implements Callable<Void> {

    /**
     * Constructor for an autopilot connection
     * @param host the host address of the server to make a connection with, standard "localhost"
     * @param TCPPort the TCP port used for communication
     */
    public AutopilotConnection(String host, int TCPPort, AutopilotOverseer overseer) {
        this.host = host;
        this.TCPPort = TCPPort;
        this.setAutopilot(new AutoPilot(overseer));
    }

    /**
     * Run maintains the main loop for one connection.
     * sequential part:
     * initialize the connection with the server
     * parts of the loop:
     * 1. receive data, check if the autopilot is already configured (if not wait for config)
     * 2. once data is received generate the response/controls with the autopilot
     * 3. send the data back to the server and goto 1.
     */
    @Override
    public Void call() {
        //initialize the connection
        initServerConnection();
        System.out.println("connection to server acquired");
        //start the main loop
        try {
            clientMainLoop();
        }catch(Exception e){
            e.printStackTrace();
        }

        return null;
    }

    /**
     * The main loop
     */
    private void clientMainLoop(){

        Autopilot_v2 autopilot = this.getAutopilot();
        boolean simulating = true;
        while(simulating){
            try {
                AutopilotOutputs outputs = null;
                //first check if the autopilot is already configured
                if (!this.isAutopilotConfigured()) {
                    AutopilotConfig config = this.getAutopilotConfig();
                    AutopilotInputs_v2 inputs = this.getInputs();
                    outputs = autopilot.simulationStarted(config, inputs);
                    this.setAutopilotConfigured();
                } else {
                    // if already configured we only need to get the new inputs
                    AutopilotInputs_v2 inputs = this.getInputs();
                    // and generate the outputs
                    outputs = autopilot.timePassed(inputs);
                }

                //in either case we need to send the results back to the server
                writeOutputs(outputs);
                //we may loop again

            }catch(SimulationEndedException e){
                //simulation ended exception is thrown by the termination method
                //we're finished break the loop:
                simulating = false;
            }
        }

    }

    /**
     * Listens for incoming inputs from the server (containing autopilot input v2 objects
     * @return the results generated by the server and received by the client
     */
    private AutopilotInputs_v2 getInputs(){
        //get the data stream
        DataInputStream inputStream = this.getInputStream();
        //get the data from the input stream
        AutopilotInputs_v2 inputs = null;
        try {
//            System.out.println("Reading inputs");
             inputs = AutopilotInputs_v2Reader.read(inputStream);
        } catch(java.io.EOFException e){
            //the connection has closed terminate:
            terminateConnection();

        } catch (IOException e) {
            //something went wrong, just print the trace (i have no idea what could have gone wrong
            e.printStackTrace();
        }
        //return the newly received data
        return inputs;
    }

    /**
     * Listens for incoming configurations, if already configured throw error
     * @return the received configuration
     */
    private AutopilotConfig getAutopilotConfig(){
        // first check if already configured
        if(this.isAutopilotConfigured()){
            //if so, throw exception
            throw new IllegalStateException(AUTOPILOT_ALREADY_CONFIGURED);
        }

        //get the input stream:
        DataInputStream inputStream = this.getInputStream();
        AutopilotConfig config = null;
        //if not, listen for incoming config
        try {
            //read the incoming config
            config = AutopilotConfigReader.read(inputStream);
        } catch(java.io.EOFException e){
            //connection lost, close down the connection
            terminateConnection();

        } catch (IOException e) {
            e.printStackTrace();
        }

        //return the read config
        return config;
    }

    /**
     * Writes the outputs generated by the autopilot to the server
     * @param outputs the outputs generated by the autopilot
     */
    private void writeOutputs(AutopilotOutputs outputs){
        //get the data output stream
        DataOutputStream outputStream = this.getOutputStream();
        // then send the outputs over the line
        try {
            AutopilotOutputsWriter.write(outputStream, outputs);
        } catch(java.io.EOFException e){
            //connection lost, close down the connection
            terminateConnection();
        } catch (IOException e) {
            //no idea what has happened, just print the stack trace
            e.printStackTrace();
        }
    }

    /*
    Initializer & termination stuff
     */

    /**
     * Initializes a connection with the server based on the current TCP port and host address
     * sets the socket, input stream and output stream
     */
    private void initServerConnection(){
        //first get the necessary data to establish the connection
        String host = this.getHost();
        int TCPPort = this.getTCPPort();

        //ask for a connection
        try {

            Socket socket = new Socket(host, TCPPort);
            // create the streams
            DataInputStream inputStream = new DataInputStream(socket.getInputStream());
            DataOutputStream outputStream = new DataOutputStream(socket.getOutputStream());

            //save the created socket and streams locally
            this.setSocket(socket);
            this.setInputStream(inputStream);
            this.setOutputStream(outputStream);

        } catch (IOException e) {
            //something went wrong during establishing the connection print stack
            e.printStackTrace();
        }

        //after all this the connection is established and ready to roll

    }

    /**
     * Terminates the connection with the server by closing the socket and streams
     * throws a simulation ended exception to notify the main loop that we've finished
     */
    private void terminateConnection(){
        DataInputStream inputStream = this.getInputStream();
        DataOutputStream outputStream = this.getOutputStream();
        Socket socket = this.getSocket();

        try {
            inputStream.close();
            outputStream.close();
            socket.close();
        } catch (IOException e) {
            //we're closing down, errors can happen... we just don't care, the job is done
            e.printStackTrace();
        }

        throw new SimulationEndedException();

    }

    /**
     * Getter for the autopilot wherefore the connection is responsible for
     * @return the autopilot that is connected to the testbed via this autopilot connection
     */
    private Autopilot_v2 getAutopilot() {
        return autopilot;
    }

    /**
     * Setter for the autopilot associated with this connection
     * @param autopilot the autopilot wherefore this connection is responsible
     */
    private void setAutopilot(Autopilot_v2 autopilot) {
        this.autopilot = autopilot;
    }

    /**
     * The socket used in the connection with the testbed
     * @return the socket of the client used by the connection for communication with the testbed
     */
    private Socket getSocket() {
        return socket;
    }

    /**
     * Setter for the socket that is responsible for server communication
     * @param socket the TCP socket used for communication
     */
    private void setSocket(Socket socket) {
        this.socket = socket;
    }

    /**
     * Getter for the data stream where we receive the input from the testbed (containing config and instructions)
     * @return the data input stream used for server communication
     */
    private DataInputStream getInputStream() {
        return inputStream;
    }

    /**
     * Setter for the input stream used for server communication
     * @param inputStream
     */
    private void setInputStream(DataInputStream inputStream) {
        this.inputStream = inputStream;
    }

    /**
     * Getter for the data output stream for server communication, the stream is used to send autopilot outputs
     * to the testbed
     * @return the output stream used for communication with the testbed server
     */
    private DataOutputStream getOutputStream() {
        return outputStream;
    }

    /**
     * Setter for the output stream
     * @param outputStream the output data stream
     */
    private void setOutputStream(DataOutputStream outputStream) {
        this.outputStream = outputStream;
    }

    /**
     * Getter for the current host address (used in the config)
     * @return the current host address
     * note: here we'll only use localhost
     */
    private String getHost() {
        return host;
    }

    /**
     * Getter for the TCP port used in the communication with the server
     * @return the TCP port used for communication
     */
    private int getTCPPort() {
        return TCPPort;
    }


    /*
     Instances
     */

    /**
     * The associated autopilot, this communication thread is responsible for the communication and
     * activation of this particular autopilot
     */
    private Autopilot_v2 autopilot;

    /**
     * Stores the configuration of the autopilot associated with the connection
     */
    private AutopilotConfig config;

    /**
     * The TCP/IP socket used for client-server communication
     */
    private Socket socket;

    /**
     * The data input stream where the connection receives the simulation data from the testbed
     */
    private DataInputStream inputStream;

    /**
     * The data output stream where the connections sends the outputs generated by the autopilot to
     */
    private DataOutputStream outputStream;

    /**
     * The host used for the creation of the socket
     */
    private String host = "localhost";

    /**
     * The TCP port used in communication with the testbed server
     */
    private int TCPPort;


    /*
    Flags: instances, getters and setters
     */

    /**
     * Checks if the autopilot is already configured, if so, return true
     * @return true if the autopilot is already configured
     */
    public boolean isAutopilotConfigured() {
        return autopilotConfigured;
    }

    /**
     * Sets the is configured flag to true (only one time use)
     */
    private void setAutopilotConfigured() {
        autopilotConfigured =true;
    }

    /**
     * Flag to indicate if the autopilot is already configured or not
     */
    private boolean autopilotConfigured = false;

    /*
    Error messages
     */
    public static final String AUTOPILOT_ALREADY_CONFIGURED = "AutopilotInterfaces is already configured";
}
