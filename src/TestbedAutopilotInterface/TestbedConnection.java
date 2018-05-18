package TestbedAutopilotInterface;

import AutopilotInterfaces.*;
import TestbedAutopilotInterface.Overseer.PackageService;
import TestbedAutopilotInterface.Overseer.WorldDelivery;
import internal.Testbed.Drone;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.Set;
import java.util.concurrent.Callable;

/**
 * Created (and fully implemented) by Martijn on 23/02/2018.
 * a class of threads for a testbed server
 * each thread is responsible for the communication with an autopilot from the autopilot module
 */
public class TestbedConnection implements Callable<Void> {


    /**
     * A Thread implementation for a thread in the testbed
     * @param socket the socket for communication with the autopilot
     * @param drone the drone the thread is responsible for instructing
     * @param testBedServer the server associated with the threads
     */
    TestbedConnection(Socket socket, Drone drone, TestbedServer testBedServer) throws IOException {
        this.drone = drone;
        this.socket = socket;
        this.testBedServer = testBedServer;
        // set the streams
        this.inputStream = new DataInputStream(socket.getInputStream());
        this.outputStream = new DataOutputStream(socket.getOutputStream());

    }

    /**
     * The run is only called after the main thread has calculated the next state
     * invocation at other moments may result in incorrect functioning
     * note: upon init the testbed has not yet simulated a single step, it only sends the initial state of the world
     */
    @Override
    public  Void call() {
        ConnectionState connectionState = this.getConnectionState();
        AutopilotOutputs droneInputs;
        switch (connectionState){
            case CONFIGURE:
                this.writeConfig();
                CommunicateWithAutopilot();
                //set the next state
                this.setConnectionState(ConnectionState.WAIT_FOR_DISTRIBUTION);
                break;

            //this state is made because we do not need a simulation on the first time the packages are distributed
            case WAIT_FOR_DISTRIBUTION:
                CommunicateWithAutopilot();

                PackageService packageService = this.getTestBedServer().getPackageService();
                //get the packages that were not yet assigned
                Set<WorldDelivery> notYetAssigned = packageService.getAssignedWorldDeliveries();
                System.out.println("Waiting for distribution of packages");

                while(notYetAssigned.size() == 0){
                    try {
                        wait();
                    } catch (InterruptedException e) {
                        //its all good just check
                        System.out.println("interruption: " + e);
                        notYetAssigned = packageService.getAssignedWorldDeliveries();
                    }
                }
                System.out.println("Packages assigned");

                this.setConnectionState(ConnectionState.READY);
                break;

            default:
                CommunicateWithAutopilot();
                break;

        }
//        //first check if the autopilot is already configured
//        if(connectionState == ConnectionState.CONFIGURE){
//            //if not configure it
//            this.writeConfig();
//            //then set the flag to true
//            this.setConfiguredAutopilot();
//        }

//        //then send the newly generated state to the autopilot
//        this.writeOutputs();
//        //then we wait for the response of the autopilot
//        AutopilotOutputs droneInputs = this.readInputs();
//        //write the acquired information to the drone
//        this.writeInputsToDrone(droneInputs);
//        //then we are finished and ready for the next cycle
////
//
//        if(secondIter){
//            //get the package service
//            PackageService packageService = this.getTestBedServer().getPackageService();
//            //get the packages that were not yet assigned
//            Set<WorldDelivery> notYetAssigned = packageService.getAssignedWorldDeliveries();
//            while(notYetAssigned.size() == 0){
//                try {
//                    System.out.println("Waiting for distribution of packages");
//                    wait();
//                } catch (InterruptedException e) {
//                    //its all good just check
//                    notYetAssigned = packageService.getAssignedWorldDeliveries();
//                }
//            }
//            System.out.println("Packages assigned");
//            secondIter = false;
//        }
//        if(!secondIter&&!){
//            secondIter = true;
//        }
        return null;
    }

    private void CommunicateWithAutopilot() {
        AutopilotOutputs droneInputs;
        this.writeOutputs();
        //then we wait for the response of the autopilot
        droneInputs = this.readInputs();
        //write the acquired information to the drone
        this.writeInputsToDrone(droneInputs);
    }

    /**
     * Writes the configuration of the drone to the autopilot this information is used to steer the control
     * mechanisms
     */
    private void writeConfig() {
        //first retrieve the configuration of the drone
        AutopilotConfig config = this.getDrone().getAutopilotConfig();
        //then send it over to the autopilot
        try {
            AutopilotConfigWriter.write(this.getOutputStream(), config);
        } catch (IOException e) {
            //again something beyond my expertise went wrong
            e.printStackTrace();
        }
    }

    /**
     * Writes the outputs of the testbed to the autopilot, these contain the state information of the drone
     * needed by the autopilot to guide the flight
     */
    private void writeOutputs(){
        // first retrieve the outputs from the drone(the current outputs)
        AutopilotInputs_v2 autopilotInputs = new TestbedOutputs();
        // create a class of outputs to send over the writer (AutopilotInputs object)
        try {
            AutopilotInputs_v2Writer.write(this.getOutputStream(), autopilotInputs);
        } catch (IOException e) {
            //something went wrong, dunno what
            e.printStackTrace();
        }
    }

    /**
     * Reads the input stream for new commands from the autopilot
     */
    private AutopilotOutputs readInputs(){
        //first retrieve the inputs from the input stream
        AutopilotOutputs droneInputs = null;

        try {
            //System.out.println("outputs: ");
            droneInputs = AutopilotOutputsReader.read(this.getInputStream());
            //System.out.println(Drone.extractInputs(droneInputs));
        } catch (IOException e) {
            //and yet again something utterly mysterious happened
            e.printStackTrace();
        }
        return droneInputs;
    }

    /**
     * Writes the given AutopilotOutputs to the drone
     * @param autopilotOutputs the outputs generated by the autopilot
     */
    private void writeInputsToDrone(AutopilotOutputs autopilotOutputs){
        //then send the new inputs to the associated drone
        Drone drone = this.getDrone();
        //at this stage drone inputs cannot be null anymore (no need to check)
        //because the system will have terminated with an error (maybe provide some standard input?)
        drone.setAutopilotOutputs(autopilotOutputs);
    }

    /**
     * Called to terminate the thread, closes all sockets and streams properly
     */
    void terminateConnection(){
        try {
            this.getOutputStream().close();
            this.getInputStream().close();
            this.getSocket().close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }


    /**
     * Getter for the socket of the testbed thread
     * @return the socket used for communication with the autopilot
     */
    private Socket getSocket() {
        return socket;
    }

    /**
     * Getter for the drone associated with the given thread
     * @return the associated drone
     */
    private Drone getDrone() {
        return drone;
    }

    /**
     * Getter for the data input stream, used for receiving the commands of the autopilot
     * @return the data input stream
     */
    private DataInputStream getInputStream() {
        return inputStream;
    }

    /**
     * Getter for the data output stream, used for sending the flight data to the autopilot
     * @return the data output stream
     */
    private DataOutputStream getOutputStream() {
        return outputStream;
    }

    /**
     * Getter for the associated testbed Server
     * @return the associated testbed server
     */
    private TestbedServer getTestBedServer() {
        return testBedServer;
    }


    /**
     * Getter for the configuration flag of the autopilot, if this flag is true no configuration information
     * is sent upon calling run
     * @return true if the autopilot is configured
     */
    private boolean isConfiguredAutopilot() {
        return configuredAutopilot;
    }

    /**
     * Sets the configuration flag to true
     */
    private void setConfiguredAutopilot() {
            this.configuredAutopilot = true;
    }

    /**
     * Getter for the connection state
     * @return the connection state of the drone, may be either config, distribution or ready
     */
    private ConnectionState getConnectionState() {
        return connectionState;
    }

    /**
     * Setter for the connection state, indicates what the connection has to do (eg config or wait until packages are distributed)
     * @param connectionState the state of the connection
     */
    private void setConnectionState(ConnectionState connectionState) {
        this.connectionState = connectionState;
    }

    /*
     * Instance variables
     */


    /**
     * Object that holds the client socket used for termination
     */
    private Socket socket;

    /**
     * Object that holds the associated drone in the world
     */
    private Drone drone;

    /**
     * Object that holds the input data stream, receiving commands from the autopilot
     */
    private DataInputStream inputStream;
    /**
     * Object that holds the output data stream, used for sending the status to the autopilot
     */
    private DataOutputStream outputStream;

    /**
     * Object that stores the associated testbed server (needed to retrieve info)
     */
    private TestbedServer testBedServer;

    /*
    Flags
     */

    /**
     * Flag that shows if the autopilot is already configured, if not the autopilot write step
     * needs to send the configuring information first
     */
    private boolean configuredAutopilot = false;

    private boolean secondIter = false;

    private ConnectionState connectionState = ConnectionState.CONFIGURE;

    private enum ConnectionState {
        CONFIGURE, WAIT_FOR_DISTRIBUTION, READY
    }

    /*
    Private classes used for the implementation of the threads
     */
    /**
     * A class of inputs for an autopilot used for sending information over to the autopilot
     */
    private class TestbedOutputs implements AutopilotInputs_v2{

        /**
         * Here the get image is empty, this is because the autopilot flies without input from its camera
         */
        @Override
        public byte[] getImage() {
            return new byte[0];
        }

        @Override
        public float getX() {
            return TestbedConnection.this.getDrone().getPosition().getxValue();
        }

        @Override
        public float getY() {
            return TestbedConnection.this.getDrone().getPosition().getyValue();
        }

        @Override
        public float getZ() {
            return TestbedConnection.this.getDrone().getPosition().getzValue();
        }

        @Override
        public float getHeading() {
            return TestbedConnection.this.getDrone().getOrientation().getxValue();
        }

        @Override
        public float getPitch() {
            return TestbedConnection.this.getDrone().getOrientation().getyValue();
        }

        @Override
        public float getRoll() {
            return TestbedConnection.this.getDrone().getOrientation().getzValue();
        }

        @Override
        public float getElapsedTime() {
            // first get the server
            TestbedServer server = TestbedConnection.this.getTestBedServer();
            // then extract the simulation time
            return server.getElapsedTime();
        }
    }
}
