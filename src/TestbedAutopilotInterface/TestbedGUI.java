package TestbedAutopilotInterface;

import gui.GraphicsObjects.Cube;
import gui.GraphicsObjects.Tile;
import gui.GraphicsObjects.Wheel;
import gui.GL.Time;
import gui.Windows.Graphics;
import gui.Windows.Settings;
import gui.Windows.Window;
import gui.WorldObjects.Objects;
import math.Vector3f;
import org.lwjgl.glfw.GLFWVidMode;

import java.io.IOException;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentLinkedQueue;

import static org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor;
import static org.lwjgl.glfw.GLFW.glfwGetVideoMode;

/**
 * Created by Martijn on 26/03/2018.
 * A class of tesbed Gui's for visualising the testbed contents
 */
public class TestbedGUI implements Callable<Void>{

    /**
     * Constructor for a testbed GUI, generates all the gui elements from the testbed
     * @param guiQueue the queue to use for communication with the testbed, this is the
     *                 queue where the testbed will place its simulation results
     */
    public TestbedGUI(ConcurrentLinkedQueue<GUIQueueElement> guiQueue) {
        this.guiQueue = guiQueue;
    }

    private void initTestbedGUI() throws IOException {

        // first generate the graphics
        this.generateGraphics();

        // the settings of the world influence the way the windows are rendered so
        // we may only initialize them after the world config is known
        while (prevGUIQueueElement == null) {
            readFromQueue();
        }
        Objects.createAll(prevGUIQueueElement);
        this.initWindows();

        // set the timer for real time sync
        Time.initTime();
    }

    @Override
    public Void call() throws Exception {

        initTestbedGUI();

        while(true) {
            //set the timer for real frame rate
            Time.update();

            Objects.update(getPrevGUIQueueElement());
            Objects.renderAll();
            readFromQueue();

        }
    }

    private void generateGraphics(){
        //create a new graphics object to associate with the server
        this.setGraphics(new Graphics());

        //provide the graphics for generating cubes
        Cube.setGraphics(this.getGraphics());
        Tile.setGraphics(this.getGraphics());
        Wheel.setGraphics(this.getGraphics());
        Objects.setGraphics(this.getGraphics());

        // get monitor size
        GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        setMonitorWidth(vidmode.width());
        setMonitorHeight(vidmode.height());

        //construct the windows
        this.setDroneView(new Window(getMonitorWidth()/2, getMonitorHeight()/2 - 30, 0.0f, 0.05f, "Drone view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setTopDownView(new Window(getMonitorWidth()/2, getMonitorHeight()/3 - 30, 1f, 0.04f, "Top down view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setSideView(new Window(getMonitorWidth()/2, getMonitorHeight()/3 - 30, 1f, 0.52f, "Side view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setChaseView(new Window(getMonitorWidth()/2, getMonitorHeight()/2 - 30, 0f, 1f, "Chase view", new Vector3f(1.0f, 1.0f, 1.0f), true));

        //then add the windows to the graphics engine
        this.getGraphics().addWindow("Drone view", this.getDroneView());
        this.getGraphics().addWindow("Top down view", this.getTopDownView());
        this.getGraphics().addWindow("Side view", this.getSideView());
        this.getGraphics().addWindow("Chase view", this.getChaseView());
    }

    /**
     * Initialize the windows used in the simulation
     */
    private void initWindows(){
        this.getDroneView().initWindow(Settings.DRONE_CAM);
        this.getTopDownView().initWindow(Settings.DRONE_TOP_DOWN_CAM);
        this.getChaseView().initWindow(Settings.DRONE_CHASE_CAM);
        this.getSideView().initWindow(Settings.DRONE_SIDE_CAM);

        this.getGraphics().makeTextWindow("Stats", getMonitorWidth()/2, getMonitorHeight()/3, getMonitorWidth()/2, getMonitorHeight()*2/3, Objects.getDrones().get(Objects.getMainDroneID()));
    }

    /**
     * Reads an element from the queue. If the queue is empty returns the previously read element
     * as a placeholder until the next element will be simulated
     * @return a GUIQueue element containing the testbed results for rendering
     */
    private GUIQueueElement readFromQueue(){
        ConcurrentLinkedQueue<GUIQueueElement> queue = this.getGuiQueue();
        //peek in the queue, check if it is empty or not
        if(queue.isEmpty()){
            //if empty, return the previous one
            return this.getPrevGUIQueueElement();
        }
        //if not poll the head from the fifo queue
        GUIQueueElement queueHead = queue.poll();
        //set the previous one to the current
        this.setPrevGUIQueueElement(queueHead);
        //return the result
        return queueHead;
    }

    /**
     * Setter for the previous queue element
     * Saved to re-render while the testbed is busy simulating
     * @param prevGUIQueueElement the previous gui element
     */
    private void setPrevGUIQueueElement(GUIQueueElement prevGUIQueueElement) {
        this.prevGUIQueueElement = prevGUIQueueElement;
    }

    /**
     * getter for the previously polled Queue element
     * @return the previous queue element that was read from the queue
     */
    private GUIQueueElement getPrevGUIQueueElement() {
        return prevGUIQueueElement;
    }

    /**
     * Getter for the queue where all the testbed results will be posted
     * @return a concurrent linked queue to read the data retrieved by the testbed
     */
    private ConcurrentLinkedQueue<GUIQueueElement> getGuiQueue() {
        return guiQueue;
    }

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

    /**
     * Getters and setters for monitor width and height.
     */
    public void setMonitorWidth(int monitorWidth) { this.monitorWidth = monitorWidth; }
    public void setMonitorHeight(int monitorHeight) { this.monitorHeight = monitorHeight; }
    public int getMonitorWidth() { return monitorWidth; }
    public int getMonitorHeight() { return monitorHeight; }

    /**
     * The previously received Queue element
     */
    private GUIQueueElement prevGUIQueueElement;

    /**
     * The queue the Gui receives the queue elements from to render on screen
     */
    private ConcurrentLinkedQueue<GUIQueueElement> guiQueue;

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

    /**
     * Integers to remember the width and height of the monitor.
     */
    private int monitorWidth;
    private int monitorHeight;

    /*
    Message strings
     */
    private static final String INVALID_GRAPHICS = "Invalid graphics, graphics engine is already initialized or the provided engine is a null reference";
}
