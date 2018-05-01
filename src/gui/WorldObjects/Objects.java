package gui.WorldObjects;

import TestbedAutopilotInterface.GUI.AirportGuiState;
import TestbedAutopilotInterface.GUI.CubeGuiState;
import TestbedAutopilotInterface.GUI.DroneGuiState;
import TestbedAutopilotInterface.GUI.GUIQueueElement;
import gui.GraphicsObjects.GraphicsObject;
import gui.Windows.Graphics;

import java.util.*;

public class Objects {
    private static Set<GraphicsObject> objectSet = new HashSet<GraphicsObject>();
    private static Map<String, DroneGuiState> droneGuiStates = new HashMap<String, DroneGuiState>();
    private static Set<CubeGuiState> cubeGuiStates = new HashSet<CubeGuiState>();
    private static Set<AirportGuiState> airportGuiStates = new HashSet<AirportGuiState>();

    private static Map<String, Drone> drones = new HashMap<String, Drone>();
    private static Set<WorldObject> worldObjects = new HashSet<WorldObject>();
    private static Graphics graphics;
    private static Floor floor;
    private static Set<Airport> airports = new HashSet<Airport>();
    private static String mainDroneID = null;

    public static void createAll(GUIQueueElement queueElement) {
        updateStates(queueElement);

        floor = new Floor();
        addWorldObject(floor);

        for ( AirportGuiState airportGuiState : getAirportGuiStates()) {
            Airport airport = new Airport(airportGuiState.getPosition().convertToVector3f(), airportGuiState.getPrimaryRunWay().convertToVector3f());
            airports.add(airport);
            addWorldObject(airport);
        }

        for (String droneID : getDroneGuiStates().keySet()) {
            if (mainDroneID == null)
                mainDroneID = droneID;
            getDrones().put(droneID, new Drone(getDroneGuiStates().get(droneID)));
            addWorldObject(getDrones().get(droneID));
        }
    }


    public static void updateStates(GUIQueueElement queueElement) {
        setDroneGuiStates(queueElement.getDroneStates());
        setAirportGuiStates(queueElement.getAirports());
        setCubeGuiStates(queueElement.getCubePositions());
    }

    public static void update(GUIQueueElement queueElement) {
        updateStates(queueElement);

        for (String droneID : getDroneGuiStates().keySet()) {
            if (getDrones().containsKey(droneID)) {
                getDrones().get(droneID).updateObjects(getDroneGuiStates().get(droneID));
            } else {
                getDrones().put(droneID, new Drone(getDroneGuiStates().get(droneID)));
                addWorldObject(getDrones().get(droneID));
            }
        }
    }

    public static void updateGraphicsObjects() {
        Set<GraphicsObject> newGraphicsObjects = new HashSet<GraphicsObject>();
        for (WorldObject worldObject: getWorldObjects()) {
            newGraphicsObjects.addAll(worldObject.getGraphicsObjectSet());
        }
        setObjectSet(newGraphicsObjects);
    }

    public static void renderAll() {
        updateGraphicsObjects();
        getGraphics().renderWindows(getObjectSet(), getDroneGuiStates(), getMainDroneID());
    }

    /**
     * Method that returns a set of all the objects in the world that belong to a given subclass
     * @param type the class to which the requested objects should belong
     * @author anthonyrathe
     */
    public static <type> Set<type> getObjectSet(Class<? extends WorldObject> type){
        Set<type> objects = new HashSet<type>();
        for (GraphicsObject object : getObjectSet()) {
            if (type.isInstance(object)){
                objects.add((type)object);
            }
        }
        return objects;
    }

    public static Set<GraphicsObject> getObjectSet() {
        return Objects.objectSet;
    }

    public static void addObjectSet(Set<GraphicsObject> graphicsObjects) {
        Objects.objectSet.addAll(graphicsObjects);
    }

    public static void setObjectSet(Set<GraphicsObject> graphicsObjects) {
         Objects.objectSet = graphicsObjects;
    }

    public static void setGraphics(Graphics graphics) {
        Objects.graphics = graphics;
    }

    public static Graphics getGraphics() {
        return Objects.graphics;
    }

    public static Map<String, DroneGuiState> getDroneGuiStates() {
        return Objects.droneGuiStates;
    }

    public static void setDroneGuiStates(Map<String, DroneGuiState> droneGuiStates) {
        Objects.droneGuiStates = droneGuiStates;
    }

    public static Map<String, Drone> getDrones() {
        return drones;
    }

    public static void setDrones(Map<String, Drone> drones) {
        Objects.drones = drones;
    }

    public static Set<WorldObject> getWorldObjects() {
        return worldObjects;
    }

    public static void setWorldObjects(Set<WorldObject> worldObjects) {
        Objects.worldObjects = worldObjects;
    }

    public static void addWorldObjects(Set<WorldObject> worldObjects) {
        Objects.worldObjects.addAll(worldObjects);
    }

    public static void addWorldObject(WorldObject worldObject) {
        Objects.worldObjects.add(worldObject);
    }

    public static String getMainDroneID() {
        return mainDroneID;
    }

    public static Set<CubeGuiState> getCubeGuiStates() {
        return cubeGuiStates;
    }

    public static void setCubeGuiStates(Set<CubeGuiState> cubeGuiStates) {
        Objects.cubeGuiStates = cubeGuiStates;
    }

    public static Set<AirportGuiState> getAirportGuiStates() {
        return airportGuiStates;
    }

    public static void setAirportGuiStates(Set<AirportGuiState> airportGuiStates) {
        Objects.airportGuiStates = airportGuiStates;
    }
}
