package gui.WorldObjects;

import TestbedAutopilotInterface.AirportGuiState;
import TestbedAutopilotInterface.CubeGuiState;
import TestbedAutopilotInterface.DroneGuiState;
import TestbedAutopilotInterface.GUIQueueElement;
import gui.GraphicsObjects.GraphicsObject;
import gui.Windows.Graphics;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class Objects {
    private static Set<GraphicsObject> objectSet = new HashSet<>();
    private static Map<String, DroneGuiState> droneGuiStates;
    private static Set<CubeGuiState> cubeGuiStates;
    private static Set<AirportGuiState> airportGuiStates;

    private static Map<String, Drone> drones;
    private static Set<WorldObject> worldObjects;
    private static Graphics graphics;
    private static Floor floor;
    private static final String mainDroneID = "0";

    public static void createAll(GUIQueueElement queueElement) {
        updateStates(queueElement);

        floor = new Floor();

        for (String droneID : getDroneGuiStates().keySet()) {
            getDrones().put(droneID, new Drone(getDroneGuiStates().get(droneID)));
        }

        worldObjects.addAll(getDrones().values());
    }

    public static void updateStates(GUIQueueElement queueElement) {
        setDroneGuiStates(queueElement.getDroneStates());
        setAirportGuiStates(queueElement.getAirport());
        setCubeGuiStates(queueElement.getCubePositions());
    }

    public static void update(GUIQueueElement queueElement) {
        updateStates(queueElement);

        for (String droneID : getDroneGuiStates().keySet()) {
            getDrones().get(droneID).updateObjects(getDroneGuiStates().get(droneID));
        }
    }

    public static void renderAll() {
        getGraphics().renderWindows(getObjectSet(), getDrones().get(getMainDroneID()));
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
