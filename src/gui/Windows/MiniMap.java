package gui.Windows;

import TestbedAutopilotInterface.GUI.AirportGuiState;
import TestbedAutopilotInterface.GUI.DroneGuiState;
import gui.GraphicsObjects.Tile;
import internal.Helper.Vector;
import math.Vector2f;
import math.Vector4f;

import java.awt.*;
import java.awt.Graphics;
import java.awt.event.*;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

public class MiniMap extends Frame {
    int width;
    int height;
    static int radius = 5;
    Vector4f map;
    Vector4f realSurf;

    private String mainDroneID;
    private Map<Vector2f, Color> dronePositions = new HashMap<Vector2f, Color>();
    private Map<Vector2f, Color> airportPositions = new HashMap<Vector2f, Color>();

    public MiniMap(String title, int width, int height, int xPos, int yPos, Map<String, DroneGuiState> droneStates, String mainDrone, Set<AirportGuiState> airportGuiStates) {
        super(title);
        setLocation(xPos, yPos);
        this.width = width;
        this.height = height;
        map = new Vector4f(9, 38, this.width-19, this.height-48);
//        map = new Vector4f(50, 50, 1000, 1000);
        setSize(width, height);
        setVisible(true);

        this.mainDroneID = mainDrone;
        setDronePositions(droneStates);
        setAirportPositions(airportGuiStates);
        calcRealSurf();

        addWindowListener(new WindowAdapter() {
                              public void windowClosing(WindowEvent e) {
                                  dispose();
                                  System.exit(0);
                              }
                          }
        );
    }

    public void setDronePositions(Map<String, DroneGuiState> droneStates) {
        for (String droneID : droneStates.keySet()) {
            Vector2f position = new Vector2f(droneStates.get(droneID).getPosition().getxValue(), droneStates.get(droneID).getPosition().getzValue());
            if (!dronePositions.containsKey(position)) {
                Color color = Color.black;
                if (droneID == mainDroneID)
                    color = Color.blue;
                dronePositions.put(position, color);
            }
        }
    }

    public void setAirportPositions(Set<AirportGuiState> airportGuiStates) {
        for (AirportGuiState airportGuiState: airportGuiStates) {
            Vector2f position = new Vector2f(airportGuiState.getPosition().getxValue(), airportGuiState.getPosition().getzValue());
            Color color = Color.red;
            airportPositions.put(position, color);
        }
    }

    public void calcRealSurf() {
        this.realSurf = new Vector4f();
        for (Vector2f position: airportPositions.keySet()) {
            if (position.x < realSurf.x)
                realSurf.x = position.x;
            if (position.y < realSurf.y)
                realSurf.y = position.y;
            if (position.x > realSurf.z)
                realSurf.z = position.x;
            if (position.y > realSurf.w)
                realSurf.w = position.y;
        }
        float value = Math.max(Math.max(Math.abs(realSurf.x), Math.abs(realSurf.y)), Math.max(Math.abs(realSurf.z), Math.abs(realSurf.w)));
        realSurf.x = -value;
        realSurf.y = -value;
        realSurf.z = value;
        realSurf.w = value;

        realSurf.z = realSurf.z-realSurf.x;
        realSurf.w = realSurf.w-realSurf.y;

        realSurf = realSurf.scale(2f);
    }

    public void update(Map<String, DroneGuiState> droneStates, String mainDroneID) {
        this.mainDroneID = mainDroneID;
        dronePositions.clear();
        setDronePositions(droneStates);
        paint(this.getGraphics());
    }

    public void paint(Graphics g) {
        g.clearRect((int) map.x-radius, (int) map.y-radius, (int) map.z+radius, (int) map.w+radius);
        g.setColor(Color.green);
        g.fillRect((int) map.x, (int) map.y, (int) map.z, (int) map.w);

        for (Vector2f position: airportPositions.keySet()) {
            g.setColor(airportPositions.get(position));
            g.fillRect((int) (((position.x-realSurf.x)/realSurf.z)*map.z+map.x) - radius, (int) (((position.y-realSurf.y)/realSurf.w)*map.w+map.y) - radius, radius * 2, radius * 2);
        }
        for (Vector2f position: dronePositions.keySet()) {
            g.setColor(dronePositions.get(position));
            g.fillOval((int) (((position.x-realSurf.x)/realSurf.z)*map.z+map.x) - radius, (int) (((position.y-realSurf.y)/realSurf.w)*map.w+map.y) - radius, radius * 2, radius * 2);
        }


    }

//    public void createFloor(int n, float nx, float nz) {
//
//        for (int i = 0; i < n*n; i++) {
//            Vector delta = new Vector(nx*(i%n), 0, nz*(i/n));
//            Vector position = delta.vectorSum(getPosition());
//            Vector color = new Vector((60.0f+(i%2)*60), 1, 0.6f);
//            Tile tile = new Tile(position.convertToVector3f(), color.convertToVector3f());
//            tile.setSize(new Vector(nx, 1, nz));
//            this.setAssociatedGraphicsObject(tile);
//        }
//    }

    public void close() {
    }
}
