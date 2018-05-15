package gui.Windows;

import TestbedAutopilotInterface.GUI.AirportGuiState;
import TestbedAutopilotInterface.GUI.DeliveryGuiState;
import TestbedAutopilotInterface.GUI.DroneGuiState;
import gui.WorldObjects.Objects;
import internal.Helper.HSVconverter;
import math.Vector2f;
import math.Vector3f;
import math.Vector4f;

import java.awt.*;
import java.awt.Graphics;
import java.awt.event.*;
import java.util.*;

public class MiniMap extends Frame {
    int width;
    int height;
    static int radius = 5;
    Vector4f map;
    Vector4f realSurf;
    int flickr = 0;

    private String mainDroneID;
    private Map<Vector2f, MiniMapObject> dronePositions = new HashMap<Vector2f, MiniMapObject>();
    private Map<Vector2f, MiniMapObject> airportPositions = new HashMap<Vector2f, MiniMapObject>();

    public MiniMap(String title, int width, int height, int xPos, int yPos, Map<String, DroneGuiState> droneStates, String mainDrone, Set<AirportGuiState> airportGuiStates, Set<DeliveryGuiState> deliveryGuiStates) {
        super(title);
        setLocation(xPos, yPos);
        this.width = width;
        this.height = height;
        map = new Vector4f(9, 38, this.width-18, this.height-47);
//        map = new Vector4f(50, 50, 1000, 1000);
        setSize(width, height);
        setVisible(true);

        this.mainDroneID = mainDrone;
        setDronePositions(droneStates, deliveryGuiStates);
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

    public void setDronePositions(Map<String, DroneGuiState> droneStates, Set<DeliveryGuiState> deliveryGuiStates) {
        Set<String> packageDrones = new HashSet<String>();
        for (DeliveryGuiState delivery: deliveryGuiStates) {
            if (delivery.isPickedUp() && !delivery.isDelivered()) {
                packageDrones.add(delivery.getDeliveryDrone());
            }
        }

        for (String droneID : droneStates.keySet()) {
            Vector2f position = new Vector2f(droneStates.get(droneID).getPosition().getxValue(), droneStates.get(droneID).getPosition().getzValue());
            if (!dronePositions.containsKey(position)) {
                Vector3f colour = Objects.getDrones().get(droneID).getColor();
                Vector3f col = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
                Color color = new Color(col.x, col.y, col.z);
                dronePositions.put(position, new MiniMapObject(packageDrones.contains(droneID), color, droneID));
            }
        }
    }

    public void setAirportPositions(Set<AirportGuiState> airportGuiStates) {
        for (AirportGuiState airportGuiState: airportGuiStates) {
            Vector2f position = new Vector2f(airportGuiState.getPosition().getxValue(), airportGuiState.getPosition().getzValue());
            Color color = Color.darkGray;
            airportPositions.put(position, new MiniMapObject(color, airportGuiState.getAirportID()));
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
        float value = Math.max(Math.max(Math.abs(realSurf.x), Math.abs(realSurf.y)), Math.max(Math.abs(realSurf.z), Math.abs(realSurf.w)))+2000f;
        realSurf.x = -value;
        realSurf.y = -value;
        realSurf.z = value;
        realSurf.w = value;

        realSurf.z = realSurf.z-realSurf.x;
        realSurf.w = realSurf.w-realSurf.y;

        realSurf = realSurf.scale(1.2f);
    }

    public void update(Map<String, DroneGuiState> droneStates, String mainDroneID, Set<DeliveryGuiState> deliveryGuiStates) {
        flickr ++;
        if (flickr%60 == 0)
            flickr = 0;
        this.mainDroneID = mainDroneID;
        dronePositions.clear();
        setDronePositions(droneStates, deliveryGuiStates);
        setAirportStats(deliveryGuiStates);
        paint(this.getGraphics());
    }

    public void setAirportStats(Set<DeliveryGuiState> deliveryGuiStates) {

        for (Vector2f position: airportPositions.keySet()) {
            int ID = airportPositions.get(position).getID();
            int packagesToDeliver = 0;
            int packagesDelivered = 0;
            for (DeliveryGuiState delivery: deliveryGuiStates) {
                if (delivery.getSourceAirport() == ID && !delivery.isPickedUp()) {
                    packagesToDeliver++;
                }
                if (delivery.getDestinationAirport() == ID && delivery.isDelivered()) {
                    packagesDelivered++;
                }
            }
            airportPositions.get(position).setPackagesdelivery(packagesToDeliver, packagesDelivered);
        }
    }

    public void paint(Graphics g) {
        g.clearRect((int) map.x-radius, (int) map.y-radius, (int) map.z+radius, (int) map.w+radius);
        g.setColor(Color.green);
        g.fillRect((int) map.x, (int) map.y, (int) map.z, (int) map.w);
        int fontSize = 15;
        g.setFont(new Font("TimesRoman", Font.PLAIN, fontSize));
        Graphics2D g2d = (Graphics2D)g;

        for (Vector2f position: airportPositions.keySet()) {
            g.setColor(airportPositions.get(position).getColor());
            Vector2f rectPos = new Vector2f((((position.x-realSurf.x)/realSurf.z)*map.z+map.x) - radius, (((position.y-realSurf.y)/realSurf.w)*map.w+map.y) - radius);
            g.fillRect((int) rectPos.x, (int) rectPos.y, radius * 2, radius * 2);
            g2d.drawString(Integer.toString(airportPositions.get(position).getPackagesToDeliver()), rectPos.x-15, rectPos.y);
            g2d.drawString(Integer.toString(airportPositions.get(position).getPackagesDelivered()), rectPos.x+15, rectPos.y);
        }
        for (Vector2f position: dronePositions.keySet()) {
            if (flickr%5 != 0 || !dronePositions.get(position).hasPackage()) {
                g.setColor(dronePositions.get(position).getColor());
                g.fillOval((int) (((position.x - realSurf.x) / realSurf.z) * map.z + map.x) - radius, (int) (((position.y - realSurf.y) / realSurf.w) * map.w + map.y) - radius, radius * 2, radius * 2);
                g.setColor(Color.black);
                g.drawOval((int) (((position.x - realSurf.x) / realSurf.z) * map.z + map.x) - (radius) - 1, (int) (((position.y - realSurf.y) / realSurf.w) * map.w + map.y) - (radius) - 1, radius * 2 + 1, radius * 2 + 1);
            }
        }

        Vector3f mainPosition = Objects.getDrones().get(mainDroneID).getPosition();
        g.setColor(Color.white);
        g.drawOval((int) (((mainPosition.x-realSurf.x)/realSurf.z)*map.z+map.x) - (int) (radius*1.5)-3, (int) (((mainPosition.z-realSurf.y)/realSurf.w)*map.w+map.y) - (int) (radius*1.5)-3, radius * 3+4, radius * 3+4);
        g.setColor(Color.black);
        g.drawOval((int) (((mainPosition.x-realSurf.x)/realSurf.z)*map.z+map.x) - (int) (radius*1.5)-2, (int) (((mainPosition.z-realSurf.y)/realSurf.w)*map.w+map.y) - (int) (radius*1.5)-2, radius * 3+2, radius * 3+2);
        g.drawOval((int) (((mainPosition.x-realSurf.x)/realSurf.z)*map.z+map.x) - (int) (radius*1.5)-4, (int) (((mainPosition.z-realSurf.y)/realSurf.w)*map.w+map.y) - (int) (radius*1.5)-4, radius * 3+6, radius * 3+6);
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
