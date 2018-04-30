package gui.Windows;

import TestbedAutopilotInterface.GUI.DroneGuiState;
import math.Vector2f;

import java.awt.*;
import java.awt.Graphics;
import java.awt.event.*;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

public class MiniMap extends Frame {
    static int width = 400;
    static int height = 300;
    static int x = 100;
    static int y = 100;
    static int dx = 3;
    static int dy = 3;
    static int radius = 5;
    static int boundx = 50;
    static int boundy = 50;
    static int boundwidth = 300;
    static int boundheight = 200;


    private String mainDroneID;
    private Set<Vector2f> dronePositions = new HashSet<Vector2f>();

    public MiniMap(String title, int width, int height, int xPos, int yPos, Map<String, DroneGuiState> droneStates, String mainDrone) {
        super(title);
        setLocation(xPos, yPos);
        setSize(width, height);
        setVisible(true);

        this.mainDroneID = mainDrone;
        for (String droneID : droneStates.keySet()) {
            dronePositions.add(new Vector2f(droneStates.get(droneID).getPosition().getxValue(), droneStates.get(droneID).getPosition().getyValue()));
        }

        addWindowListener(new WindowAdapter() {
                              public void windowClosing(WindowEvent e) {
                                  dispose();
                                  System.exit(0);
                              }
                          }
        );
    }

    public void update(Map<String, DroneGuiState> droneStates) {
        dronePositions.clear();
        for (String droneID : droneStates.keySet()) {
            dronePositions.add(new Vector2f(droneStates.get(droneID).getPosition().getxValue(), droneStates.get(droneID).getPosition().getyValue()));
        }
        test();
    }

    public void close() {
    }

    public void test() {
        this.paint(this.getGraphics());
        if (x+dx < boundx+radius || x+dx > boundx+boundwidth-radius)
            dx = -dx;
        if (y+dy < boundy+radius || y+dy > boundy+boundheight-radius)
            dy = -dy;
        x += dx;
        y += dy;
        try {
            TimeUnit.MILLISECONDS.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void paint(Graphics g) {
        g.clearRect(0, 0, width, height);

        g.setColor(Color.red);
        g.fillOval(x-radius, y-radius, radius*2, radius*2);

        g.setColor(Color.blue);
        g.drawRect(boundx, boundy, boundwidth, boundheight);
    }
}
