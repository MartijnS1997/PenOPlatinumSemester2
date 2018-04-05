package gui.WorldObjects;

import gui.GraphicsObjects.GraphicsObject;
import gui.GraphicsObjects.Tile;
import internal.Helper.Vector;
import math.Vector3f;

import java.util.HashSet;
import java.util.Set;

public class Airport implements WorldObject {
    private Set<GraphicsObject> graphicsObjectSet = new HashSet<GraphicsObject>();
    private Vector3f position;
    float width;
    float length;

    public Airport() {
        setPosition(new Vector3f(0,0.2f,0));
        width = 15;
        length = 280;
        initGraphicsObjects();
    }

    @Override
    public void initGraphicsObjects() {
        createLandStrip(width, length, 0);
        createLandStrip(width, length, 1);
        createGate(width, 0);
        createGate(width, 1);
    }

    public void createLandStrip(float W, float L, float number) {

        Vector3f position = getPosition().add(new Vector3f(0, 0, -(L+W)/2*(float)Math.pow(-1, number)));
        Vector3f color = new Vector3f(0, 0, 0.25f*number);
        Tile tile = new Tile(position, color);
        tile.setSize(new Vector3f(2*W, 1, L));
        addGraphicsObject(tile);
    }

    public void createGate(float W, float number) {
        Vector3f position = getPosition().add(new Vector3f(-W/2*(float)Math.pow(-1, number), 0, 0));
        Vector3f color = new Vector3f(0, 0, 0.45f + number/2);
        Tile tile = new Tile(position, color);
        tile.setSize(new Vector3f(W, 1, W));
        addGraphicsObject(tile);
    }

    @Override
    public void addGraphicsObject(GraphicsObject graphicsObject) {
        this.graphicsObjectSet.add(graphicsObject);
    }

    @Override
    public Set<GraphicsObject> getGraphicsObjectSet() {
        return this.graphicsObjectSet;
    }

    @Override
    public Vector3f getPosition() {
        return this.position;
    }

    @Override
    public void setPosition(Vector3f position) {
        this.position = position;
    }
}
