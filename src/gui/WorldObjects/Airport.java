package gui.WorldObjects;

import gui.GraphicsObjects.Cube;
import gui.GraphicsObjects.GraphicsObject;
import gui.GraphicsObjects.Tile;
import math.Vector3f;

import java.util.HashSet;
import java.util.Set;

public class Airport implements WorldObject {
    private Set<GraphicsObject> graphicsObjectSet = new HashSet<GraphicsObject>();
    private Vector3f position;
    private Vector3f orientation = new Vector3f();
    float width;
    float length;

    public Airport() {
        setPosition(new Vector3f(0,0.2f,0));
        width = 15;
        length = 280;
        initGraphicsObjects();
    }

    public Airport(Vector3f position) {
        setPosition(new Vector3f(0,0.2f,0).add(position));
        width = 15;
        length = 280;
        initGraphicsObjects();
    }

    public Airport(Vector3f position, Vector3f orientationVector) {
        setPosition(new Vector3f(0, 0.1f, 0).add(position));
        setOrientation(new Vector3f(Vector3f.getHeadingAngle(orientationVector),0,0));
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

        Vector3f position = getPosition().add(new Vector3f((float) (-(L+W)/2*(float)Math.pow(-1, number)*Math.sin(getOrientation().x)), 0, (float) (-(L+W)/2*(float)Math.pow(-1, number)*Math.cos(getOrientation().x))));
        Vector3f color = new Vector3f(0, 0, 0.2f + 0.1f*number);
//        Tile tile = new Tile(position, color, orientation);
        Cube tile = new Cube(position, color, false);
        tile.update(position, orientation);
        tile.setSize(new Vector3f(2*W, 1, L));
        addGraphicsObject(tile);
    }

    public void createGate(float W, float number) {
        Vector3f position = getPosition().add(new Vector3f((float) (-W/2*(float)Math.pow(-1, number)*Math.cos(-getOrientation().x)), 1, (float) (-W/2*(float)Math.pow(-1, number)*Math.sin(-getOrientation().x))));
        Vector3f color = new Vector3f(0, 0, 0.45f + number/2);
//        Tile tile = new Tile(position, color, orientation);
        Cube tile = new Cube(position, color, false);
        tile.update(position, orientation);
        tile.setSize(W);
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

    public Vector3f getOrientation() {
        return this.orientation;
    }

    public void setOrientation(Vector3f orientation) {
        this.orientation = orientation;
    }
}
