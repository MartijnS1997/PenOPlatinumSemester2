package gui.WorldObjects;

import gui.GraphicsObjects.Cube;
import gui.GraphicsObjects.GraphicsObject;
import math.Vector3f;

import java.util.HashSet;
import java.util.Set;

public class Block implements WorldObject {
    private Set<GraphicsObject> graphicsObjectSet = new HashSet<GraphicsObject>();
    private Vector3f position;

    public Block() {
        initGraphicsObjects();
    }

    @Override
    public void initGraphicsObjects() {
        this.addGraphicsObject(new Cube(new Vector3f(), new Vector3f(), false));
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
