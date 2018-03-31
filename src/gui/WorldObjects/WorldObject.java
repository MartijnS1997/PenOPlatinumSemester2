package gui.WorldObjects;

import gui.GraphicsObjects.GraphicsObject;
import math.Vector3f;

import java.util.Set;

public interface WorldObject {
    void initGraphicsObjects();

    void addGraphicsObject(GraphicsObject graphicsObject);

    public Set<GraphicsObject> getGraphicsObjectSet();

    public Vector3f getPosition();

    public void setPosition(Vector3f position);
}
