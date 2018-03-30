package gui.WorldObjects;

import TestbedAutopilotInterface.DroneGuiState;
import gui.GraphicsObjects.Cube;
import gui.GraphicsObjects.GraphicsObject;
import gui.GraphicsObjects.Polygon;
import gui.GraphicsObjects.Wheel;
import math.Vector3f;

import java.util.Set;

public class Drone implements WorldObject{

    private Vector3f position;
    private Vector3f velocity;
    private Vector3f orientation;
    private float scalingFactor = 1;
    private Set<GraphicsObject> graphicsObjectSet;

    private float distanceTraveled = 0;

    public Drone(DroneGuiState droneGuiState) {
        setState(droneGuiState);
        initGraphicsObjects();
    }

    public void setState(DroneGuiState droneGuiState) {
        setOrientation(droneGuiState.getOrientation().convertToVector3f());
        setPosition(droneGuiState.getPosition().convertToVector3f());
        setVelocity(droneGuiState.getVelocity().convertToVector3f());
    }

    public void updateObjects(DroneGuiState droneGuiState) {
        Vector3f prevPosition = getPosition();
        setState(droneGuiState);
        setDistanceTraveled(getDistanceTraveled() + getPosition().subtract(prevPosition).length());
        for (GraphicsObject graphicsObject: getGraphicsObjectSet()) {
            graphicsObject.update(getPosition(), getOrientation());
        }
    }

    public void initGraphicsObjects() {
        Cube wings = new Cube(getPosition(), new Vector3f(240f, 1f, 1f), false);
        wings.setSize(new Vector3f(5f, 0.1f, 0.7f).scale(scalingFactor));
        this.addGraphicsObject(wings);

        Vector3f position2 = new Vector3f(0f, 0f, 1.75f).scale(scalingFactor);
        Cube middleFrame = new Cube(position2, new Vector3f(240f, 1f, 1f), wings, false);
        middleFrame.setSize(new Vector3f(0.6f, 0.6f, 2.5f).scale(scalingFactor));
        this.addGraphicsObject(middleFrame);

        Vector3f position3 = new Vector3f(0f, 0.6f, 4.5f).scale(scalingFactor);
        Cube verticalStabilizer = new Cube(position3, new Vector3f(240f, 1f, 1f), wings, false);
        verticalStabilizer.setSize(new Vector3f(0.2f, 1.2f, 0.5f).scale(scalingFactor));
        this.addGraphicsObject(verticalStabilizer);

        Vector3f position4 = new Vector3f(0f, 1f, 4.5f).scale(scalingFactor);
        Cube horizontalStabilizer = new Cube(position4, new Vector3f(240f, 1f, 1f), wings, false);
        horizontalStabilizer.setSize(new Vector3f(1.7f, 0.1f, 0.5f).scale(scalingFactor));
        this.addGraphicsObject(horizontalStabilizer);

        Vector3f position5 = new Vector3f(0f, 0f, 3.6f).scale(scalingFactor);
        Cube longFrame = new Cube(position5, new Vector3f(240f, 1f, 1f), wings, false);
        longFrame.setSize(new Vector3f(0.3f, 0.3f, 1.5f).scale(scalingFactor));
        this.addGraphicsObject(longFrame);

        Vector3f position6 = new Vector3f(0f, 0.4f, -0.5f).scale(scalingFactor);
        Cube shortFrame = new Cube(position6, new Vector3f(240f, 1f, 1f), wings, false);
        shortFrame.setSize(new Vector3f(1f, 1f, 2f).scale(scalingFactor));
        this.addGraphicsObject(shortFrame);

        Vector3f position7 = new Vector3f(0f, 0f, -1.5f).scale(scalingFactor);
        Cube front = new Cube(position7, new Vector3f(240f, 1f, 1f), wings, false);
        front.setSize(new Vector3f(0.8f, 0.5f, 1f).scale(scalingFactor));
        this.addGraphicsObject(front);


        Vector3f positioncp1 = new Vector3f(0f, 0.8f, -0.5f).scale(scalingFactor);
        Cube cockpit1 = new Cube(positioncp1, new Vector3f(30f, 1f, 1f), wings, false);
        cockpit1.setSize(new Vector3f(0.7f, 0.8f, 1f).scale(scalingFactor));
        this.addGraphicsObject(cockpit1);

//			Vector3f positioncp2 = new Vector3f(0f, 0.8f, -0.5f).scale(scalingFactor);
//			Wheel cockpitcp2 = new Wheel(positioncp1, new Vector3f(30f, 1f, 1f), wings, 25);
//			cockpit1.setSize(new Vector3f(0.7f, 0.8f, 1f).scale(scalingFactor));
//			this.addGraphicsObject(cockpit1);

        Vector3f rearwheel1position = new Vector3f(0.5f, -0.5f, 0f).scale(scalingFactor);
        Wheel rearwheel1 = new Wheel(rearwheel1position, new Vector3f(0f, 1f, 1f), wings, 25);
        rearwheel1.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
        this.addGraphicsObject(rearwheel1);

        Vector3f rearwheel2position = new Vector3f(-0.5f, -0.5f, 0f).scale(scalingFactor);
        Wheel rearwheel2 = new Wheel(rearwheel2position, new Vector3f(0f, 1f, 1f), wings, 25);
        rearwheel2.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
        this.addGraphicsObject(rearwheel2);

        Vector3f frontwheelposition = new Vector3f(0f, -0.5f, -1.5f).scale(scalingFactor);
        Wheel frontwheel = new Wheel(frontwheelposition, new Vector3f(0f, 1f, 1f), wings, 25);
        frontwheel.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
        this.addGraphicsObject(frontwheel);
    }

    public void addGraphicsObject(GraphicsObject graphicsObject) {
        this.graphicsObjectSet.add(graphicsObject);
    }

    public Set<GraphicsObject> getGraphicsObjectSet() {
        return this.graphicsObjectSet;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    public Vector3f getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector3f velocity) {
        this.velocity = velocity;
    }

    public Vector3f getOrientation() {
        return orientation;
    }

    public void setOrientation(Vector3f orientation) {
        this.orientation = orientation;
    }

    public float getDistanceTraveled() {
        return distanceTraveled;
    }

    public void setDistanceTraveled(float distanceTraveled) {
        this.distanceTraveled = distanceTraveled;
    }
}
