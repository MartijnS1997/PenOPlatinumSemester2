package gui.WorldObjects;

import gui.GraphicsObjects.GraphicsObject;
import gui.GraphicsObjects.Tile;
import math.Vector3f;

import java.util.HashSet;
import java.util.Set;

public class Floor implements WorldObject {
    private Vector3f position;
    private int n;
    private float nx;
    private float nz;
    private Set<GraphicsObject> graphicsObjectSet = new HashSet<GraphicsObject>();

    public Floor() {
        n = 51; // n moet oneven zijn
        nx = 20f;
        nz = 20f;
        this.position = new Vector3f(-n*nx/2, 0, -n*nz/2);
        initGraphicsObjects();
    }

    @Override
    public void initGraphicsObjects() {
        Tile tile1 = new Tile(new Vector3f(0, -5f, 0), new Vector3f(90, 1, 0.6f), false);
        tile1.setSize(new Vector3f(10000, 1, 10000));
        this.addGraphicsObject(tile1);
        for (int i = 0; i < n*n; i++) {
            Vector3f delta = new Vector3f(nx*(i%n), 0, nz*(i/n));
            Vector3f position = delta.add(getPosition());
            Vector3f color = new Vector3f((60.0f+(i%2)*60), 1, 0.6f);
            Tile tile = new Tile(position, color, true);
            tile.setSize(new Vector3f(nx, 1, nz));
            this.addGraphicsObject(tile);
        }
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
