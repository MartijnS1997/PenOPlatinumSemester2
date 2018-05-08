package gui.Windows;

import java.awt.*;

public class MiniMapObject {
    private boolean packag;
    private Color color;
    private String name;
    private int ID;

    private int packagesToDeliver;
    private int packagesDelivered;

    public MiniMapObject(boolean packag, Color color, String name) {
        this.packag = packag;
        this.color = color;
        this.name = name;
    }

    public MiniMapObject(Color color, int ID) {
        this.color = color;
        this.ID = ID;
    }

    public void setPackagesdelivery(int packagesToDeliver, int packagesDelivered) {
        this.packagesToDeliver = packagesToDeliver;
        this.packagesDelivered = packagesDelivered;
    }

    public boolean hasPackage() {
        return this.packag;
    }

    public Color getColor() {
        return this.color;
    }

    public String getName() {
        return this.name;
    }

    public int getID() {
        return this.ID;
    }

    public int getPackagesDelivered() {
        return packagesDelivered;
    }

    public int getPackagesToDeliver() {
        return packagesToDeliver;
    }
}

