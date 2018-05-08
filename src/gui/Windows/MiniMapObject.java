package gui.Windows;

import java.awt.*;

public class MiniMapObject {
    private boolean packag;
    private Color color;
    private String name;

    public MiniMapObject(boolean packag, Color color, String name) {
        this.packag = packag;
        this.color = color;
        this.name = name;
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
}
