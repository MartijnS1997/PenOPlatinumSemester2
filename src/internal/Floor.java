package internal;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import gui.GraphicsObject;

public class Floor implements WorldObject {

	private Vector position;
	
	private Set<GraphicsObject> floorTiles = new HashSet<>();

	public Floor(Vector position) {
		this.position = position;
	}
	
	@Override
	public void toNextState(float deltaTime) throws IOException {
		// do nothing
	}

	@Override
	public Vector getPosition() {
		return this.position;
	}

	@Override
	public Set<GraphicsObject> getAssociatedGraphicsObjects() {
		return this.floorTiles;
	}

	public void setAssociatedGraphicsObject(GraphicsObject tile) {
		this.floorTiles.add(tile);
		
	}

}
