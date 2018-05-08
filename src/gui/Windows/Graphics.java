package gui.Windows;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.system.MemoryUtil.NULL;

import TestbedAutopilotInterface.GUI.AirportGuiState;
import TestbedAutopilotInterface.GUI.DroneGuiState;
import gui.GraphicsObjects.GraphicsObject;
import gui.WorldObjects.Drone;
import org.lwjgl.glfw.GLFWErrorCallback;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class Graphics {
	
	public HashMap<String, Window> windows =  new HashMap<String, Window>();
	private boolean terminated = false;
	private TextWindow textWindow = null;
	private MiniMap miniMap = null;
	
	public Graphics() {	
		
		// Setup an error callback. The default implementation
		// will print the error message in System.err.
		GLFWErrorCallback.createPrint(System.err).set();

		// Initialize GLFW. Most GLFW functions will not work before doing this.
		if ( !glfwInit() )
			throw new IllegalStateException("Unable to initialize GLFW");
	
		// Configure GLFW
		glfwDefaultWindowHints(); // optional, the current window hints are already the default
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // the window will stay hidden after creation
		glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); // the window will not be resizable
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_SAMPLES, 4);
	}
	
	public void addWindow(String key, Window window) {
		windows.put(key, window);
	}
	
	public void makeTextWindow(String title, int width, int height, int xPos, int yPos, DroneGuiState droneState) {
		
		this.textWindow = TextWindow.createAndShowWindow(this, title, width, height, xPos, yPos, droneState);
	}

	public void makeMiniMap(String title, int width, int height, int xPos, int yPos, Map<String, DroneGuiState> droneStates, String mainDrone, Set<AirportGuiState> airportGuiStates) {

		this.miniMap = new MiniMap(title, width, height, xPos, yPos, droneStates, mainDrone, airportGuiStates);
	}
	
	public void makeButtonWindow() {
		ButtonWindow.createAndShowWindow(this);
	}
	
	public void renderWindows(Set<GraphicsObject> renderObjects, Map<String, DroneGuiState> droneStates, String mainDrone) {
		// Poll for window events. The key callback above will only be
		// invoked during this call.
		glfwPollEvents();
		
		if (this.textWindow != null)
			this.textWindow.update(droneStates.get(mainDrone));
		if (this.miniMap != null)
			this.miniMap.update(droneStates, mainDrone);
		
		for (String key: windows.keySet()) {
			Window window = windows.get(key);
			
			// Make the OpenGL context current
			glfwMakeContextCurrent(window.getHandler());
			window.render(renderObjects, droneStates.get(mainDrone));
			if (window.isTerminated()) {
				windows.remove(key);
				break;
			}
			glfwMakeContextCurrent(NULL);
		}
		if (windows.isEmpty())
			terminate();	
	}
	
	public void renderWindow(String key, Set<GraphicsObject> renderObjects, DroneGuiState droneState) {
		// Poll for window events. The key callback above will only be
		// invoked during this call.
		glfwPollEvents();

		Window window = windows.get(key);
		if (window != null) {
			// Make the OpenGL context current
			glfwMakeContextCurrent(window.getHandler());
			window.render(renderObjects, droneState);
			if (window.isTerminated())
				windows.remove(key);
			glfwMakeContextCurrent(NULL);
		} 
		if (windows.isEmpty())
			terminate();

	}

	// Terminate GLFW and free the error callback
	public void terminate() {
		if (miniMap != null)
			miniMap.close();
		glfwTerminate();
		glfwSetErrorCallback(null).free();
		this.terminated = true;
	}
	
	public Window getWindow(String key) {
		return windows.get(key);
	}

	public boolean isTerminated() {
		return this.terminated;
	}
}
