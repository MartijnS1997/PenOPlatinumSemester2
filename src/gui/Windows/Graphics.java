package gui.Windows;

import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MAJOR;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MINOR;
import static org.lwjgl.glfw.GLFW.GLFW_FALSE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_CORE_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_RESIZABLE;
import static org.lwjgl.glfw.GLFW.GLFW_VISIBLE;
import static org.lwjgl.glfw.GLFW.glfwDefaultWindowHints;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwSetErrorCallback;
import static org.lwjgl.glfw.GLFW.glfwTerminate;
import static org.lwjgl.glfw.GLFW.glfwWindowHint;
import static org.lwjgl.system.MemoryUtil.NULL;

import TestbedAutopilotInterface.DroneGuiState;
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
	private static final String droneID = "0";
	
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
	}
	
	public void addWindow(String key, Window window) {
		windows.put(key, window);
	}
	
	public void makeTextWindow(String title, int width, int height, int xPos, int yPos, Drone drone) {
		
		this.textWindow = TextWindow.createAndShowWindow(this, title, width, height, xPos, yPos, drone);
	}
	
	public void makeButtonWindow() {
		ButtonWindow.createAndShowWindow(this);
	}
	
	public void renderWindows(Set<GraphicsObject> renderObjects, Drone drone) {
		// Poll for window events. The key callback above will only be
		// invoked during this call.
		glfwPollEvents();
		
		if (this.textWindow != null)
			this.textWindow.update(drone);
		
		for (String key: windows.keySet()) {
			Window window = windows.get(key);
			
			// Make the OpenGL context current
			glfwMakeContextCurrent(window.getHandler());
			window.render(renderObjects, drone);
			if (window.isTerminated()) {
				windows.remove(key);
				break;
			}
			glfwMakeContextCurrent(NULL);
		}
		if (windows.isEmpty())
			terminate();	
	}
	
	public void renderWindow(String key, Set<GraphicsObject> renderObjects, Drone drone) {
		// Poll for window events. The key callback above will only be
		// invoked during this call.
		glfwPollEvents();

		Window window = windows.get(key);
		if (window != null) {
			// Make the OpenGL context current
			glfwMakeContextCurrent(window.getHandler());
			window.render(renderObjects, drone);
			if (window.isTerminated())
				windows.remove(key);
			glfwMakeContextCurrent(NULL);
		} 
		if (windows.isEmpty())
			terminate();

	}

	// Terminate GLFW and free the error callback
	public void terminate() {
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