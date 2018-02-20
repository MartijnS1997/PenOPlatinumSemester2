package gui;

import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.util.Arrays;

import internal.HSVconverter;
import internal.Vector;
import math.Vector3f;

public class Tile implements GraphicsObject {

//	static float[] positions = new float[]{
//
//			-0.5f,  0.0f, -0.5f,
//			-0.5f,  0.0f,  0.5f,
//			0.5f,  0.0f,  0.5f,
//			0.5f,  0.0f, -0.5f,
//	};
//	static int[] indices = new int[]{
//			0, 1, 2,
//			0, 2, 3,
//	};
	
	static float[] positions = new float[]{
			// VO
			-0.5f,  0.5f,  0.5f,
			// V1
			-0.5f, -0.5f,  0.5f,
			// V2
			0.5f, -0.5f,  0.5f,
			// V3
			0.5f,  0.5f,  0.5f,
			
			// V4
			-0.5f,  0.5f, -0.5f,
			// V5
			-0.5f,  0.5f,  0.5f,
			// V6
			0.5f,  0.5f,  0.5f,
			// V7
			0.5f,  0.5f, -0.5f,
			
			// V8
			0.5f,  0.5f,  0.5f,
			// V9
			0.5f, -0.5f,  0.5f,
			// V10
			0.5f, -0.5f, -0.5f,
			// V11
			0.5f,  0.5f, -0.5f,
			
			// V12
			-0.5f, -0.5f, -0.5f,
			// V13
			-0.5f, -0.5f,  0.5f,
			// V14
			-0.5f,  0.5f,  0.5f,
			// V15
			-0.5f,  0.5f, -0.5f,
			
			// V16
			0.5f, -0.5f,  0.5f,
			// V17
			-0.5f, -0.5f,  0.5f,
			// V18
			-0.5f, -0.5f, -0.5f,
			// V19
			0.5f, -0.5f, -0.5f,
			
			// V20
			0.5f, -0.5f, -0.5f,
			// V21
			-0.5f, -0.5f, -0.5f,
			// V22
			-0.5f,  0.5f, -0.5f,
			// V23
			0.5f,  0.5f, -0.5f,
	};
	
	static int[] indices = new int[]{
			// Front face
			0, 1, 3, 3, 1, 2,
			// Top Face
			4, 5, 6, 7, 4, 6,
			// Right face
			8, 9, 10, 11, 8, 10,
			// Left face
			12, 13, 14, 12, 14, 15,
			// Bottom face
			16, 17, 18, 16, 18, 19,
			// Back face
			20, 21, 22, 20, 22, 23,
	};
	
	private float[] colours;

	static Graphics g;
	
	private Mesh mesh;
	private Vector3f position = new Vector3f();
	private Vector3f size = new Vector3f(1f, 1f, 1f);
	
	static public void setGraphics(Graphics graphics) {
		g = graphics;
	}
	
	public Tile(Vector3f colour) {
		System.out.println("helo");
		setColours(colour);
		
		for (String key: g.windows.keySet()) {
			glfwMakeContextCurrent(g.windows.get(key).getHandler());
			this.mesh = new Mesh();
			mesh.init(positions, colours, indices);
			glfwMakeContextCurrent(NULL);
		}
		System.out.println("helo2");
	}
	
	public Tile(Vector3f position, Vector3f colour) {
		this(colour);
		
		this.position = position;
	}
	
	public void render() {
		this.mesh.render();
	}
	
	public void delete() {
		this.mesh.delete();
	}

	public Vector getPosition(){
		return Vector.vector3fToVector(this.getPos());
	}
	
	public Vector3f getSize() {
		return this.size;
	}
	
	public Vector3f getPos() {
		return this.position;
	}
	
	public void setSize(float size) {
		this.size = new Vector3f(size, size, size);
	}
	
	public void setSize(Vector3f size) {
		this.size = size;
	}
	
	public void setSize(Vector vector) {
		this.setSize(vector.convertToVector3f());
	}
	
	public void setColours(Vector3f colour) {	
		Vector3f posY = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negY = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f posX = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negX = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f posZ = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negZ = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		
		this.colours = new float[]{
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
		};
	}

	@Override
	public String toString() {
		return "Cube{" +
				"colours=" + Arrays.toString(colours) +
				'}';
	}
}
