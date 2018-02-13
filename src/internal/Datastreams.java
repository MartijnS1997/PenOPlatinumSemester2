
//****MOVED TO DRONE CLASS*****


//	package internal;
//
//	import java.io.DataOutputStream;
//	import java.io.FileOutputStream;
//	import java.io.IOException;
//	import internal.Drone;
//	import Autopilot.AutopilotConfig;
//	import Autopilot.AutopilotConfigWriter;
//	import Autopilot.AutopilotInputs;
//	import Autopilot.AutopilotInputsWriter;
//	import Autopilot.AutopilotOutputs;
//	import Autopilot.AutopilotOutputsWriter;
//
//	public class Datastreams {
//		 /**
//	     * makes a data output stream for the autopilotconfig 
//	     * @throws IOException
//	     */
//	    public void setupAutopilotConfig()throws IOException{
//	    	DataOutputStream dataOutputStream =
//	                new DataOutputStream(new FileOutputStream(dataStreamLocationConfig));
//	    	
//
//	        AutopilotConfig value = new AutopilotConfig() {
//	            public float getGravity() { return GRAVITY; }
//	            public float getWingX() { return wingX; }
//	            public float getTailSize() { return tailSize; }
//	            public float getEngineMass() { return getEngineMass(); }
//	            public float getWingMass() { return getLeftWing().getMass(); }
//	            public float getTailMass() { return getLeftWing().getMass(); }
//	            public float getMaxThrust() { return getMaxThrust(); }
//	            public float getMaxAOA() { return getLeftWing().getAngleOfAttack(); }
//	            public float getWingLiftSlope() { return getLeftWing().getLiftSlope(); }
//	            public float getHorStabLiftSlope() { return getHorizontalStab().getLiftSlope(); }
//	            public float getVerStabLiftSlope() { return getVerticalStab().getLiftSlope(); }
//	            public float getHorizontalAngleOfView() { return getHorizontalAngleOfView(); }
//	            public float getVerticalAngleOfView() { return getVerticalAngleOfView(); }
//	            public int getNbColumns() { return getNbOfColumns(); }
//	            public int getNbRows() { return getNbOfRows(); }
//	        };
//	        AutopilotConfigWriter.write(dataOutputStream, value);    	
//	    	dataOutputStream.close();
//	    }
//		/**
//		 * 
//		 * @throws IOException
//		 */
//	    public void setupAutopilotInputs()throws IOException{
//	    	DataOutputStream dataOutputStream =
//	                new DataOutputStream(new FileOutputStream(dataStreamLocationInputs));
//	    	
//	    	Vector pos = getPosition();
//	    	Vector posOnWorld = droneOnWorld(pos);
//	    	float x = posOnWorld.getxValue();
//	    	float y = posOnWorld.getyValue();
//	    	float z = posOnWorld.getzValue();
//	    	
//	    	float heading = getOrientation().getxValue();
//	    	float pitch = getOrientation().getyValue();
//	    	float roll = getOrientation().getzValue();
//	    	
//	    	
//	    	AutopilotInputs value = new AutopilotInputs() {
//	            public byte[] getImage() { return image; }
//	            public float getX() { return x; }
//	            public float getY() { return y; }
//	            public float getZ() { return z; }
//	            public float getHeading() { return heading; }
//	            public float getPitch() { return pitch; }
//	            public float getRoll() { return roll; }
//	            public float getElapsedTime() { return elapsedTime; }
//	        };
//
//	        AutopilotInputsWriter.write(dataOutputStream, value);
//	        
//	    	dataOutputStream.close();
//	    }
//	    
//	    
//		/**
//		 * 
//		 * @throws IOException
//		 */
//	    public void setupAutopilotOutputs()throws IOException{
//	    	DataOutputStream dataOutputStream =
//	                new DataOutputStream(new FileOutputStream(dataStreamLocationInputs));
//	    	
//	    	 AutopilotOutputs value = new AutopilotOutputs() {
//	             public float getThrust() { return getThrust(); }
//	             public float getLeftWingInclination() { return getLeftWingInclination(); }
//	             public float getRightWingInclination() { return getRightWingInclination(); }
//	             public float getHorStabInclination() { return getHorStabInclination(); }
//	             public float getVerStabInclination() { return getVerStabInclination(); }
//	         };
//
//	        AutopilotOutputsWriter.write(dataOutputStream, value);
//	        
//	    	dataOutputStream.close();
//	    }    
//	    
//	    
//	    /**
//	     * @return Returns the horizontal angle of view of the camera
//	     */
//	    public float getHorizontalAngleOfView(){
//	    	return this.horizontalAngleOfView;
//	    }
//	    /**
//	     * @return Returns the vertical angle of view of the camera
//	     */
//	    public float getVerticalAngleOfView(){
//	    	return this.verticalAngleOfView;
//	    }
//	    
//	    /**
//	     * @return Returns the number of columns of pixels the camera image has 
//	     */
//	    public int getNbOfColumns(){
//	    	return this.nbColumns;
//	    }
//	    /**
//	     * @return Returns the number of rows of pixels the camera image has
//	     */
//	    public int getNbOfRows(){
//	    	return this.nbRows;
//	    }
//	    
//	    /**
//	     * Variable for the filename that's created when making the AutopilotConfig datastream 
//	     */
//	    private String dataStreamLocationConfig = "APConfig.txt";
//	    
//	    /**
//	     * Variable for the filename that's created when making the AutopilotInputs datastream 
//	     */
//	    private String dataStreamLocationInputs = "APInputs.txt";
//	    
//	    /**
//	     * Variable for the filename that's created when making the AutopilotOutputs datastream 
//	     */
//	    private String dataStreamLocationOutputs = "APOutputs.txt";
//	    /**
//	     * Variable for the horizontal angle of view (in degrees, immutable)
//	     */
//	    private float horizontalAngleOfView = 120; //degree
//	    /**
//	     * Variable for the vertical angle of view (in degrees, immutable)
//	     */
//	    private float verticalAngleOfView = 120; //degree
//	    /**
//	     * Number of columns of pixels in the camera image (immutable)
//	     */
//	    private int nbColumns = 200;
//	    /**
//	     * Number of rows of pixels in the camera image (immutable)
//	     */
//	    private int nbRows = 200;
//	}
//
