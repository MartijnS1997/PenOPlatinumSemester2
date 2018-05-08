package tests;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Autopilot.AutoPilot;
import internal.Autopilot.CasTestController;
import internal.Autopilot.Controller;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;
import internal.Testbed.Drone;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.copySign;

/**
 * Created by Martijn on 8/05/2018.
 * used to configure the collision avoidance system
 * --> simulates the drone behavior
 */
public class collisionTests {


    public static Drone drone;

    public static AutopilotConfig config;

    @Before
    public void setupMutableFixture(){


        config = new AutopilotConfig() {
            @Override
            public String getDroneID() {
                return "0";
            }

            @Override
            public float getGravity() {
                return GRAVITY;
            }

            @Override
            public float getWingX() {
                return MAIN_WING_X_POS;
            }

            @Override
            public float getTailSize() {
                return STABILIZER_POSITION;
            }

            @Override
            public float getWheelY() {
                return WHEEL_Y_POS;
            }

            @Override
            public float getFrontWheelZ() {
                return FRONT_WHEEL_Z_POS;
            }

            @Override
            public float getRearWheelZ() {
                return REAR_WHEEL_Z_POS;
            }

            @Override
            public float getRearWheelX() {
                return REAR_WHEEL_X_POS;
            }

            @Override
            public float getTyreSlope() {
                return TYRE_SLOPE;
            }

            @Override
            public float getDampSlope() {
                return DAMP_SLOPE;
            }

            @Override
            public float getTyreRadius() {
                return TYRE_RADIUS;
            }

            @Override
            public float getRMax() {
                return MAX_BRAKE_FORCE;
            }

            @Override
            public float getFcMax() {
                return MAX_FRICTION_COEFF;
            }

            @Override
            public float getEngineMass() {
                return ENGINE_MASS;
            }

            @Override
            public float getWingMass() {
                return MAIN_WING_MASS;
            }

            @Override
            public float getTailMass() {
                return STABILIZER_MASS;
            }

            @Override
            public float getMaxThrust() {
                return MAX_THRUST;
            }

            @Override
            public float getMaxAOA() {
                return MAX_AOA;
            }

            @Override
            public float getWingLiftSlope() {
                return MAIN_LIFT_SLOPE;
            }

            @Override
            public float getHorStabLiftSlope() {
                return HORIZONTAL_STABILIZER_LIFT_SLOPE;
            }

            @Override
            public float getVerStabLiftSlope() {
                return VERTICAL_STABILIZER_LIFT_SLOPE;
            }

            @Override
            public float getHorizontalAngleOfView() {
                return HORIZONTAL_ANGLE_OF_VIEW_CAMERA;
            }

            @Override
            public float getVerticalAngleOfView() {
                return VERTICAL_ANGLE_OF_VIEW_CAMERA;
            }

            @Override
            public int getNbColumns() {
                return NUMBER_OF_COLUMN_PIXELS_CAMERA;
            }

            @Override
            public int getNbRows() {
                return NUMBER_OF_ROW_PIXELS_CAMERA;
            }
        };

    }

    @Test
    public final void descendTest() throws IOException {
        Files.write(Paths.get("collisionLog.txt"), "".getBytes());


        //create the drone configured in ascend position
        float altitudeDiff = 1000f;
        float totalTravelled = 6300f;
        float TOA = altitudeDiff / totalTravelled;
        float heading = 0;
        float pitch = -(float) atan(TOA);
        float roll = 0;

        float xPos = 0;
        float yPos = 1000;
        float zPos = 0;

        float velocitySize = 50f;

        Vector position = new Vector(xPos, yPos, zPos);
        Vector orientation = new Vector(heading, pitch, roll);
        Vector velocity = getVelocityVector(velocitySize, orientation);
        Vector rotation = new Vector();

        drone = new Drone(position, velocity, orientation, rotation, config);
        drone.setAutopilotOutputs(dummyOutputs);
        AutoPilot autopilot = new AutoPilot(null);
        autopilot.setConfig(config);

        CasTestController testController = new CasTestController(autopilot);
        testController.setConfig(config);
        AutopilotInputs_v2 prevInputs = new testOutputs();
        int nbSteps = 3000;
        float deltaTime = 0.001f;

        //add the temporary minimum
        float currentMin = yPos;
        for (int i = 0; i != nbSteps; i++) {
            drone.toNextState(deltaTime);
            elapsedTime += deltaTime;
            AutopilotInputs_v2 currentInputs = new testOutputs();
            AutopilotOutputs outputs = testController.getControlActions(currentInputs, prevInputs);
            drone.setAutopilotOutputs(outputs);
            prevInputs = currentInputs;

            if(drone.getPosition().getyValue() < currentMin){
                currentMin = drone.getPosition().getyValue();
            }

            collisionLog(drone.getPosition());

        }

        System.out.println("minimal altitude " + currentMin);

    }

    @Test
    public void descendRegression() throws IOException {
        Files.write(Paths.get("collisionLog.txt"), "".getBytes());

        float deltaAltitude = 50;
        float baseDiff = 0;
        float maxDiff = 1500;
        List<Float> minAltitudes = new ArrayList<>();
        for (float altitudeDiff = baseDiff; altitudeDiff <= maxDiff; altitudeDiff += deltaAltitude) {

            //create the drone configured in ascend position
            float totalTravelled = 6300f;
            float TOA = altitudeDiff / totalTravelled;
            float heading = 0;
            float pitch = -(float) atan(TOA);
            float roll = 0;

            float xPos = 0;
            float yPos = 1000;
            float zPos = 0;

            float velocitySize = 50f;

            Vector position = new Vector(xPos, yPos, zPos);
            Vector orientation = new Vector(heading, pitch, roll);
            Vector velocity = getVelocityVector(velocitySize, orientation);
            Vector rotation = new Vector();

            drone = new Drone(position, velocity, orientation, rotation, config);
            drone.setAutopilotOutputs(dummyOutputs);
            AutoPilot autopilot = new AutoPilot(null);
            autopilot.setConfig(config);

            CasTestController testController = new CasTestController(autopilot);
            testController.setConfig(config);
            AutopilotInputs_v2 prevInputs = new testOutputs();
            int nbSteps = 3000;
            float deltaTime = 0.001f;

            //add the temporary minimum
            float currentMin = yPos;
            float zMinPos = xPos;
            for (int i = 0; i != nbSteps; i++) {
                drone.toNextState(deltaTime);
                elapsedTime += deltaTime;
                AutopilotInputs_v2 currentInputs = new testOutputs();
                AutopilotOutputs outputs = testController.getControlActions(currentInputs, prevInputs);
                drone.setAutopilotOutputs(outputs);
                prevInputs = currentInputs;

                if (drone.getPosition().getyValue() < currentMin) {
                    currentMin = drone.getPosition().getyValue();
                    zMinPos = drone.getPosition().getzValue();
                }

            }

            System.out.println("Current altitude diff: " + altitudeDiff);
            System.out.println("min altitude: " + currentMin);

            minAltitudes.add(currentMin);
            collisionLog(new Vector((float) (altitudeDiff/(2*PI)), -zMinPos, 0));
        }
    }

    protected static void collisionLog(Vector errorVector){
        String logString = errorVector.getxValue() + ";" + errorVector.getyValue() + ";" + errorVector.getzValue() + "\n";
        try {
            Files.write(Paths.get("collisionLog.txt"), logString.getBytes(), StandardOpenOption.APPEND);
        }catch (IOException e) {
            //exception handling left as an exercise for the reader
        }
    }


    private static Vector getVelocityVector(float velocity, Vector orientation){
        Vector headingVector = new Vector(0,0,-1);
        Vector velocityOrientation = PhysXEngine.droneOnWorld(headingVector, orientation);
        return velocityOrientation.scalarMult(velocity);
    }


    private class testOutputs implements AutopilotInputs_v2{

        public testOutputs() {
            heading = drone.getHeading();
            pitch = drone.getPitch();
            roll = drone.getRoll();

            xPos = drone.getPosition().getxValue();
            yPos = drone.getPosition().getyValue();
            zPos = drone.getPosition().getzValue();

            elapsedTime2 = elapsedTime;
        }

        /**
         * Here the get image is empty, this is because the autopilot flies without input from its camera
         */
        @Override
        public byte[] getImage() {
            return new byte[0];
        }

        @Override
        public float getX() {
            return xPos;
        }

        @Override
        public float getY() {
            return yPos;
        }

        @Override
        public float getZ() {
            return zPos;
        }

        @Override
        public float getHeading() {
            return heading;
        }

        @Override
        public float getPitch() {
            return pitch;
        }

        @Override
        public float getRoll() {
            return roll;
        }

        @Override
        public float getElapsedTime() {
            // first get the server
            // then extract the simulation time
            return elapsedTime2;
        }


        private float heading;
        private float pitch;
        private float roll;
        private float xPos;
        private float yPos;
        private float zPos;
        private float elapsedTime2;


    }

    private final static float deltaTime = 0.001f;
    private float elapsedTime = 0;

    private final static float INIT_COMPRESSION = 0.05f;

    private final static float GRAVITY = 9.81f;
    private final static float MAIN_WING_X_POS = 4.2f;
    private final static float STABILIZER_POSITION = 4.2f;
    private final static float WHEEL_Y_POS = 1f;
    private final static float FRONT_WHEEL_Z_POS = 1f;
    private final static float REAR_WHEEL_Z_POS = 0.5f;
    private final static float REAR_WHEEL_X_POS = 0.5f;
    private final static float TYRE_RADIUS = 0.20f;
    private final static float MAX_BRAKE_FORCE = 1000f;
    private final static float MAX_FRICTION_COEFF = 0.9f;
    private final static float ENGINE_MASS = 180f;
    private final static float MAIN_WING_MASS = 100f;
    private final static float STABILIZER_MASS = 100f;
    private final static float MAX_THRUST = 2000f;
    private final static float MAX_AOA = (float) (PI/12f);
    private final static float MAIN_LIFT_SLOPE = 10f;
    private final static float HORIZONTAL_STABILIZER_LIFT_SLOPE = 5f;
    private final static float VERTICAL_STABILIZER_LIFT_SLOPE = 5f;
    private final static float HORIZONTAL_ANGLE_OF_VIEW_CAMERA = (float) (120f*PI/180);
    private final static float VERTICAL_ANGLE_OF_VIEW_CAMERA = (float) (120f*PI/180);
    private final static int   NUMBER_OF_COLUMN_PIXELS_CAMERA = 200;
    private final static int   NUMBER_OF_ROW_PIXELS_CAMERA = 200;
    private final static float TYRE_SLOPE = (ENGINE_MASS+STABILIZER_MASS+MAIN_WING_MASS*2)*GRAVITY * 1/3f *1/INIT_COMPRESSION; // the slope of the tyre
    private final static float DAMP_SLOPE = 4000f;//4000f;

    private final static float MAIN_STABLE_INCLINATION = (float) (5f*PI/180f);
    private final static float HOR_STABLE_INCLINATION = 0f;
    private final static float VER_STABLE_INCLINATION = 0f;
    private final static float STABLE_COMPRESSION = 0.05f;
    private final static float STABLE_Y_POS= TYRE_RADIUS - STABLE_COMPRESSION + WHEEL_Y_POS;
    public final static float START_Y = WHEEL_Y_POS + TYRE_RADIUS;

    private final static float FRONT_BREAK_FORCE = 0f;
    private final static float LEFT_BREAK_FORCE = 0f;
    private final static float RIGHT_BREAK_FORCE = 0f;

    AutopilotOutputs dummyOutputs = new AutopilotOutputs() {
        @Override
        public float getThrust() {
            return 0;
        }

        @Override
        public float getLeftWingInclination() {
            return 0;
        }

        @Override
        public float getRightWingInclination() {
            return 0;
        }

        @Override
        public float getHorStabInclination() {
            return 0;
        }

        @Override
        public float getVerStabInclination() {
            return 0;
        }

        @Override
        public float getFrontBrakeForce() {
            return 0;
        }

        @Override
        public float getLeftBrakeForce() {
            return 0;
        }

        @Override
        public float getRightBrakeForce() {
            return 0;
        }
    };



}
