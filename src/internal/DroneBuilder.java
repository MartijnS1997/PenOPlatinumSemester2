package internal;


import Autopilot.AutopilotConfig;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 23/10/2017.
 * @author Martijn Sauwens
 */
public class DroneBuilder {

    /**
     * Constants used to create the drone & configure the autopilot
     */

    public final static float  BETA_ENGINE_MASS = .250f;
    public final static float  BETA_MAX_THRUST = 5.0f;
    public final static float  BETA_MAIN_WING_MASS = .25f;
    public final static float  BETA_STABILIZER_MASS = 0.125f;
    public final static float  BETA_MAINWING_START_INCL = (float) PI/12.0f;
    public final static float  BETA_STABS_START_INCL = 0.0f;
    public final static float  BETA_MAX_ANGLE_OF_ATTACK = (float) ( PI/2.0 - 0.001f);
    public final static float  BETA_LIFT_COEFFICIENT = .3f;
    public final static float  BETA_LIFT_COEFFICIENT_STAB =.15f;
    public final static Vector BETA_LEFTWING_POS = new Vector(-.5f, 0.0f, 0.0f);
    public final static Vector BETA_RIGHTWING_POS = new Vector(.5f, 0.0f, 0.0f);
    public final static Vector BETA_STABILIZE_POS = new Vector(0.0f, 0.0f, .5f);
    public final static Vector BETA_STARTPOS = new Vector();
    public final static Vector BETA_START_VEL = new Vector(0,0,-6.32f);
    public final static Vector BETA_START_ORIENTATION = new Vector();
    public final static Vector BETA_START_ROTATION = new Vector();


    public final static float ENGINE_MASS = 5.0f;
    public final static float MAX_THRUST = 250.0f;
    public final static float MAIN_WING_MASS = 2.5f;
    public final static float STABILIZER_MASS = 1.25f;
    public final static float MAINWING_START_INCL = (float) PI/12.0f;
    public final static float STABS_START_INCL = 0.0f;
    public final static float MAX_ANGLE_OF_ATTACK = (float) ( PI/2.0 - 0.001f);
    public final static float LIFT_COEFFICIENT = 5.0f;
    public final static float LIFT_COEFFICIENT_STAB =1.0f;
    public final static Vector LEFTWING_POS = new Vector(-1.0f, 0.0f, 0.0f);
    public final static Vector RIGHTWING_POS = new Vector(1.0f, 0.0f, 0.0f);
    public final static Vector STABILIZE_POS = new Vector(0.0f, 0.0f, 2.0f);
    public final static Vector STARTPOS = new Vector();
    public final static Vector START_VEL = new Vector(0,0,-6.32f);
    public final static Vector START_ORIENTATION = new Vector();
    public final static Vector START_ROTATION = new Vector();


    public DroneBuilder(boolean balanced) {
        this.balanced = true;
    }


    public Drone createDrone(){
        return createDrone(ALPHA_CONFIG);
    }

    public Drone createDrone(String configMode) {
        HorizontalWingPhysX rightMain, leftMain, horizontalStabilizer;
        VerticalWingPhysX verticalStabilizer;
        Drone drone;

        rightMain = new HorizontalWingPhysX(RIGHTWING_POS, LIFT_COEFFICIENT, MAIN_WING_MASS, MAX_ANGLE_OF_ATTACK, MAINWING_START_INCL);
        leftMain = new HorizontalWingPhysX(LEFTWING_POS, LIFT_COEFFICIENT, MAIN_WING_MASS, MAX_ANGLE_OF_ATTACK, MAINWING_START_INCL);
        horizontalStabilizer = new HorizontalWingPhysX(STABILIZE_POS, LIFT_COEFFICIENT_STAB, STABILIZER_MASS, MAX_ANGLE_OF_ATTACK, STABS_START_INCL);
        verticalStabilizer = new VerticalWingPhysX(STABILIZE_POS, LIFT_COEFFICIENT_STAB, STABILIZER_MASS, MAX_ANGLE_OF_ATTACK, STABS_START_INCL);

        drone = new Drone(STARTPOS, START_VEL, START_ORIENTATION, START_ROTATION, createConfig(configMode));

        // if the drone needs to be balanced, do so (balancing is the act of setting the vertical force to 0
        // and the Z value to 0
        if(this.isBalanced()){
            //balanceDrone(drone);
        }

        System.out.println("Drone velocity: " + drone.getVelocity());
        System.out.println("Drone thrust: " + drone.getThrust());

        return drone;
    }

    /**
     * Constants to create the autopilotConfig & Autopilot
     */
    private final static float HORIZONTALVIEW = (float) (120.0f* PI/180.0f);
    private final static float VERTICALVIEW = (float) (120.0f* PI/180.0f);
    private final static int NB_ROWS = 200;
    private final static int NB_COLS= 200;

    /**
     * Creates an un configured autopilot
     * @return an autopilot class object
     */
    public AutoPilot createAutoPilot(){
       return  new AutoPilot();
    }

    /**
     * Pseudo constructor for a configuration of an autopilotConfig
     * @return an Autopilot config
     */
    public AutopilotConfig createConfig(String configMode) {
        switch (configMode) {
            case PhysXEngine.ALPHA_MODE:
                return new AutopilotConfig() {

                    @Override
                    public float getGravity() {
                        return 9.81f;
                    }

                    @Override
                    public float getWingX() {
                        return RIGHTWING_POS.getxValue();
                    }

                    @Override
                    public float getTailSize() {
                        return STABILIZE_POS.getzValue();
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
                        return (float) (40 * PI / 180f);
                    }

                    @Override
                    public float getWingLiftSlope() {
                        return LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorStabLiftSlope() {
                        return LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getVerStabLiftSlope() {
                        return LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorizontalAngleOfView() {
                        return HORIZONTALVIEW;
                    }

                    @Override
                    public float getVerticalAngleOfView() {
                        return VERTICALVIEW;
                    }

                    @Override
                    public int getNbColumns() {
                        return NB_COLS;
                    }

                    @Override
                    public int getNbRows() {
                        return NB_ROWS;
                    }
                };

            case PhysXEngine.BETA_MODE:
                return new AutopilotConfig(){
                    @Override
                    public float getGravity() {
                        return 9.81f;
                    }

                    @Override
                    public float getWingX() {
                        return  BETA_RIGHTWING_POS.getxValue();
                    }

                    @Override
                    public float getTailSize() {
                        return BETA_STABILIZE_POS.getzValue();
                    }

                    @Override
                    public float getEngineMass() {
                        return BETA_ENGINE_MASS;
                    }

                    @Override
                    public float getWingMass() {
                        return BETA_MAIN_WING_MASS;
                    }

                    @Override
                    public float getTailMass() {
                        return BETA_STABILIZER_MASS;
                    }

                    @Override
                    public float getMaxThrust() {
                        return BETA_MAX_THRUST;
                    }

                    @Override
                    public float getMaxAOA() {
                        return (float) (40*PI/180f);
                    }

                    @Override
                    public float getWingLiftSlope() {
                        return BETA_LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorStabLiftSlope() {
                        return BETA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getVerStabLiftSlope() {
                        return BETA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getHorizontalAngleOfView() {
                        return HORIZONTALVIEW;
                    }

                    @Override
                    public float getVerticalAngleOfView() {
                        return VERTICALVIEW;
                    }

                    @Override
                    public int getNbColumns() {
                        return NB_COLS;
                    }

                    @Override
                    public int getNbRows() {
                        return NB_ROWS;
                    }
                };
            default:
                throw new IllegalArgumentException("wrong config mode");
        }

    }




    public boolean isBalanced() {
        return balanced;
    }

    /**
     * Varaible that stores if the drone needs to be balanced before flight
     */
    private boolean balanced;

    private final static String ALPHA_CONFIG = "ALPHA_CONFIG";
    private final static String BETA_CONFIG = "BETA_CONFIG";


}
/*
        float droneMass = 0.0f;
        float engineMass = 10.0f;
        float maxThrust = 100.0f;
        float mainWingMass = 5.0f;
        float stabilizerMass = 2.0f;
        float mainWingStartIncl = (float) (Math.PI / 12.0f);
        float stabInclination = 0.0f;
        float maxAngleOfAttack = (float) (Math.PI / 2.0f - 0.01f);
        float liftCoefficient = 1.0f;
        Vector leftWingPos = new Vector(-4.0f, 0.0f, 0.0f);
        Vector rightWingPos = new Vector(4.0f, 0.0f, 0.0f);
        Vector stabilizerPos = new Vector(0.0f, 0.0f, 8.0f);
        Vector startPos = new Vector(0.0f, 0.0f, 0.0f);
        Vector startVelocity = new Vector(0.0f, 0.0f, -20.490349f);
        Vector startOrientation = new Vector(0.0f, 0.0f, 0.0f);
        Vector startRotation = new Vector(0.0f, 0.0f, 0.0f);

        rightMain =new HorizontalWingPhysX(rightWingPos, liftCoefficient, mainWingMass, maxAngleOfAttack, mainWingStartIncl);

        leftMain =new HorizontalWingPhysX(leftWingPos, liftCoefficient, mainWingMass, maxAngleOfAttack, mainWingStartIncl);

        horizontalStabilizer =new HorizontalWingPhysX(stabilizerPos, liftCoefficient, stabilizerMass, maxAngleOfAttack,0.0f);

        verticalStabilizer =new VerticalWingPhysX(stabilizerPos, liftCoefficient, stabilizerMass, maxAngleOfAttack, stabInclination);

        drone =new Drone(droneMass, engineMass, maxThrust, startPos, startVelocity, startOrientation, startRotation, rightMain, leftMain, horizontalStabilizer, verticalStabilizer, null);

        return drone;
 */



