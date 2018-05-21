package internal.Physics;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Testbed.Drone;
import internal.Testbed.DroneState;
import internal.Helper.Vector;

import static java.lang.Math.abs;

/**
 * Created by Martijn on 14/02/2018.
 * A class used for simulating the behavior of the tyres of the drone
 */
public class ChassisPhysX {

    /**
     * Constructor of the chassis
     * @param config the configuration of the drone
     */
    public ChassisPhysX(AutopilotConfig config, PhysXEngine physXEngine){
        //extract the constants for the wheels
        float tyreRadius = config.getTyreRadius();
        float tyreSlope = config.getTyreSlope();
        float dampSlope = config.getDampSlope();
        float maxBrake = config.getRMax();
        float maxFricCoeff = config.getFcMax();

        // extract the positions of the wheels
        Vector frontTyrePos = new Vector(0f, -abs(config.getWheelY()), -abs(config.getFrontWheelZ()));
        Vector rearLeftTyrePos = new Vector(-abs(config.getRearWheelX()), -abs(config.getWheelY()), abs(config.getRearWheelZ()));
        Vector rearRightTyrePos = new Vector(abs(config.getRearWheelX()), -abs(config.getWheelY()), abs(config.getRearWheelZ()));

        //construct the wheels
        this.frontTyre = new TyrePhysX(this, frontTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearLeftTyre = new RearTyrePhysX(this, rearLeftTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearRightTyre = new RearTyrePhysX(this, rearRightTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.associatedPhysicsEngine = physXEngine;


    }

    /**
     * Checks if the chassis touches the ground, used in validation for usage of drone constructor in the air
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if and only if one of the tyres touches the ground
     */
    protected boolean touchesGround(Vector orientation, Vector position){

        TyrePhysX frontTyre = this.getFrontTyre();
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        boolean groundTouched = frontTyre.getTyreDistanceToGround(orientation, position) <= frontTyre.getTyreRadius();
        groundTouched = groundTouched || (rearLeftTyre.getTyreDistanceToGround(orientation, position) <= rearLeftTyre.getTyreRadius());
        return groundTouched || (rearRightTyre.getTyreDistanceToGround(orientation, position) <= rearRightTyre.getTyreRadius());
    }


    /**
     * Saves the net forces exerted on the chassis of the drone to the provided drone forces object
     * this is the sum of the forces of all wheels acting on the drone
     * @param droneForces the forces acting on the drone, containing all the airborne forces (lift, thrust and gravity)
     * @param state the state of the drone (position, velocity, etc)
     * @param inputs the inputs from the autopilot (containing the commands for the brake force)
     * @param deltaTime the time step used for the simulation
     */
    public void netChassisForces(FastTransformations fastTransformations, DroneForces droneForces, DroneState state, AutopilotOutputs inputs, float deltaTime /*Vector nonChassisForces*/){
        Vector scaledNonChassisForces = droneForces.getTotalAirborneForces();//nonChassisForces.scalarMult(nbOfActiveTyres);
        //first calculate the known forces exerted by the tires
        TyrePhysX frontTyre = this.getFrontTyre();
//        System.out.println("front: ");
        Vector frontTyreForce = frontTyre.getNetForceTyre(fastTransformations, state, scaledNonChassisForces, inputs.getFrontBrakeForce(), deltaTime, state.getPrevFrontTyreDelta() );
//        System.out.println("Left: ");
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(fastTransformations, state, scaledNonChassisForces, inputs.getLeftBrakeForce(), deltaTime, state.getPrevRearLeftTyreDelta());
//        System.out.println("Right: ");
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(fastTransformations, state,scaledNonChassisForces, inputs.getRightBrakeForce(), deltaTime, state.getPrevRearRightTyreDelta());

//        System.out.println();
        //save the forces so the moments can access them
        droneForces.setWheelForces(frontTyreForce, rearLeftTyreForce, rearRightTyreForce);
    }

    private static int nbOfActiveTyres(AutopilotOutputs inputs){
        int totalActiveTyres = 0;
        if(inputs.getFrontBrakeForce() > 0){
            totalActiveTyres ++;
        }
        if(inputs.getRightBrakeForce() > 0){
            totalActiveTyres ++;
        }
        if(inputs.getLeftBrakeForce() >0){
            totalActiveTyres ++;
        }

        return totalActiveTyres;
    }

    /**
     * Calculates the net moment exerted by the chassis on the drone (in drone axis system)
     * @param state the state of the drone at moment of invoking the method
     * @return the net chassis moment in the drone axis system
     */
    public Vector netChassisMoment(FastTransformations transformations, DroneForces droneForces, DroneState state){

        //get the tyres
        TyrePhysX frontTyre = this.getFrontTyre();
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        TyrePhysX rearRightTyre = this.getRearRightTyre();

        //calculate the moment for every tyre based on the force of the tyre saved in the drone forces object
        Vector frontTyreMoment = frontTyre.getNetMomentTyre(transformations, state, droneForces.getFrontWheelForce());
        Vector rearLeftTyreMoment = rearLeftTyre.getNetMomentTyre(transformations, state, droneForces.getRearLeftWheelForce());
        Vector rearRightTyreMoment = rearRightTyre.getNetMomentTyre(transformations, state, droneForces.getRearRightWheelForce());

        Vector[] moments = new Vector[]{frontTyreMoment, rearLeftTyreMoment, rearRightTyreMoment};
        return Vector.sumVectorArray(moments);

    }

    /**
     * Sets the correct tyre delta for all the tyres of the drone
     * @param drone the drone where the tyre delta needs to be initialized
     */
    public void setDroneTyreDelta(Drone drone){
        Vector orientation = drone.getOrientation();
        Vector position = drone.getPosition();
        if(!drone.getPhysXEngine().chassisTouchesGround(orientation, position)){
            throw new IllegalStateException("Chassis is not Touching the ground");
        }
        //acquire the tyres
        TyrePhysX frontTyre = this.getFrontTyre();
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        TyrePhysX rearRightTyre = this.getRearRightTyre();

//        System.out.println("prev front delta: " + frontTyre.calcRadiusDelta(orientation, position));
//        System.out.println("prev rear delta: " + rearLeftTyre.calcRadiusDelta(orientation, position));

        //then set the deltas
        drone.setPrevFrontTyreDelta(frontTyre.calcRadiusDelta(orientation, position));
        drone.setPrevRearLeftTyreDelta(rearLeftTyre.calcRadiusDelta(orientation, position));
        drone.setPrevRearRightTyreDelta(rearRightTyre.calcRadiusDelta(orientation, position));
    }

    /**
     * Returns true if one of the tyres has crashed
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if the chassis experienced a crash
     */
    public boolean checkCrash(Vector orientation, Vector position){
        if(this.getFrontTyre().checkCrash(orientation, position))
            return true;
        else if(this.getRearLeftTyre().checkCrash(orientation, position))
            return true;
        else if(this.getRearRightTyre().checkCrash(orientation, position))
            return true;
        else {
            return false;
        }
    }


    public TyrePhysX getFrontTyre() {
        return frontTyre;
    }

    public TyrePhysX getRearLeftTyre() {
        return rearLeftTyre;
    }

    public TyrePhysX getRearRightTyre() {
        return rearRightTyre;
    }

    public PhysXEngine getAssociatedPhysicsEngine() {
        return associatedPhysicsEngine;
    }

    /**
     * Instance variables: the tyres of the chassis
     */
    TyrePhysX frontTyre;
    RearTyrePhysX rearLeftTyre;
    RearTyrePhysX rearRightTyre;

    private Vector frontTyreForces;
    private Vector rearLeftTyreForces;
    private Vector rearRightTyreForces;
    private PhysXEngine associatedPhysicsEngine;
}
