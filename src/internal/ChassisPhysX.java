package internal;

import Autopilot.AutopilotConfig;

/**
 * Created by Martijn on 14/02/2018.
 * A class used for simulating the behavior of the tyres of the drone
 */
public class ChassisPhysX {

    /**
     * Constructor of the chassis
     * @param config the configuration of the drone
     */
    public ChassisPhysX(AutopilotConfig config){
        //extract the constants for the wheels
        float tyreReadius = config.getTyreRadius();
        float tyreSlope = config.getTyreSlope();
        float dampSlope = config.getDampSlope();
        float maxBrake = config.getRMax();
        float maxFricCoeff = config.getFcMax();

        // extract the positions of the wheels
        Vector frontTyrePos = new Vector(0f, config.getWheelY(), config.getFrontWheelZ());
        Vector rearLeftTyrePos = new Vector(-config.getRearWheelX(), config.getWheelY(), config.getRearWheelZ());
        Vector rearRightTyrePos = new Vector(config.getRearWheelX(), config.getWheelY(), config.getRearWheelZ());

        //construct the wheels
        this.frontTyre = new TyrePhysX(frontTyrePos, tyreReadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearLeftTyre = new RearTyrePhysX(rearLeftTyrePos, tyreReadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearRightTyre = new RearTyrePhysX(rearRightTyrePos, tyreReadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);


    }


    //TODO complete this
    public Vector netChassisForces(Vector orientation, Vector rotation, Vector position, Vector velocity, float brakeForce, float deltaTime, float prevTyreDeltaFront, float prevTyreDeltaRearLeft, float prevTyreDeltaRearRight){
        //first calculate the known forces exerted by the tires
        TyrePhysX frontTyre = this.getFrontTyre();
        Vector frontTyreForce = frontTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaFront );
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaRearLeft);
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaRearRight);

        Vector[] forces = {frontTyreForce, rearLeftTyreForce, rearRightTyreForce};

        return Vector.sumVectorArray(forces);
    }

    /**
     * Calculates the net moment exerted by the chassis on the drone (in drone axis system)
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone
     * @param position the position of the drone (world axis system)
     * @param velocity the velocity of the drone (world axis system)
     * @param brakeForce the brake force exerted on the wheels
     * @param deltaTime the dime difference between steps
     * @return the net chassis moment in the drone axis system
     */
    public Vector netChassisMoment(Vector orientation, Vector rotation, Vector position, Vector velocity, float brakeForce, float deltaTime, float prevTyreDeltaFront, float prevTyreDeltaRearLeft, float prevTyreDeltaRearRight){
        TyrePhysX frontTyre = this.getFrontTyre();
        Vector frontTyreForce = frontTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaFront );
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaRearLeft);
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(orientation, rotation, position, velocity, brakeForce, deltaTime, prevTyreDeltaRearRight);

        Vector frontTyreMoment = frontTyre.getNetMomentTyre(orientation, position, frontTyreForce);
        Vector rearLeftTyreMoment = rearLeftTyre.getNetMomentTyre(orientation, position, rearLeftTyreForce);
        Vector rearRightTyreMoment = rearRightTyre.getNetMomentTyre(orientation, position, rearRightTyreForce);

        Vector[] moments = {frontTyreMoment, rearLeftTyreMoment, rearRightTyreMoment};

        return Vector.sumVectorArray(moments);
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


    private TyrePhysX getFrontTyre() {
        return frontTyre;
    }

    private TyrePhysX getRearLeftTyre() {
        return rearLeftTyre;
    }

    private TyrePhysX getRearRightTyre() {
        return rearRightTyre;
    }

    /**
     * Instance variables: the tyres of the chassis
     */
    TyrePhysX frontTyre;
    RearTyrePhysX rearLeftTyre;
    RearTyrePhysX rearRightTyre;
}
