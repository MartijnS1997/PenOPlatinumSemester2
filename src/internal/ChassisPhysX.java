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
        this.rearLeftTyre = new TyrePhysX(rearLeftTyrePos, tyreReadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearRightTyre = new TyrePhysX(rearRightTyrePos, tyreReadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);


    }

    /**
     * Updates the chassis to the next state
     */
    public void nextStateChasis(Vector orientation, Vector position){
        this.getFrontTyre().nextState(orientation, position);
        this.getRearLeftTyre().nextState(orientation, position);
        this.getRearRightTyre().nextState(orientation, position);
    }


    //TODO complete this
    public Vector netChassisForces(Vector orientation, Vector position, Vector velocity, float brakeForce, float deltaTime, Vector DroneForce, Vector DroneMoment){
        //first calculate the known forces exerted by the tires
        TyrePhysX frontTyre = this.getFrontTyre();
        Vector frontTyreForce = frontTyre.getNetForceTyre(orientation, position, velocity, brakeForce, deltaTime);
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(orientation, position, velocity, brakeForce, deltaTime);
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(orientation, position, velocity, brakeForce, deltaTime);

        //if the velocity of the tire along the x-axis is non zero, exert y-force * fricCoeff on the rear tyres
        //otherwise we would need an infinite pulse

        //if not exert min(y-force*fricCoeff, L) with L the force along the x-axis in the drone axis system
        return null;
    }

    public Vector netChassisMoment(Vector orientation, Vector DroneForce, Vector DroneMoment){
        return null;
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
    TyrePhysX rearLeftTyre;
    TyrePhysX rearRightTyre;
}
