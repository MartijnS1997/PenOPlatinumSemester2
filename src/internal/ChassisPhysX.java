package internal;

import Autopilot.AutopilotConfig;
import Autopilot.AutopilotInputs;
import Autopilot.AutopilotInputs_v2;
import Autopilot.AutopilotOutputs;

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
    public Vector netChassisForces(DroneState state, AutopilotOutputs inputs, float deltaTime){
        //first calculate the known forces exerted by the tires
        TyrePhysX frontTyre = this.getFrontTyre();
        Vector frontTyreForce = frontTyre.getNetForceTyre(state, inputs.getFrontBrakeForce(), deltaTime, state.getPrevFrontTyreDelta() );
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(state, inputs.getLeftBrakeForce(), deltaTime, state.getPrevRearLeftTyreDelta());
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(state, inputs.getRightBrakeForce(), deltaTime, state.getPrevRearRightTyreDelta());

        Vector[] forces = {frontTyreForce, rearLeftTyreForce, rearRightTyreForce};

        return Vector.sumVectorArray(forces);
    }

    /**
     * Calculates the net moment exerted by the chassis on the drone (in drone axis system)
     * @param state the state of the drone at moment of invoking the method
     * @param deltaTime the time passed between two simulation steps
     * @return the net chassis moment in the drone axis system
     */
    public Vector netChassisMoment(DroneState state, AutopilotOutputs inputs, float deltaTime){
        //System.out.println("NEW ITERATION, ROLL: " + state.getOrientation().getzValue());
        TyrePhysX frontTyre = this.getFrontTyre();
        //System.out.println("Front-tyre: ");
        Vector frontTyreForce = frontTyre.getNetForceTyre(state, inputs.getFrontBrakeForce(), deltaTime, state.getPrevFrontTyreDelta() );
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        //System.out.println("Left-Tyre: ");
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(state, inputs.getLeftBrakeForce(), deltaTime, state.getPrevRearLeftTyreDelta());
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        //System.out.println("Right-trye: ");
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(state, inputs.getRightBrakeForce(), deltaTime, state.getPrevRearRightTyreDelta());

        Vector frontTyreMoment = frontTyre.getNetMomentTyre(state, frontTyreForce);
        Vector rearLeftTyreMoment = rearLeftTyre.getNetMomentTyre(state, rearLeftTyreForce);
        Vector rearRightTyreMoment = rearRightTyre.getNetMomentTyre(state, rearRightTyreForce);

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
