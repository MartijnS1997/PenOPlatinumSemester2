package internal;

import static java.lang.Math.sin;

/**
 * Created by Martijn on 13/02/2018.
 * A class of tyres, normal Tyres only exert a force in the y-component as an reaction to deltaRadius
 */
//TODO: checker for tyre position
public class TyrePhysX {

    public TyrePhysX(Vector tyrePosition, float tyreRadius, float tyreSlope, float dampSlope, float maxBrake, float maxFricCoeff) {
        this.tyrePosition = tyrePosition;
        this.tyreRadius = tyreRadius;
        this.tyreSlope = tyreSlope;
        this.dampSlope = dampSlope;
        this.maxBrake = maxBrake;
        this.maxFricCoeff = maxFricCoeff;
    }

    /**
     * returns the net force exerted by the tyre: the force exerted by the deltaRadius and the brakes in world axis system
     * @param orientation the orientation of the drone
     * @param position the position of the drone (world axis system)
     * @param velocity the velocity of the drone (world axis system)
     * @param brakeForce the force exerted by the brakes
     * @param prevTyreDelta the previous tyre compression
     * @return the net forces exerted by the tyres in the world axis system
     */
    public Vector getNetForceTyre(Vector orientation, Vector rotation, Vector position, Vector velocity, float brakeForce, float deltaTime, float prevTyreDelta){
        float groundDist = this.getTyreDistanceToGround(orientation, position);
        //if the tyre doesn't touch the ground, the exerted force is zero
        if(groundDist >= this.getTyreRadius()){
            return new Vector();
        }
        //not so easy case:

        //first calculate the normal force:
        Vector verticalForce = this.getNormalForce(orientation, position, deltaTime, prevTyreDelta); //naming for consistency

        //then get the brake force
        Vector brakes = this.getBrakeForce(orientation, velocity, brakeForce);

        // the resulting force on the tyres
        return brakes.vectorSum(verticalForce);

    }

    /**
     * Calculates the netto moment on the drone
     * @param orientation the orientation of the drone
     * @param netForce the net force exerted on the drone
     * @return the net moment in the drone axis system
     */
    public Vector getNetMomentTyre(Vector orientation, Vector position,  Vector netForce){
        //first calculate the force arm
        Vector relPosAxleDrone = this.getTyrePosition();
        float currentTyreRadius = this.getTyreRadius() - this.calcRadiusDelta(orientation, position);
        Vector relPosTyreBottomDrone = relPosAxleDrone.vectorSum(new Vector(0f, currentTyreRadius, 0f)); //given in the drone axis system

        //all the forces apply to the bottom of the tyre
        //1. transform the net forces to the drone axis system
        Vector netForceDrone = PhysXEngine.worldOnDrone(netForce, orientation);

        return netForceDrone.crossProduct(relPosTyreBottomDrone);
    }

    /**
     * Calculates the brake force exerted on the wheels of the drone in the world axis system
     * @param orientation the orientation of the drone
     * @param velocity the velocity of the drone
     * @param brakeForce the brake force exerted on the wheels (abs value)
     * @return  zero vector if the cannot be a brake force exerted (@see canExertBrakeForce)
     *          the brakeForce set in the opposite direction of the velocity in the drone axis system
     */
    private Vector getBrakeForce(Vector orientation, Vector velocity, float brakeForce){
        //calculate the brake force in the world axis system
        //get the sign of the velocity component amongst the z-axis of the drone
        Vector velocityDrone = PhysXEngine.worldOnDrone(velocity, orientation);

        //check if we may brake
        if(canExertBrakeForce(velocityDrone)){
            return new Vector(); // if not, return zero vector
        }

        //get the sign
        float xVelSign = velocityDrone.getzValue();

        //if we may brake, set the brake force opposite to the sign of the velocity in the drone axis system
        Vector brakeForceDrone =  new Vector(0,0, - Math.abs(brakeForce)*xVelSign);

        //then transform the brake force to the world axis system
        //all the y-components of the brake force need to be set to zero (can be a result of the transformation)
        Vector tBrakeForceDrone= PhysXEngine.droneOnWorld(brakeForceDrone, orientation);
        return new Vector(tBrakeForceDrone.getxValue(), 0f, tBrakeForceDrone.getzValue());

    }

    /**
     * Checks if a brake force may be exerted, is only valid if the drone is not stationary
     * @param velocityDrone the velocity of the drone in drone axis system
     * @return true if and only if the velocity amongst the z-axis is nonzero
     * note: may add interval
     */
    private boolean canExertBrakeForce(Vector velocityDrone){
        return Math.abs(velocityDrone.getzValue()) > 0;
    }

    /**
     * Calculates the normal force exerted on the tyre, given in the world axis system
     * @return
     */
    protected Vector getNormalForce(Vector orientation, Vector position, float deltaTime, float prevTyreDelta){
        float tyreDelta = this.calcRadiusDelta(orientation, position);

        if(tyreDelta <= 0){
            return new Vector();
        }
        //calculate the force generated by the tyreslope
        float tyreSlope = this.getTyreSlope();
        float tyreSlopeForce = tyreSlope*tyreDelta;

        //dampSlope force calculation
        float dampSlope = this.getDampSlope();
        float deltaTyreDelta = tyreDelta - prevTyreDelta;
        float deltaDiff = deltaTyreDelta/deltaTime;
        float dampSlopeForce = dampSlope*deltaDiff;

        return new Vector(0f, tyreSlopeForce + dampSlopeForce, 0f);
    }



    /**
     * Calculates the absolute position of the tyre
     * @param orientation the orientation of the drone
     * @param position the position of the drone in the world axis system
     * @return the absolute postion of the tyre
     */
    private Vector getAbsolutePosition(Vector orientation, Vector position){
        //first get the position of the wheel
        Vector relTyrePosDrone = this.getTyrePosition();
        //then transform it to the world
        Vector relTyrePosWorld = PhysXEngine.droneOnWorld(relTyrePosDrone, orientation);
        //then get the absolute position
        return relTyrePosWorld.vectorSum(position);
    }

    /**
     * Calculates the radius delta, may return a negative delta, means that the tyre is not on the ground
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return the radius delta
     */
    protected float calcRadiusDelta(Vector orientation, Vector position){
        float groundDist = this.getTyreDistanceToGround(orientation, position);
        float tyreRad = this.getTyreRadius();

        return tyreRad - groundDist;
    }

    /**
     * Calculates the distance from the center of the tyre to the ground seen parallel from the orientation
     * of the tyre
     * @param orientation the orientation of the drone
     * @param position position of the drone
     * @return the distance to the ground
     */
    private float getTyreDistanceToGround(Vector orientation, Vector position){
        //calculate the position of the center of the tyre in the world axis system
        Vector tyrePos = this.getTyrePosition();
        //transform to world axis
        tyrePos = PhysXEngine.droneOnWorld(tyrePos, orientation);
        //calculate the abs pos:
        Vector worldTyrePos = tyrePos.vectorSum(position);
        //get the height of the center coord of the tyre
        float centerHeight = worldTyrePos.getyValue();
        //get the roll of the plane
        float roll = orientation.getzValue();

        //calculate the distance from the center of the tyre to the ground parallel to the tyre orientation
        return (float) (centerHeight/sin(roll));
    }

    /**
     * returns true if and only if the centre of the tyre goes below the y = 0 mark (the axle touches the ground, not so good)
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if the axle touches the ground
     */
    public boolean checkCrash(Vector orientation, Vector position){
        Vector relTyrePosDrone = this.getTyrePosition();
        Vector relTyrePosWorld = PhysXEngine.droneOnWorld(relTyrePosDrone, orientation);
        Vector absPos = position.vectorSum(relTyrePosWorld);
        return absPos.getyValue() <= 0;
    }

    public Vector getTyrePosition() {
        return tyrePosition;
    }

    public float getTyreRadius() {
        return tyreRadius;
    }

    public float getTyreSlope() {
        return tyreSlope;
    }

    public float getDampSlope() {
        return dampSlope;
    }

    public float getMaxBrake() {
        return maxBrake;
    }

    public float getMaxFricCoeff() {
        return maxFricCoeff;
    }


    private Vector tyrePosition;
    private float tyreRadius;
    private float tyreSlope;
    private float dampSlope;
    private float maxBrake;
    private float maxFricCoeff;



}
