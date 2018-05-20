package internal.Physics;

import com.sun.org.apache.bcel.internal.generic.FASTORE;
import internal.Helper.SquareMatrix;
import internal.Helper.Vector;
import tests.SquareMatrixTest;

/**
 * A class of drone forces, used to avoid unnecessary re-computation of the forces
 * on the drone (for eg the moment calculations)
 * All the vectors are specified in the world axis system unless stated otherwise
 */
public class DroneForces{

    /**
     * Default constructor
     */
    public DroneForces() {

    }

    /**
     * Calculates the total force exerted on the drone
     * @return a vector containing the total force exerted on the drone
     */
    protected Vector getTotalForce(){
        //get the total force acting on the drone by the calculation calls (to prevent errors)
        //--> do not use the getNet...() methods! they are only for saving and should only be used
        //    by the methods that calculate the saved data
        Vector netAirborne = this.getTotalAirborneForces();
        Vector netWheel = this.getTotalWheelForces();

        return netAirborne.vectorSum(netWheel);
    }

    /**
     * Setter for the lift forces that are exerted on the drone
     * @param leftWingForce the force on the left wing of the drone
     * @param rightWingForce the force on the right wing of the drone
     * @param horizontalStabilizerForce the force on the horizontal stabilizer of the drone
     * @param verticalStabilizerForce the force on the vertical stabilizer of the drone
     */
    protected void setLiftForces(Vector leftWingForce, Vector rightWingForce, Vector horizontalStabilizerForce, Vector verticalStabilizerForce){
        this.leftWingForce = leftWingForce;
        this.rightWingForce = rightWingForce;
        this.horizontalStabilizerForce = horizontalStabilizerForce;
        this.verticalStabilizerForce = verticalStabilizerForce;
    }

    /**
     * Setter for the forces on the wheels of the drone
     * @param frontWheelForce the force on the wheel in the front of the drone
     * @param rearLeftWheelForce the force of the wheel on the rear of the drone (and to the left)
     * @param rearRightWheelForce the force on the wheel on the rear of the drone (and to the right)
     */
    protected void setWheelForces(Vector frontWheelForce, Vector rearLeftWheelForce, Vector rearRightWheelForce){
        this.frontWheelForce = frontWheelForce;
        this.rearLeftWheelForce = rearLeftWheelForce;
        this.rearRightWheelForce = rearRightWheelForce;
    }

    /**
     * Getter for the sum of all the airborne forces acting on the drone
     * this is the sum of the left wing force, right wing force, horizontal stabilizer, vertical stabilizer
     * thrust and gravity forces that are acting on the drone
     * @return the sum of all the forces acting on the drone
     */
    protected Vector getTotalAirborneForces(){

        //Before any computation is done, check if the result isn't already known
        Vector netAirborne = this.getNetAirborne();
        if(netAirborne != null){
            return netAirborne;
        }


        Vector leftWingForce = this.getLeftWingForce();
        Vector rightWingForce = this.getRightWingForce();
        Vector horizontalStabilizerForce = this.getHorizontalStabilizerForce();
        Vector verticalStabilizerForce = this.getVerticalStabilizerForce();
        Vector thrustForce = this.getThrust();
        Vector gravityForce = this.getGravity();

        Vector[] forceArray = new Vector[]{leftWingForce, rightWingForce, horizontalStabilizerForce, verticalStabilizerForce, gravityForce, thrustForce};
        netAirborne =  Vector.sumVectorArray(forceArray);
        this.setNetAirborne(netAirborne);

        return netAirborne;
    }

    /**
     * Getter for the total sum of the forces acting on the wheels of the drone
     * @return the forces acting on the wheels of the drone
     */
    protected Vector getTotalWheelForces(){

        //first check if the answer to the computation isn't already known
        Vector netChassis = this.getNetChassis();
        if(netChassis != null){
            return netChassis;
        }

        Vector frontTyreForce = this.getFrontWheelForce();
        Vector rearLeftTyreForce = this.getRearLeftWheelForce();
        Vector rearRightTyreForce = this.getRearRightWheelForce();

        Vector[] wheelForceArray = new Vector[]{frontTyreForce, rearLeftTyreForce, rearRightTyreForce};

        netChassis = Vector.sumVectorArray(wheelForceArray);
        this.setNetChassis(netChassis);

        return netChassis;

    }

    /**
     * Calculates the moment exerted on the left wing of the drone
     * @param leftWing the left wing to calculate the moment for
     * @param fastTransformations the transformation matrices used to transform a vector in the world axis system
     *                           to the drone axis system
     * @param orientation the orientation of the drone axis system
     * @return the moment exerted on the left wing of the drone
     */
    protected Vector getLeftWingMoment(WingPhysX leftWing, FastTransformations fastTransformations, Vector orientation){
        //first get the lift of the left wing (in world axis)
        Vector leftWingForce = this.getLeftWingForce();
        //transform to drone axis
        Vector leftWingForceDrone = fastTransformations.worldOnDrone(leftWingForce, orientation);

        //then get the relative position of the wing
        Vector leftWingPos =  leftWing.getRelativePosition();

        //calculate the moment
        return leftWingPos.crossProduct(leftWingForceDrone);
    }

    /**
     * Calculates the moment exerted on the right wing of the drone
     * @param rightWing the right wing to calculate the moment for
     * @param fastTransformations the transformation matrices used to transform a vector in the world axis system
     *                           to the drone axis system
     * @param orientation the orientation of the drone axis system
     * @return the moment exerted on the right wing of the drone
     */
    protected Vector getRightWingMoment(WingPhysX rightWing, FastTransformations fastTransformations, Vector orientation){
        //first get the lift of the right wing (in world axis)
        Vector rightWingForce = this.getRightWingForce();
        //transform to drone axis
        Vector rightWingForceDrone = fastTransformations.worldOnDrone(rightWingForce, orientation);

        //then get the relative position of the wing
        Vector rightWingPos =  rightWing.getRelativePosition();

        //calculate the moment
        return rightWingPos.crossProduct(rightWingForceDrone);
    }

    /**
     * Calculates the moment exerted on the horizontal stabilizer of the drone
     * @param horizontalStabilizer the horizontal stabilizer to calculate the moment for
     * @param fastTransformations the transformation matrices used to transform a vector in the world axis system
     *                           to the drone axis system
     * @return the moment exerted on the horizontal stabilizer of the drone
     */
    protected Vector getHorizontalStabilizerMoment(WingPhysX horizontalStabilizer, FastTransformations fastTransformations, Vector orientation){
        //first get the lift of the horizontal stabilizer (in world axis)
        Vector horizontalStabilizerForce = this.getHorizontalStabilizerForce();
        Vector horStabForceDrone = fastTransformations.worldOnDrone(horizontalStabilizerForce, orientation);

        //then get the relative position of the wing
        Vector horStabPos = horizontalStabilizer.getRelativePosition();

        //calculate the moment
        return horStabPos.crossProduct(horStabForceDrone);
    }



    /**
     * Calculates the moment exerted on the horizontal stabilizer of the drone
     * @param verticalStabilizer the horizontal stabilizer to calculate the moment for
     * @param fastTransformations the transformation matrices used to transform a vector in the world axis system
     *                           to the drone axis system
     * @param orientation the orientation of the drone axis system
     * @return the moment exerted on the horizontal stabilizer of the drone
     */
    protected Vector getVerticalStabilizerMoment(WingPhysX verticalStabilizer, FastTransformations fastTransformations, Vector orientation){
        //first get the lift of the horizontal stabilizer (in world axis)
        Vector verticalStabilizerForce = this.getVerticalStabilizerForce();
        Vector verStabForce = fastTransformations.worldOnDrone(verticalStabilizerForce, orientation);

        //then get the relative position of the wing
        Vector verStabPos = verticalStabilizer.getRelativePosition();

        //calculate the moment
        return verStabPos.crossProduct(verStabForce);
    }



    /**
     * Getter for the airborne forces exerted on the drone
     * --> is used to save the results of the calculations such that it won't spend calculation time
     */
    private Vector getNetAirborne() {
        return netAirborne;
    }

    /**
     * Setter for the net airborne forces
     * --> save the result
     */
    private void setNetAirborne(Vector netAirborne) {
        this.netAirborne = netAirborne;
    }

    /**
     * Getter for the net chassis forces, these are saved to avoid re-computation
     */
    private Vector getNetChassis() {
        return netChassis;
    }

    /**
     * Setter for the net chassis forces, save these after the first calculation to avoid
     * unnecessary re-computation
     */
    private void setNetChassis(Vector netChassis) {
        this.netChassis = netChassis;
    }

    protected Vector getThrust() {
        return thrust;
    }

    protected void setThrust(Vector thrust) {
        this.thrust = thrust;
    }

    protected Vector getGravity() {
        return gravity;
    }

    protected void setGravity(Vector gravity) {
        this.gravity = gravity;
    }

    protected Vector getLeftWingForce() {
        return leftWingForce;
    }

    protected void setLeftWingForce(Vector leftWingForce) {
        this.leftWingForce = leftWingForce;
    }

    protected Vector getRightWingForce() {
        return rightWingForce;
    }

    protected void setRightWingForce(Vector rightWingForce) {
        this.rightWingForce = rightWingForce;
    }

    protected Vector getHorizontalStabilizerForce() {
        return horizontalStabilizerForce;
    }

    protected void setHorizontalStabilizerForce(Vector horizontalStabilizerForce) {
        this.horizontalStabilizerForce = horizontalStabilizerForce;
    }

    protected Vector getVerticalStabilizerForce() {
        return verticalStabilizerForce;
    }

    protected void setVerticalStabilizerForce(Vector verticalStabilizerForce) {
        this.verticalStabilizerForce = verticalStabilizerForce;
    }

    protected Vector getFrontWheelForce() {
        return frontWheelForce;
    }

    protected void setFrontWheelForce(Vector frontWheelForce) {
        this.frontWheelForce = frontWheelForce;
    }

    protected Vector getRearLeftWheelForce() {
        return rearLeftWheelForce;
    }

    protected void setRearLeftWheelForce(Vector rearLeftWheelForce) {
        this.rearLeftWheelForce = rearLeftWheelForce;
    }

    protected Vector getRearRightWheelForce() {
        return rearRightWheelForce;
    }

    protected void setRearRightWheelForce(Vector rearRightWheelForce) {
        this.rearRightWheelForce = rearRightWheelForce;
    }

    /*
    Composite forces (that are saved after calculation)
     */
    private Vector netAirborne = null;
    private Vector netChassis = null;

    /*
    Individual forces acting on the drone
     */
    private Vector thrust;
    private Vector gravity;
    private Vector leftWingForce;
    private Vector rightWingForce;
    private Vector horizontalStabilizerForce;
    private Vector verticalStabilizerForce;
    private Vector frontWheelForce;
    private Vector rearLeftWheelForce;
    private Vector rearRightWheelForce;

}
