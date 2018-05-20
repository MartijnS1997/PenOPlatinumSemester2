package internal.Physics;

import internal.Helper.SquareMatrix;
import internal.Helper.Vector;

/**
 * Created by Martijn on 20/05/2018.
 * A class of fast transformations used to speed up the calculations in the testbed
 * --> if one instance is used the transformation matrices are saved such that if the orientation
 *     between the invocations doesn't change that the previous transformation matrix is re-used
 */
public class FastTransformations {

    public FastTransformations(){

    }

    /**
     * Transforms the given vector in the world axis system to a vector in the drone axis system
     * when the drone axis system has the given orientation
     * @param vector the vector to be transformed (in world axis system)
     * @param orientation the orientation of the drone axis system
     * @return the transformed vector (in drone axis system)
     */
    public Vector worldOnDrone(Vector vector, Vector orientation){
        //check if the saved transformation matrix is close enough
        Vector previousOrientation = this.getWorldOnDroneOrientation();

        if(orientation.relativeEquals(previousOrientation, EPSILON)){
            SquareMatrix worldOnDrone = this.getPreviousWorldOnDroneMatrix();
            return worldOnDrone.matrixVectorProduct(vector);
        }

        //if not, the transformation matrix has to be calculated
        SquareMatrix worldOnDrone = SquareMatrix.getWorldOnDroneTransformMatrix(orientation);
        //save the matrix
        this.setPreviousWorldOnDroneMatrix(worldOnDrone);
        //save the orientation
        this.setWorldOnDroneOrientation(orientation);

        //calculate the transformation
        return worldOnDrone.matrixVectorProduct(vector);
    }

    /** Transforms the given vector in the drone axis system to a vector in the world axis system
     * when the drone axis system has the given orientation
     * @param vector the vector to be transformed (in drone axis system)
     * @param orientation the orientation of the drone axis system
     * @return the transformed vector (in world axis system)
     */
    public Vector droneOnWorld(Vector vector, Vector orientation){
        //check if the saved transformation is close enough
        Vector previousOrientation = this.getDroneOnWorldOrientation();
        if(orientation.relativeEquals(previousOrientation, EPSILON)){
            SquareMatrix droneOnWorld = this.getPreviousDroneOnWorldMatrix();
            return droneOnWorld.matrixVectorProduct(vector);
        }

        //if not, the transformation matrix has to be calculated
        SquareMatrix droneOnWorld = SquareMatrix.getDroneOnWorldTransformMatrix(orientation);
        //save the matrix
        this.setPreviousDroneOnWorldMatrix(droneOnWorld);
        //save the orientation
        this.setDroneOnWorldOrientation(orientation);
        //calculate the transformation
        return droneOnWorld.matrixVectorProduct(vector);
    }

    /**
     * Getter for the orientation previously used to construct the transformation matrix for the drone on world
     * transformation
     * @return the orientation previously used to construct a transformation matrix
     */
    private Vector getDroneOnWorldOrientation() {
        return droneOnWorldOrientation;
    }

    /**
     * Setter for the orientation used to construct the drone on world transformation matrix
     * @param droneOnWorldOrientation the orientation used to construct the drone on world matrix
     */
    private void setDroneOnWorldOrientation(Vector droneOnWorldOrientation) {
        this.droneOnWorldOrientation = droneOnWorldOrientation;
    }

    /**
     * Getter for the world on drone orientation that the world on drone matrix currently saved is
     * configured for
     * @return the orientation the world on drone matrix is currently configured for
     */
    private Vector getWorldOnDroneOrientation() {
        return worldOnDroneOrientation;
    }

    /**
     * Setter for the world on drone orientation that the world on drone matrix is currenly configured for
     * @param worldOnDroneOrientation the orientation the world on drone matrix is configured for
     */
    private void setWorldOnDroneOrientation(Vector worldOnDroneOrientation) {
        this.worldOnDroneOrientation = worldOnDroneOrientation;
    }

    /**
     * Getter for the saved world on drone transformation matrix
     * @return the transformation matrix previously used to transform a vector in the world axis system to
     *         a vector in the drone axis system
     */
    private SquareMatrix getPreviousWorldOnDroneMatrix() {
        return previousWorldOnDroneMatrix;
    }

    /**
     * Setter for the world on drone matrix that can be used for the next call (saved work)
     * @param previousWorldOnDroneMatrix the transformation matrix used to project a vector in the world axis system
     *                                   to a vector in the drone axis system
     */
    private void setPreviousWorldOnDroneMatrix(SquareMatrix previousWorldOnDroneMatrix) {
        this.previousWorldOnDroneMatrix = previousWorldOnDroneMatrix;
    }

    /**
     * Getter for the drone on world matrix previously used (saved work)
     * @return the transformation matrix
     */
    private SquareMatrix getPreviousDroneOnWorldMatrix() {
        return previousDroneOnWorldMatrix;
    }

    /**
     * Setter for the drone on world matrix used to save work while calculating the transformations
     * @param previousDroneOnWorldMatrix the transformation matrix to save
     */
    private void setPreviousDroneOnWorldMatrix(SquareMatrix previousDroneOnWorldMatrix) {
        this.previousDroneOnWorldMatrix = previousDroneOnWorldMatrix;
    }

    /*
    Saved state
     */
    private Vector worldOnDroneOrientation;
    private SquareMatrix previousWorldOnDroneMatrix;

    private Vector droneOnWorldOrientation;
    private SquareMatrix previousDroneOnWorldMatrix;

    /*
    Constants
     */
    private final static float EPSILON = 1E-5f;
}
