package internal.Helper;

/**
 * Whole class Made by Martijn
 */

import math.Vector3f;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static java.lang.StrictMath.abs;
//import be.kuleuven.cs.som.annotate.*;

/**
 * A class of Immutable Vectors in 3D space
 * Each vector has a x, y and z coordinate
 * The size of each component is represented by a floating point number
 */

//@Value
public class Vector {
	
	/**
	 * Constructor for a vector
	 * @param xValue the x coordinate of the vector
	 * @param yValue the y coordinate of the vector
	 * @param zValue the z coordinate of the vector
	 * @author Martijn Sauwens
	 */
	public Vector(float xValue, float yValue, float zValue){
		this.xValue = xValue;
		this.yValue = yValue;
		this.zValue = zValue;
	}
	
	/**
	 * Default constructor for a vector, creates a null-vector
	 * @author Martijn Sauwens
	 */
	public Vector(){
		
		this(0,0,0);
	}

	/**
	 * Contructor for a vector given a length 3 array
	 * @param vectorArray a length 3 array contianing the values for the vector
	 * @author Martijn Sauwens
	 */
	public Vector(float[] vectorArray){
		if(!isValidArray(vectorArray))
			throw new IllegalArgumentException(INVALID_ARRAY);

		this.xValue = vectorArray[0];
		this.yValue = vectorArray[1];
		this.zValue = vectorArray[2];

	}

	public boolean isValidArray(float[] array){
		return array.length == 3;
	}
	
	
	/**
	 * returns a new vector containing the vector sum of this and other
	 * @param other the other vector in the sum
	 * @author Martijn Sauwens
	 */
	public Vector vectorSum(Vector other){
		float x_part = this.getxValue() + other.getxValue();
		float y_part = this.getyValue() + other.getyValue();
		float z_part = this.getzValue() + other.getzValue();
		
		return new Vector(x_part, y_part, z_part);
	}
	
	/**
	 * Returns the vector difference: this - other
	 * @param other the other vector
	 * @return the difference between two vector
	 * @author Martijn Sauwens
	 */
	public Vector vectorDifference(Vector other){
		Vector other_negative = other.scalarMult(-1);
		return this.vectorSum(other_negative);
		
	}
	
	/**
	 * returns a new vector containing the scalar multiplication of this and the scalar
	 * @Param scalar the scalar to re scale the vector
	 * @return new Vector(this.getxValue()*scalar, this.getyValue()*scalar, this.getzvalue()*scalar)
	 * @author Martijn Sauwens
	 */
	public Vector scalarMult(float scalar){
		float x_part = this.getxValue()*scalar;
		float y_part = this.getyValue()*scalar;
		float z_part = this.getzValue()*scalar;
		
		return new Vector(x_part, y_part, z_part);
	}
	
	/**
	 * @author Anthony Rathe
	 * For testing purposes
	 */
	public Vector normalizeToLength(float length) {
		Vector vector = this.normalizeVector();
		Vector newVector = new Vector(vector.getxValue()*length, vector.getyValue()*length, vector.getzValue()*length);
//		System.out.println(vector.getxValue() + " " + vector.getyValue() + " " + vector.getzValue() + "-->" + vector.getSize() + " : " + newVector.getSize());
		return newVector;
	}
	
	/**
	 * Calculates the scalar product of two vectors
	 * @param other the other vector
	 * 
	 * @return 	the scalar product of the two vectors
	 * 			with xi, yi, zi the components of each vector
	 * 			| result == x1*x2 + y1*y2 + z1*z2
	 * @author Martijn Sauwens
	 */
	public float scalarProduct(Vector other) throws NullPointerException{
		
		if(other == null){
			throw new NullPointerException();
		}
		
		float x_part = other.getxValue()*this.getxValue();
		float y_part = other.getyValue()*this.getyValue();
		float z_part = other.getzValue()*this.getzValue();
		
		return x_part + y_part + z_part;
	}
	
	/**
	 * returns a rescaled version of the given vector with 2-norm == 1
	 * @return |result == this.scalarMult(1/(this.getSize())
	 * @author Martijn Sauwens
	 */
	public Vector normalizeVector(){
		float size = this.getSize();
		return this.scalarMult(1/size);
	}
	
	/**
	 * returns the two norm of the vector
	 * @return see implementation
	 * @author Martijn Sauwens
	 */
	public float getSize(){
	
		return (float) Math.sqrt(this.scalarProduct(this));
	}
	
	/**
	 * Calculates enclosed angle between two vectors
	 * @param other the other vector
	 * @return the angle between the two given vectors
	 */
	public float getAngleBetween(Vector other) throws NullPointerException{
		
		if(other == null){
			throw new NullPointerException();
		}
		
		// the numerator is a scalar product of the two vectors
		float numerator = this.scalarProduct(other);
		// the denominator is a product of the 2-norms of the vectors
		float denominator = this.getSize()*other.getSize();
		
		double breuk = numerator/denominator;
		
		
		if (Math.abs(breuk) > 1f && Math.abs(breuk)*PRECISION < 1f)breuk = 1*Math.signum(breuk);
		float result = (float)Math.acos(breuk);
		return Float.isNaN(result) ? 0 : result;
	}


	/**
	 * Makes given vectors horizontal an calculates angle going from this to other.
	 * Positive angles mean a counterclockwise rotation (going from this to other)
	 * @param other
	 * @return
	 */
	public float getSignedAngleBetween(Vector other) {
		Vector origin = new Vector(0f,0f,0f);
		Vector thisVector = this.makeHorizontal();
		Vector otherVector = other.makeHorizontal();
		
		if (other.toTheRightOf(origin, thisVector))return -Math.abs(thisVector.getAngleBetween(otherVector));
		return Math.abs(thisVector.getAngleBetween(otherVector));
	}
	
	
	/**
	 * @author anthonyrathe
	 */
	public float distanceBetween(Vector other) throws NullPointerException{
		
		if(other == null){
			throw new NullPointerException();
		}
		else{
			return (float)Math.sqrt(Math.pow(this.getxValue()-other.getxValue(), 2) + Math.pow(this.getyValue()-other.getyValue(), 2) + Math.pow(this.getzValue()-other.getzValue(), 2));
		}
	}
	
	/**
	 * If, traversing along direction, going from this towards other, we would have to go backwards, then the sign of signDistanceBetween is negative.
	 * @author Anthony Rathe
	 */
	public float distanceBetween(Vector other, Vector direction){
		float distance = this.distanceBetween(other);
		if (direction.getAngleBetween(other.vectorDifference(this)) >= Math.PI)distance = -distance;
		return distance;
		
	}
	
	/**
	 * Sets y-value of vector to zero
	 * @author Anthony Rathe
	 */
	public Vector makeHorizontal() {
		return new Vector(this.getxValue(),0f,this.getzValue());
	}


	/**
	 * projects the vector onto the xz plane (ditches the y-component)
	 * @return the same vector as before but without the y-component
	 */
	public Vector projectOntoXZplane(){
		return new Vector (this.getxValue(), 0, this.getzValue());
	}

	/**
	 * Getter for the vector standing orthonormal on the instance it is invoked against
	 * the "right" orthogonal is viewed along the provided vector. Note that the orthogonal is calculated
	 * for the projection of the vector onto the xz-plane
	 * @return the normalized vector standing orthogonal on the projection of the provided instance and is
	 * 		   located to the right if viewed along the vector
	 */
	public Vector getRightOrthonormal(){
		Vector orthogonal =  new Vector(-this.getzValue(), 0, this.getxValue());
		return orthogonal.normalizeVector();
	}

	/**
	 * Returns the vector, rotated by a given angle around the y-axis. Positive angles result in counterclockwise rotation.
	 * @param angle
	 * @return
	 * @author Anthony Rathe
	 */
	public Vector rotateAroundYAxis(float angle) {
		return new Vector((float)(Math.cos(angle)*this.getxValue()+Math.sin(angle)*this.getzValue()),this.getyValue(),(float)(-Math.sin(angle)*this.getxValue()+Math.cos(angle)*this.getzValue()));
	}
	
	public boolean toTheRightOf(Vector other, Vector direction) {
		Vector thisVector = this.makeHorizontal();
		Vector otherVector = other.makeHorizontal();
		Vector normal = direction.rotateAroundYAxis((float)Math.PI/2);
		return normal.getAngleBetween(thisVector.vectorDifference(otherVector)) >= Math.PI/2;
	}
	
	/**
	 * calculates the cross product of two vectors and returns a new vector
	 * object containing the cross product results.
	 * @param other
	 * @return the cross product of the two vectors.
	 * 		| a = this, b = other
	 * 		| result == new Vector( ay*bz - az*by,
	 * 		|						az*bx - ax*bz,
	 * 		|						ax*by - ay*bx )
	 * @author Martijn Sauwens
	 */
	public Vector crossProduct(Vector other){
		
		if(other == null){
			throw new NullPointerException();
		}
		
		float x_part = this.getyValue()*other.getzValue() - this.getzValue()*other.getyValue();
		float y_part = this.getzValue()*other.getxValue() - this.getxValue()*other.getzValue();
		float z_part = this.getxValue()*other.getyValue() - this.getyValue()*other.getxValue();
		
		return new Vector(x_part, y_part, z_part);
	}
	
	/**
	 * Returns a string describing the Vector
	 * @author Martijn Sauwens
	 */
	@Override
	public String toString() {
		return "(" + xValue + ", " + yValue + ", " + zValue + ")";
	}

	
	/**
	 * Auto override of the  hash code
	 * @author Martijn Sauwens
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Float.floatToIntBits(xValue);
		result = prime * result + Float.floatToIntBits(yValue);
		result = prime * result + Float.floatToIntBits(zValue);
		return result;
	}



	/**
	 * checks if the given object is equal to the instance against which it is invoked
	 * @return |result = (obj == this)
	 * 
	 * @return true if the x, y and z values of both vectors are the same
	 * @author Martijn Sauwens
	 */
	@Override
	public boolean equals(Object obj) {
		if(obj == null){
			return false;
		}
		if(obj == this){
			return true;
		}
		if(obj instanceof Vector){
			Vector other = (Vector)obj;
			return other.getxValue() == this.getxValue() &&
				   other.getyValue() == this.getyValue() &&
				   other.getzValue() == this.getzValue();
		}else{
			return false;
		}
	}

	/**
	 * checks if the two vectors are equal within a given error range
	 * @param other the other vector
	 * @param range the range of the error
	 * @return true and only if all the components are equal within a given range (see floatAbsEquals)
	 * @author Martijn Sauwens
	 */
	public boolean rangeEquals(Vector other, float range){

		boolean xPart = floatAbsEquals(this.getxValue(), other.getxValue(), range);
		boolean yPart = floatAbsEquals(this.getyValue(), other.getyValue(), range);
		boolean zPart = floatAbsEquals(this.getzValue(), other.getzValue(), range);

		return xPart&&yPart&&zPart;
	}

	/**
	 * Checks if two given values are equal within a given range
	 * @param value1 the first value to compare
	 * @param value2 the second value to compare
	 * @param epsilon the range of the error
	 * @return true if and only if value1-value2 is in range of [-epsilon, epsilon]
	 * @author Martijn Sauwens
	 */
	private static boolean floatAbsEquals(float value1, float value2, float epsilon){
		float diff = value1 - value2;
		return  diff >= - epsilon && diff <= epsilon;
	}

	/**
	 * Checks if the provided vector equals to the error if all the components of the vector are
	 * relatively the same in a given range
	 * @param other the other vector
	 * @param relError the relative error to determine if they equal
	 * @return true if and only if all the components are within the provided relative error margin
	 */
	public boolean relativeEquals(Vector other, float relError){
		//first check if the other isn't a null
		if(other == null){
			return false;
		}

		//get all the components
		boolean xPart = relativeFloatEquals(this.getxValue(), other.getxValue(), relError);
		boolean yPart = relativeFloatEquals(this.getyValue(), other.getyValue(), relError);
		boolean zPart = relativeFloatEquals(this.getzValue(), other.getzValue(), relError);

		return xPart&&yPart&&zPart;
	}

	/**
	 * Returns true if and only if the relative error of f1 and f2 is smaller than the provided margin
	 * @param f1 the first float to check
	 * @param f2 the second float to check
	 * @param epsilon the relativer error
	 */
	private boolean relativeFloatEquals(float f1, float f2, float epsilon){
		//get the difference between f1 and f2
		float diff = f1 - f2;
		//get the relative error
		float relError = diff/f2;
		//check if it's within the acceptable range
		return abs(relError) < epsilon;
	}

	/**
	 * Rejects drift on the new vector for the given vector
	 * @param newVector contains the next value assigned to the vector variable
	 * @param driftRange the range for the allowed drift
	 * @return a vector containing the values of the newVector if and only if the new value
	 * 		   has sufficient change to the previous one (the one which it is invoked against)
	 * 		   see floatAbsEquals for the details on the range.
	 * @author Martijn Sauwens
	 */
	public Vector driftRejection(Vector newVector, float driftRange){
		float thisXPart = this.getxValue();
		float thisYPart = this.getyValue();
		float thisZPart = this.getzValue();

		float newXPart = newVector.getxValue();
		float newYPart = newVector.getyValue();
		float newZPart = newVector.getzValue();


		if(floatAbsEquals(newXPart, thisXPart, driftRange))
			 newXPart = thisXPart;
		if(floatAbsEquals(newYPart, thisYPart, driftRange))
			newYPart = thisYPart;
		if(floatAbsEquals(newZPart, thisZPart, driftRange))
			newZPart = thisZPart;

		return new Vector(newXPart, newYPart, newZPart);
	}

	/**
	 * calculates a orthogonal Projection of the given vector against the normal vector
	 * @author Martijn Sauwens
	 */
	public Vector orthogonalProjection(Vector normalVector){

		Vector normalizedNormal = normalVector.normalizeVector();
		float numerator = this.scalarProduct(normalizedNormal);
		float denominator = normalizedNormal.scalarProduct(normalizedNormal);

		Vector tempVector = normalizedNormal.scalarMult(numerator/denominator);

		return this.vectorDifference(tempVector);


	}


	/**
	 * selects the element at a given index of the vector
	 * @param index the index of the element to be obtained
	 * @return the element at position of the index
	 * @author Martijn Sauwens
	 */
	public float getElementAt(int index){
		switch(index) {
			case 0:
				return this.getxValue();
			case 1:
				return this.getyValue();
			case 2:
				return this.getzValue();
			default:
				throw new IndexOutOfBoundsException();
		}
	}

	/**
	 * Converts the given vector to an Vector3f object, used in the gui class
	 * @return a vector3f object equivalent to the instance it is invoked against
	 * @author Martijn Sauwens
	 */
	public Vector3f convertToVector3f(){
		float xPart = this.getxValue();
		float yPart = this.getyValue();
		float zPart = this.getzValue();

		return new Vector3f(xPart, yPart, zPart);
	}


	/**
	 * @author anthonyrathe
	 * @return a floating point array containing the vector values
	 */
	public float[] toArray(){
		float[] array = {getxValue(), getyValue(), getzValue()};
		return array;
	}

	/**
	 * @author anthonyrathe
	 * @return an integer array containing the values of the vector rounded to the nearest integer
	 */
	public int[] toIntArray(){
		int[] array = {Math.round(getxValue()), Math.round(getyValue()), Math.round(getzValue())};
		return array;
	}


	/**
	 * Makes a deep deepCopy of the vector currently selected and returns the deepCopy
	 * @return a deep deepCopy of the current vector
	 */
	public Vector deepCopy(){
		return new Vector(this.getxValue(), this.getyValue(), this.getzValue());
	}

	/**
	 * Projects the provided vector onto the instance against which it is invoked
	 * @param other the vector to project
	 * @return the other vector projected onto the vector which this method is invoked against
	 */
	@Deprecated
	public Vector projectOnVector(Vector other){
		//first get the numerator
		float numerator = this.scalarProduct(other);
		float denominator = this.scalarProduct(this);
		float coefficient = numerator/denominator;

		//then get multiply with the vector instance
		return this.scalarMult(coefficient);
	}

	/**
	 * Projects the instance onto the provided vector
	 * @param other the vector to project onto
	 * @return a vector containing the projection of the instance onto the other
	 */
	public Vector projectOn(Vector other){
		//first get the numerator
		float numerator = other.scalarProduct(this);
		float denominator = other.scalarProduct(other);
		float coefficient = numerator/denominator;
		return other.scalarMult(coefficient);
	}

	/**
	 * Checks if the given instance contains a negative component
	 * @return true if and only if one or more components of the vector are strictly negative
	 */
	public boolean hasNegativeComponent(){
		return this.getxValue() >= 0 && this.getyValue() >= 0 && this.getzValue() >= 0;
	}

	/**
	 * Gets the largest component of the vector
	 * @return the max component of the vector
	 */
	public Float getMaxComponent(){
		List<Float> vectorList = this.toList();
		return  Collections.max(vectorList);
	}

	/**
	 * Converts the given instance to a list of floats
	 * @return a list with at indices 0,1 and 2 the x, y and z components of the vector respectively
	 */
	public List<Float> toList(){
		List<Float> vectorList = new ArrayList<>();
		vectorList.add(this.getxValue());
		vectorList.add(this.getyValue());
		vectorList.add(this.getzValue());

		return vectorList;
	}


	/*
	Static methods
	 */

	/**
	 * Sums  all the vectors in the given array
	 * @param vectorArray the array containing all the vectors
	 * @return a vector containing the sum of all the vectors in the array
	 * @author Martijn Sauwens
	 */
	public static Vector sumVectorArray(Vector[] vectorArray){
		Vector tempVector = new Vector();
		for(Vector vector: vectorArray){

			tempVector = tempVector.vectorSum(vector);
		}

		return tempVector;
	}

	/**
	 * Sums  all the vectors in the given list
	 * @param vectorList the list containing all the vectors
	 * @return a vector containing the sum of all the vectors in the list
	 * @author Martijn Sauwens
	 */
	public static Vector sumVectorList(List<Vector> vectorList){
		Vector tempVector = new Vector();

		for(Vector vector: vectorList){
			tempVector = tempVector.vectorSum(vector);
		}

		return tempVector;
	}

	/**
	 * Converts a vector 3f vector to a standard immutable vector
	 * @param vector3f the vector to convert
	 * @return the converted vector 3f to Vector format
	 */
	public static Vector vector3fToVector(Vector3f vector3f){
		return new Vector(vector3f.x, vector3f.y, vector3f.z);
	}


	/*
	Getters and Setters
	 */

    /**
	 * Getter for the x Value of the vector
	 */
	public float getxValue() {
		return xValue;
	}

	/**
	 * Getter for the y Value of the vector
	 */
	public float getyValue() {
		return yValue;
	}

	/**
	 * Getter for the z value of the vector
	 */
	public float getzValue() {
		return zValue;
	}


	/**
	 * The variable containing the x-coordinate for the vector
	 */
	private float xValue;
	
	/**
	 * The variable containing the y-coordinate for the vector
	 */
	private float yValue;
	
	
	/**
	 * The variable containing the z-coordinate for the vector
	 */
	private float zValue;

	/*
	Error Messages
	 */

	public final static String INVALID_ARRAY = "The provided array is not of length 3";

	/*
	Constants
	 */
	public final static int VECTOR_SIZE = 3;
	public final static float PRECISION = 0.99999f;

}
