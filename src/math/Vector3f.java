/*
 * The MIT License (MIT)
 *
 * Copyright © 2015-2017, Heiko Brumme
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package math;

import java.nio.FloatBuffer;

import internal.Helper.Vector;

import static java.lang.Math.signum;

/**
 * This class represents a (x,y,z)-Vector. GLSL equivalent to vec3.
 *
 * @author Heiko Brumme
 */
public class Vector3f {

    public float x;
    public float y;
    public float z;

    /**
     * Creates a default 3-tuple vector with all values set to 0.
     */
    public Vector3f() {
        this.x = 0f;
        this.y = 0f;
        this.z = 0f;
    }

    /**
     * Creates a 3-tuple vector with specified values.
     *
     * @param x x value
     * @param y y value
     * @param z z value
     */
    public Vector3f(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Calculates the squared length of the vector.
     *
     * @return Squared length of this vector
     */
    public float lengthSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * Calculates the length of the vector.
     *
     * @return Length of this vector
     */
    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

    /**
     * Normalizes the vector.
     *
     * @return Normalized vector
     */
    public Vector3f normalize() {
        float length = length();
        return divide(length);
    }

    /**
     * Adds this vector to another vector.
     *
     * @param other The other vector
     *
     * @return Sum of this + other
     */
    public Vector3f add(Vector3f other) {
        float x = this.x + other.x;
        float y = this.y + other.y;
        float z = this.z + other.z;
        return new Vector3f(x, y, z);
    }

    /**
     * Negates this vector.
     *
     * @return Negated vector
     */
    public Vector3f negate() {
        return scale(-1f);
    }

    /**
     * Subtracts this vector from another vector.
     *
     * @param other The other vector
     *
     * @return Difference of this - other
     */
    public Vector3f subtract(Vector3f other) {
        return this.add(other.negate());
    }

    /**
     * Multiplies a vector by a scalar.
     *
     * @param scalar Scalar to multiply
     *
     * @return Scalar product of this * scalar
     */
    public Vector3f scale(float scalar) {
        float x = this.x * scalar;
        float y = this.y * scalar;
        float z = this.z * scalar;
        return new Vector3f(x, y, z);
    }
    
    public Vector3f scale(Vector3f scalar) {
        float x = this.x * scalar.x;
        float y = this.y * scalar.y;
        float z = this.z * scalar.z;
        return new Vector3f(x, y, z);
    }

    /**
     * Divides a vector by a scalar.
     *
     * @param scalar Scalar to multiply
     *
     * @return Scalar quotient of this / scalar
     */
    public Vector3f divide(float scalar) {
        return scale(1f / scalar);
    }

    /**
     * Calculates the dot product of this vector with another vector.
     *
     * @param other The other vector
     *
     * @return Dot product of this * other
     */
    public float dot(Vector3f other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    /**
     * Calculates the dot product of this vector with another vector.
     *
     * @param other The other vector
     *
     * @return Cross product of this x other
     */
    public Vector3f cross(Vector3f other) {
        float x = this.y * other.z - this.z * other.y;
        float y = this.z * other.x - this.x * other.z;
        float z = this.x * other.y - this.y * other.x;
        return new Vector3f(x, y, z);
    }

    /**
     * Calculates a linear interpolation between this vector with another
     * vector.
     *
     * @param other The other vector
     * @param alpha The alpha value, must be between 0.0 and 1.0
     *
     * @return Linear interpolated vector
     */
    public Vector3f lerp(Vector3f other, float alpha) {
        return this.scale(1f - alpha).add(other.scale(alpha));
    }

    /**
     * Stores the vector in a given Buffer.
     *
     * @param buffer The buffer to store the vector data
     */
    public void toBuffer(FloatBuffer buffer) {
        buffer.put(x).put(y).put(z);
        buffer.flip();
    }

    public static Vector toVector(Vector3f vec) {
    	return new Vector(vec.x, vec.y, vec.z);
    }
    
    public static Vector3f toVector3f(Vector vec) {
    	return new Vector3f(vec.getxValue(), vec.getyValue(), vec.getzValue());
    }

	public static Vector3f ArrayToVector3f(float[] array) {
		return new Vector3f(array[0], array[1], array[2]);
	}

    public float getAngleBetween(Vector3f other) throws NullPointerException{

        if(other == null){
            throw new NullPointerException();
        }

        // the numerator is a scalar product of the two vectors
        float numerator = this.dot(other);
        // the denominator is a product of the 2-norms of the vectors
        float denominator = this.length()*other.length();

        return (float)Math.acos(numerator/denominator);
    }

    public static float getHeadingAngle(Vector3f headingVector){
        //calculate the angle between the heading vector and the negative Z-axis to get the heading angle
        //for the drone
        Vector3f negZ = new Vector3f(0,0,-1);
        float angle = negZ.getAngleBetween(headingVector);
        //also get the sign from the y component, negative sign is right, positive is left (do negZ x HeadingVector)
        float sign =  signum(negZ.cross(headingVector).y);

        //check for NaN in the angle
        if(Float.isNaN(angle)){
            return 0;
        }

        return angle*sign;
    }
}
