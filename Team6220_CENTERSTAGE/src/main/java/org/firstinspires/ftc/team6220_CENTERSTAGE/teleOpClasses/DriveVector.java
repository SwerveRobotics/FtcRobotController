package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.roadrunner.Vector2d;

/**
 * An object that represents the x and y components of the
 * driving stick, alongside some extra utility.
 */
public class DriveVector {

    public double x;
    public double y;

    public DriveVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * a shortcut for setting both the x and y components of the vector
     * @param x
     * @param y
     */
    public void setXY(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * rotates this vector by an angle in degrees; negative = clockwise
     * @param rotationDegrees the number of degrees to rotate it by
     */
    public void rotate(double rotationDegrees) {
        double angleRadians = Math.toRadians(rotationDegrees);
        this.setXY(
            this.x * Math.cos(angleRadians) - this.y * Math.sin(angleRadians),
            this.x * Math.sin(angleRadians) + this.y * Math.cos(angleRadians)
        );
    }

    /**
     * converts this vector into roadrunner's vector object implementation
     * @return a new Vector2d retaining the x and y components
     */
    public Vector2d toVector2d() {
        return new Vector2d(this.x, this.y);
    }

    /**
     * gets the magnitude of the vector
     */
    public double magnitude() {
        return Math.hypot(this.x, this.y);
    }

    /**
     * multiplies both x and y component by a value
     * @param a multiplier value
     */
    public void times(double a) {
        this.setXY(
            this.x * a,
            this.y * a
        );
    }
    /**
     * multiplies x and y components by different values
     * @param a multiplier value for x
     * @param b multiplier value for y
     */
    public void times(double a, double b) {
        this.setXY(
                this.x * a,
                this.y * b
        );
    }

    /**
     * divides both x and y component by a value
     * @param a divisor value
     */
    public void divide(double a) {
        this.setXY(
                this.x / a,
                this.y / a
        );
    }

    /**
     * limits the vector's magnitude to 1 or under
     */
    public void limit() {
        // normalize to length 1 if magnitude is greater than 1
        if (this.magnitude() > 1) {
            this.divide(this.magnitude());
        }
    }
}
