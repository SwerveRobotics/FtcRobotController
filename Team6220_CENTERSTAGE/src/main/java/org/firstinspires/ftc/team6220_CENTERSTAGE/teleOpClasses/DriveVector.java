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

    public double magnitude() {
        return Math.hypot(this.x, this.y);
    }
}
