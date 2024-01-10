package org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

public class AutoDriveTo {

    //replace vars with vars from road runner.
    public static double driveAccel = 1;
    public static double maxDriveVelocity = 100;

    double lastTime;
    double initialDist;
    boolean hasInit;

    public AutoDriveTo() {
        hasInit = false;
    }
    private double findDecelerationVelocity(double acceleration, double distance) {
        return 2 * acceleration * distance;
    }

    private double normalizeX(double x, double y) {
        double hypot = Math.hypot(x, y);
        return x / hypot;
    }

    private double normalizeY(double x, double y) {
        double hypot = Math.hypot(x, y);
        return y / hypot;
    }

    Vector2d normDistVector = new Vector2d(0, 0);

    public Vector2d motionProfileWithVector(Vector2d distVector){
        double velocity;
        double timeSenseBoot = BaseOpMode.TIME.milliseconds(), currentTime;
        double distRemaining;
        Vector2d velocityVector;

        if (!hasInit) {
            initialDist = Math.hypot(distVector.x, distVector.y);

            normDistVector = distVector;
            normDistVector.sqrNorm();
            hasInit = true;
        }

        distRemaining = initialDist - Math.hypot(distVector.x, distVector.y);

        currentTime = timeSenseBoot - lastTime;

        velocity = driveAccel * currentTime;

        velocity = Math.min(velocity, maxDriveVelocity);

        velocity = Math.min(velocity, findDecelerationVelocity(driveAccel, distRemaining));

        velocityVector = new Vector2d(normDistVector.x * velocity, normDistVector.y * velocity);

        lastTime = currentTime;

        return velocityVector;
    }

    public void driveTo() {

    }
}