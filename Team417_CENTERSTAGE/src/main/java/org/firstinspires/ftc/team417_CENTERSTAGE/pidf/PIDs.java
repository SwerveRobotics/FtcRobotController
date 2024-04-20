package org.firstinspires.ftc.team417_CENTERSTAGE.pidf;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;

public class PIDs {
    private double kP;
    private double kI;
    private double kD;
    private double cumulativeError;
    private double lastError = 0;
    private double lastSpeed = 0;

    public PIDs(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public Vector2d calculate(Vector2d path) {
        double deltaError;
        double error = path.norm();

        if (error == 0)
            return new Vector2d(0, 0);

        path = path.div(error);

        cumulativeError += error * Constants.DELTA_T;

        deltaError = error - lastError;

        lastError = error;

        return path.times(error * kP + cumulativeError * kI + deltaError * kD);
    }

    public Vector2d skew(Vector2d vel, double accel) {
        double speed = vel.norm();

        if (speed == 0)
            return new Vector2d(0, 0);
        vel = vel.div(speed);

        if (speed - lastSpeed > accel * Constants.DELTA_T)
            speed += accel * Constants.DELTA_T;
        lastSpeed = speed;

        return vel.times(speed);
    }
}
