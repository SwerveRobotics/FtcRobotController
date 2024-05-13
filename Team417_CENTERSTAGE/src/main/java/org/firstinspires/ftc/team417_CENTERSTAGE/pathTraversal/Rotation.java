package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.pidf.PIDs;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public class Rotation {

    final double maxAccel = MecanumDrive.PARAMS.maxProfileAccel;
    final double maxDeccel = MecanumDrive.PARAMS.maxProfileAccel;
    final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    private final double powerPCTToRadians = 19.653556 * Math.PI / 9.0; //divide by dist to center of rotation.
    private double usedMovement;
    final MecanumDrive drive;

    private double orientation = 0;

    public Rotation(MecanumDrive drive) {
        this.drive = drive;
    }

    private double confine(double num) {
        while(num > Math.PI)
            num -= Math.PI * 2;
        while(num < -Math.PI)
            num += Math.PI * 2;

        return num;
    }

    public double orientation(double goalOrientation, double timeSinceInit) {
        Vector2d normVector;
        double speed;
        double travel;
        double distRemaining = goalOrientation - drive.pose.heading.toDouble();

        distRemaining = confine(distRemaining);

        speed = maxAccel * timeSinceInit;
        speed = Math.min(speed, maxSpeed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * maxDeccel * Math.abs(distRemaining))));

        travel = speed / 100.0 * powerPCTToRadians * Constants.LOOP_TIME - usedMovement;

        if (travel > distRemaining) {
            usedMovement = distRemaining;
            return 1000;
        }

        speed = speed * distRemaining / Math.abs(distRemaining);
        orientation += speed;

        usedMovement = 0;
        return orientation;
    }

    /*public double rotationalVel(double goalOrientation, double timeSinceInit) {
        double orientation = orientation(goalOrientation, timeSinceInit);
        double travel = confine(orientation - drive.pose.heading.toDouble());

        PIDs PID = new PIDs(1, 0, 0);

        tavel = PID.calculate(vel);

        return confine();
    }*/
}
