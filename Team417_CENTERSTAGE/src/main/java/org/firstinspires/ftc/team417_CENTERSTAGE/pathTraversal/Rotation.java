package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public class Rotation {

    final double maxAccel = MecanumDrive.PARAMS.maxProfileAccel;
    final double maxDeccel = MecanumDrive.PARAMS.maxProfileAccel;
    final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    final MecanumDrive drive;

    public Rotation(MecanumDrive drive) {
        this.drive = drive;
    }

    private double confine

    public DPoint rotationalVel(double goalOrientation, double timeSinceInit) {
        Vector2d normVector;
        double speed;
        double travel;
        double distRemaining = goalOrientation - drive.pose.heading.toDouble();

        speed = maxAccel * timeSinceInit;
        speed = Math.min(speed, maxSpeed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * maxDeccel * distRemaining)));

        travel = speed / 100.0 * powerPCTToIn * Constants.LOOP_TIME - usedMovement;

        if (travel > distRemaining) {
            usedMovement = distRemaining;
            return null;
        }

        normVector = goalVector.div(goalVector.norm());
        pos = pos.plus(normVector.times(travel));

        usedMovement = 0;
        return pos;
    }
}
