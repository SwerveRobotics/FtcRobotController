package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.pidf.PIDs;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public class PathFollowing2 {
    private final double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    private final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    private final double powerPCTToIn = 19.653556 * Math.PI;
    private double usedMovement;
    private int pathIndex;
    private double initTime;
    private DPoint pos;
    private final MecanumDrive drive;
    private final Canvas canvas;
    private final Telemetry telemetry;

    public PathFollowing2(MecanumDrive drive, Canvas canvas, Telemetry telemetry) {
        this.drive = drive;
        this.canvas = canvas;
        this.telemetry = telemetry;
        pos = new DPoint(0, 0);
    }

    public DPoint linearPathFollowing(Vector2d goalVector, DPoint goalPos, double timeSinceInit, double additionalDistRemaining) {
        Vector2d normVector;
        double speed;
        double travel;
        double distRemaining = goalPos.toVector(pos).norm();

        speed = driveAccel * timeSinceInit;
        speed = Math.min(speed, maxSpeed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * driveDeccel * (additionalDistRemaining + distRemaining))));

        telemetry.addData("speed", speed);

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

    public DPoint cubicPathFollowing(Bezier path, double timeSinceInit) {
        Vector2d currentLine;

        telemetry.addData("PathIndex", pathIndex);
        telemetry.addData("path.linearPoints.size()", path.linearPoints.size());
        telemetry.addData("pos.x", pos.x);
        telemetry.addData("pos.y", pos.y);

        if (pathIndex >= path.linearPoints.size()) {
            return null;
        }

        path.graph(canvas, pos, pathIndex);

        currentLine = path.linearPoints.get(pathIndex).toVector(path.linearPoints.get(pathIndex - 1));

        if (linearPathFollowing(currentLine, path.linearPoints.get(pathIndex), timeSinceInit, path.length(pathIndex)) == null) {
            pos = path.linearPoints.get(pathIndex);
            pathIndex++;
            return cubicPathFollowing(path, timeSinceInit);
        }

        return pos;
    }

    public boolean cubicDriveTo(Bezier path, boolean init) {
        PIDs PID = new PIDs(1, 0, 0);
        DPoint goalPos;
        DPoint currentPos = new DPoint(drive.pose.position.x, drive.pose.position.y);
        Vector2d vel;
        double timeSinceInit;
        double currentTime = Constants.TIME;

        if (init) {
            usedMovement = 0;
            initTime = currentTime;
            pathIndex = 1;
            pos = currentPos;
        }

        timeSinceInit = currentTime - initTime;

        goalPos = cubicPathFollowing(path, timeSinceInit);

        if (goalPos == null) {
            telemetry.addData("DONE!", 1);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return true;
        }

        vel = PID.calculate(goalPos.toVector(currentPos));
        drive.setDrivePowers(new PoseVelocity2d(PID.skew(vel, driveAccel), 0));
        //drive.setDrivePowers(new PoseVelocity2d(goalPos.toVector(currentPos),0));

        return false;
    }
}
