package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public class PathFollowing {

    private final double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    //private final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    private final double maxSpeed = 0.4;
    private Vector2d linearVel = new Vector2d(0, 0);
    private double initTime;
    private int pathIndex = 1;
    private boolean linearFinished = false;
    private boolean cubicFinished = false;

    MecanumDrive drive;
    Canvas canvas;
    TelemetryPacket packet;
    Telemetry telemetry;
    public PathFollowing(MecanumDrive drive, Canvas canvas, TelemetryPacket packet, Telemetry telemetry) {
        this.drive = drive;
        this.canvas = canvas;
        this.packet = packet;
        this.telemetry = telemetry;
    }

    public Vector2d linearPathFollowing(DPoint goalPos, DPoint initialPos, double timeSinceInit, double additionalDistRemaining) {
        Vector2d goalVector;
        Vector2d normVector;
        double distRemaining;
        double speed;

        goalVector = new Vector2d(goalPos.x - drive.pose.position.x, goalPos.y - drive.pose.position.y);
        distRemaining = goalVector.norm();

        speed = timeSinceInit * driveAccel;
        speed = Math.min(maxSpeed, speed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * driveDeccel * (Math.abs(distRemaining) + additionalDistRemaining))));

        telemetry.addData("goalPos.x", goalPos.x);
        telemetry.addData("distRemaining", distRemaining);
        telemetry.addData("loop time", Constants.LOOP_TIME);
        telemetry.addData("DeltaT", Constants.DELTA_T);
        telemetry.addData("Travel", speed * Constants.LOOP_TIME / 1000);

        if (distRemaining < speed * Constants.LOOP_TIME / 1000) {
            linearFinished = true;
            return new Vector2d(0, 0);
        }

        normVector = goalVector.div(goalVector.norm());
        return normVector.times(speed);
    }
    public Vector2d cubicPathFollowing(Bezier path, double timeSinceInit) {
        Vector2d linearVel;
        DPoint currentPos = new DPoint(drive.pose.position.x, drive.pose.position.y);

        telemetry.addData("pathIndex", pathIndex);

        if (pathIndex >= path.linearPoints.size()) {
            cubicFinished = true;
            return new Vector2d(0, 0);
        }

        path.graph(canvas, currentPos, pathIndex);

        linearVel = linearPathFollowing(path.linearPoints.get(pathIndex), path.linearPoints.get(pathIndex - 1),
                                        timeSinceInit, path.length(pathIndex));

        if (linearFinished) {
            pathIndex++;
            linearFinished = false;
            cubicPathFollowing(path, timeSinceInit);
        }

        return linearVel;
    }

    public boolean cubicDriveTo(Bezier path, boolean init) {
        double timeSinceInit;
        double currentTime = Constants.TIME;

        if (init) {
            linearVel = new Vector2d(0, 0);
            initTime = currentTime;
            pathIndex = 1;
            linearFinished = false;
            cubicFinished = false;
        }

        timeSinceInit = currentTime - initTime;

        if (cubicFinished) {
            telemetry.addData("DONE!", 1);
            //return true;
        }

        linearVel = cubicPathFollowing(path, timeSinceInit);

        drive.setDrivePowers(new PoseVelocity2d(linearVel, 0));

        return false;
    }
}