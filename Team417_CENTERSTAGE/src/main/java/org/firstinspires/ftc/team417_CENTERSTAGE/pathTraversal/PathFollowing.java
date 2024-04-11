package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.ArrayList;

public class PathFollowing {

    private final double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    private final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    private final double LINE_APROX_EPSILON = 0.01;
    private final double LINEAR_PATH_VEL_EPSILON = 5; //Tune later.
    private final double LINEAR_PATH_DIST_EPSILON = 5; //Tune later.
    private Vector2d linearVel = new Vector2d(0, 0);
    private double initTime;
    private int pathIndex = 0;
    private boolean linearFinished = false;
    private boolean cubicFinished = false;

    MecanumDrive drive;
    Canvas canvas;
    public PathFollowing(MecanumDrive drive, Canvas canvas) {
        this.drive = drive;
        this.canvas = canvas;
    }

    public Vector2d linearPathFollowing(Vector2d goalVector, double timeSinceInit) {
        Vector2d normVector;
        double distRemaining;
        double speed;

        distRemaining = goalVector.norm();

        speed = timeSinceInit * driveAccel;
        speed = Math.min(maxSpeed, speed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * driveDeccel * distRemaining)));

        if (speed <= LINEAR_PATH_VEL_EPSILON && distRemaining <= LINEAR_PATH_DIST_EPSILON)
            linearFinished = true;

        normVector = goalVector.div(goalVector.norm());
        return normVector.times(speed);
    }

    public Vector2d linearPathFollowing(Vector2d goalVector, double timeSinceInit, double additionalDistRemaining) {
        Vector2d normVector;
        double distRemaining;
        double speed;

        distRemaining = goalVector.norm();

        speed = timeSinceInit * driveAccel;
        speed = Math.min(maxSpeed, speed);
        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * driveDeccel * (distRemaining + additionalDistRemaining))));

        if (distRemaining <= LINEAR_PATH_DIST_EPSILON) //broken please fix me.
            linearFinished = true;

        normVector = goalVector.div(goalVector.norm());
        return normVector.times(speed);
    }

    private double calcTotalDist(ArrayList<DPoint> points, int startingPoint) {
        double distSum = 0;

        for (int i = startingPoint; i < points.size() - 1; i++) {
            distSum += Math.hypot(points.get(i).x - points.get(i + 1).x, points.get(i).y - points.get(i + 1).y);
        }

        return distSum;
    }
    public Vector2d cubicPathFollowing(Bezier path, double timeSinceInit) {
        Vector2d linearVel;
        Vector2d nextVector;

        path.lineApproximation(LINE_APROX_EPSILON, new DPoint(drive.pose.position.x, drive.pose.position.y));
        path.graph(canvas);

        nextVector = new Vector2d(path.linearPoints.get(pathIndex).x - drive.pose.position.x,
                                  path.linearPoints.get(pathIndex).y - drive.pose.position.y);

        linearVel = linearPathFollowing(nextVector, timeSinceInit, calcTotalDist(path.linearPoints, pathIndex));

        if (linearFinished) {
            pathIndex++;
            linearFinished = false;
            return cubicPathFollowing(path,timeSinceInit);
        }

        if (pathIndex >= path.linearPoints.size())
            cubicFinished = true;

        return linearVel;
    }

    public boolean cubicDriveTo(Bezier path, boolean init) {
        double timeSinceInit;
        double currentTime = BaseOpMode.TIME.seconds();

        if (init) {
            linearVel = new Vector2d(0, 0);
            initTime = currentTime;
            pathIndex = 0;
            linearFinished = false;
            cubicFinished = false;
        }

        timeSinceInit = currentTime - initTime;

        linearVel = cubicPathFollowing(path, timeSinceInit);

        if (cubicFinished)
            return true;

        drive.setDrivePowers(new PoseVelocity2d(linearVel, 0));

        return false;
    }
}
