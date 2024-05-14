package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.pidf.PIDs;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public class PathFollowing {
    private final double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    private final double maxSpeed = MecanumDrive.PARAMS.maxWheelVel;
    private final double powerPCTToIn = 19.653556 * Math.PI;
    private final double DIST_EPSILON = 0.1;
    private double usedMovement;
    private int pathIndex;
    private double initTime;
    private DPoint pos;
    private final MecanumDrive drive;
    private final Canvas canvas;
    private final Telemetry telemetry;
    private PIDs PID;

    public PathFollowing(MecanumDrive drive, Canvas canvas, Telemetry telemetry) {
        this.drive = drive;
        this.canvas = canvas;
        this.telemetry = telemetry;
        pos = new DPoint(0, 0);
        PID = new PIDs(0, 0, 0);
    }

    public void init(DPoint startingPoint) {
        PID = new PIDs(1, 0, 0);
        usedMovement = 0;
        initTime = Constants.TIME;
        pathIndex = 1;
        pos = startingPoint;
    }

    Vector2d v = new Vector2d(0, 0);

    public DPoint linearPathFollowing(Vector2d goalVector, DPoint goalPos, double additionalDistRemaining) {
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
            usedMovement += distRemaining;
            return null;
        }

        normVector = goalVector.div(goalVector.norm());
        pos = pos.plus(normVector.times(travel));

        usedMovement = 0;
        return pos;
    }

    public boolean linearDriveTo(DPoint goal) {
        DPoint currentPos;
        Vector2d vel;

        currentPos = new DPoint(drive.pose.position.x, drive.pose.position.y);
        linearPathFollowing(goal.toVector(pos), goal, 0);

        vel = pos.toVector(currentPos);

        if (vel.norm() < DIST_EPSILON) {
            drive.setDrivePowers(null, new PoseVelocity2d(new Vector2d(0, 0), 0));
            return true;
        }

        //vel = PID.calculate(vel);
        //drive.setDrivePowers(new PoseVelocity2d(PID.skew(vel, driveAccel), 0));
        vel = pos.toVector(currentPos);
        vel = vel.times(1000.0 * Constants.LOOP_TIME);
        drive.setDrivePowers(null, new PoseVelocity2d(vel, 0));

        return false;
    }

    public DPoint cubicPathFollowing(Bezier path) {
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

        if (linearPathFollowing(currentLine, path.linearPoints.get(pathIndex), path.length(pathIndex)) == null) {
            pos = path.linearPoints.get(pathIndex);
            pathIndex++;
            return cubicPathFollowing(path);
        }

        return pos;
    }

    public boolean cubicDriveTo(Bezier path) {
        DPoint currentPos;
        Vector2d vel;

        currentPos = new DPoint(drive.pose.position.x, drive.pose.position.y);
        cubicPathFollowing(path);

        vel = pos.toVector(currentPos);

        if (vel.norm() < DIST_EPSILON) {
            drive.setDrivePowers(null, new PoseVelocity2d(new Vector2d(0, 0), 0));
            return true;
        }

        canvas.setStroke("#0000FF");
        canvas.strokeLine(currentPos.x, currentPos.y, currentPos.x + vel.x, currentPos.y + vel.y);

        //vel = PID.calculate(vel);
        //drive.setDrivePowers(new PoseVelocity2d(PID.skew(vel, driveAccel), 0));
        vel = pos.toVector(currentPos);
        vel = vel.times(1000.0 * Constants.LOOP_TIME);
        drive.setDrivePowers(null, new PoseVelocity2d(vel, 0));

        return false;
    }
}