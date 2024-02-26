package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.ArrayList;

public class PathFollowing {
    MecanumDrive drive;
    AveLoopTime aveDeltaTime;

    private final double linearDriveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double linearDriveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    private final double maxLinearSpeed = MecanumDrive.PARAMS.maxWheelVel;

    private final double lineCurveEpsilon = 0.01;

    private int currentLine;
    private double initTime;
    private Vector2d initVelocity;
    private ArrayList<DPoint> pointsOnPath = new ArrayList<>();
    private DPoint currentPoint;

    public PathFollowing(MecanumDrive drive, AveLoopTime aveDeltaTime) {
        this.drive = drive;
        this.aveDeltaTime = aveDeltaTime;
    }

    //returns the speed of the radial portion of a vector.
    private double findRadialSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.cos(differenceOfTheta);
    }

    //returns the speed of the tangential portion of a vector.
    private double findTangentialSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.sin(differenceOfTheta);
    }

    private Vector2d radialVelocityCalc(Vector2d distVector, double timeSinceInit, Vector2d initVelocity, double distRemaining) {
        Vector2d radialVelocity;
        double radialSpeed;

        //The magnitude of the radial portion of the robot's current velocity.
        radialSpeed = findRadialSpeed(distVector, initVelocity);

        //If radial speed is going toward the goal or is stationary increase it, else decrease it
        if (radialSpeed >= 0)
            radialSpeed = linearDriveAccel * timeSinceInit + radialSpeed;
        else
            radialSpeed = linearDriveDeccel * timeSinceInit + radialSpeed;

        //Cap the speed at it's max speed or the speed it needs to be decelerating
        radialSpeed = Math.min(radialSpeed, maxLinearSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * linearDriveDeccel * distRemaining)));

        //set radial velocity's theta to be the same as dist vector's
        radialVelocity = distVector.div(distVector.norm());
        radialVelocity = radialVelocity.times(radialSpeed);
        return radialVelocity;
    }

    private Vector2d tangentialVelocityCalc(Vector2d distVector, double timeSinceInit, Vector2d initVelocity) {
        Vector2d tangentialVelocity;
        double tangentialSpeed;

        //Set tangentialSpeed to the tangential portion of the current velocity
        tangentialSpeed = findTangentialSpeed(distVector, initVelocity);

        //If tangential speed is positive, decrease until zero, else increase it until zero.
        if (tangentialSpeed > 0) {
            tangentialSpeed = linearDriveDeccel * timeSinceInit + tangentialSpeed;
            tangentialSpeed = Math.min(tangentialSpeed, 0.0);
        } else if (tangentialSpeed < 0) {
            tangentialSpeed = -linearDriveDeccel * timeSinceInit + tangentialSpeed;
            tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        }

        //rotate distance vector by 90 degrees.
        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);

        return tangentialVelocity;
    }

    //Finds the total distance of a array of linear vectors. start = the first vector that is considered.
    public double findDistance(ArrayList<DPoint> points, int start) {
        double totalDist = 0;

        for (int i = start; i < points.size(); i++) {
            totalDist += Math.hypot(points.get(i).x - points.get(i + 1).x, points.get(i).y - points.get(i + 1).y);
        }

        return totalDist;
    }

    //returns the midpoints between and array of points as an array of points.
    private DPoint[] findMidPoint(DPoint[] points) {
        DPoint[] midPoints = new DPoint[points.length - 1];

        for (int i = 0; i < points.length - 1; i++) {
            midPoints[i] = new DPoint((points[i].x + points[i + 1].x) / 2.0, (points[i].y + points[i + 1].y) / 2.0);
        }

        return midPoints;
    }

    //Turns an bezier curve into an array of smaller bezier curves that resemble linear vectors.
    //epsilon = the acceptable differance between a bezier curve and a linear vector in inches.
    public ArrayList<DPoint> flattenBezier(BezierControl controlPoints, ArrayList<DPoint> resultPoints, double epsilon) {
        DPoint[] p = controlPoints.toArray();
        DPoint[] l = findMidPoint(p);
        DPoint[] q = findMidPoint(l);
        DPoint c = findMidPoint(q)[0];

        double error = Math.max(Math.hypot(2 * p[1].x - p[2].x - p[0].x, 2 * p[1].y - p[2].y - p[0].y),
                Math.hypot(2 * p[2].x - p[3].x - p[1].x, 2 * p[2].y - p[3].y - p[1].y));

        if (error < epsilon) {
            resultPoints.add(p[3]);
        } else {
            resultPoints = flattenBezier(new BezierControl(p[0], l[0], q[0], c), resultPoints, epsilon);
            resultPoints = flattenBezier(new BezierControl(c, q[1], l[2], p[3]), resultPoints, epsilon);
        }

        return resultPoints;
    }

    private DPoint velToPoint(Vector2d velocity, DPoint currentPoint) {
        return new DPoint(currentPoint.x + velocity.x * aveDeltaTime.get(),
                currentPoint.y + velocity.y * aveDeltaTime.get());
    }

    double lastTime = 0;

    public boolean cubicPathFollowing(BezierControl controlPoints, boolean hasInit) {
        Vector2d currentDist;
        Vector2d radialVelocity;
        Vector2d tangentialVelocity;
        Vector2d totalVelocity;
        double distRemaining;
        double timeSinceInit;
        double currentTime;

        if (!hasInit) {
            initTime = BaseOpMode.TIME.seconds();
            initVelocity = drive.pose.times(drive.poseVelocity).linearVel;
            currentLine = 0;
            lastTime = 0;
            currentPoint = new DPoint(drive.pose.position.x, drive.pose.position.y);

            pointsOnPath.add(controlPoints.p0);
            pointsOnPath = flattenBezier(controlPoints, pointsOnPath, lineCurveEpsilon);
        }

        if (currentLine >= pointsOnPath.size())
            return true;

        timeSinceInit = initTime - BaseOpMode.TIME.seconds();

        currentDist = new Vector2d(pointsOnPath.get(currentLine).x - drive.pose.position.x,
                                   pointsOnPath.get(currentLine).y - drive.pose.position.y);

        distRemaining = currentDist.norm() + findDistance(pointsOnPath, currentLine);

        radialVelocity = radialVelocityCalc(currentDist, timeSinceInit, initVelocity, distRemaining);

        if (radialVelocity.norm() > currentDist.norm()) {
            currentLine += 1;
            return cubicPathFollowing(controlPoints, false);
        }

        tangentialVelocity = tangentialVelocityCalc(currentDist, timeSinceInit, initVelocity);
        totalVelocity = radialVelocity.plus(tangentialVelocity);

        drive.setDrivePowers(null, new PoseVelocity2d(totalVelocity, 0));

        currentTime = BaseOpMode.TIME.seconds();

        currentPoint = currentPoint.plus(new DPoint(currentPoint.x + totalVelocity.x * (currentTime - lastTime),
                                                    currentPoint.y + totalVelocity.y * (currentTime - lastTime)));

        lastTime = currentTime;

        return false;
    }
}
