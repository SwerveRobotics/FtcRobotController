package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import android.app.admin.SystemUpdatePolicy;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.ArrayList;

public class PathFollowing {
    MecanumDrive drive;
    //Tuned velocity constants of the robot.
    private final double linearDriveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    private final double linearDriveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    private final double maxLinearSpeed = MecanumDrive.PARAMS.maxWheelVel;

    //epsilon constants
    private final double linearVelocityEpsilon = 7.5;
    private final double linearDistEpsilon = 1;
    private final double pushSpeedEpsilon = 7.5;

    //vars that have to carry their values between loop iterations.
    private double lastRadialSpeed, lastTangentialSpeed;
    private double lastTime;
    private int currentLine;

    public PathFollowing(MecanumDrive drive) {
        this.drive = drive;
        lastTime = 0;
    }

    //Takes an angle and returns a identical angle that is from negative pi to pi.
    public double confineAngleToScope(double num) {
        while (num > Math.PI)
            num -= 2.0 * Math.PI;
        while (num < -Math.PI)
            num += 2.0 * Math.PI;
        return num;
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

    private Vector2d radialVectorCalculations(Vector2d distVector, double deltaTime, Vector2d currentVelocity, double distRemainingAfterLine) {
        Vector2d radialVelocity;
        double distRemaining;
        double differenceOfSpeeds;
        double radialSpeed;

        //Distance to goal.
        distRemaining = Math.hypot(distVector.x, distVector.y) + distRemainingAfterLine;

        //The magnitude of the radial portion of the robot's current velocity.
        radialSpeed = findRadialSpeed(distVector, currentVelocity);

        //Find the differance between the last speed and the actual speed to correct for the robot
        //having not finished accelerating.
        differenceOfSpeeds = lastRadialSpeed - radialSpeed;
        if (Math.abs(differenceOfSpeeds) > pushSpeedEpsilon || lastRadialSpeed == 0)
            differenceOfSpeeds = 0;

        //If radial speed is going toward the goal or is stationary increase it, else decrease it
        if (radialSpeed >= 0)
            radialSpeed = radialSpeed + (linearDriveAccel * deltaTime);
        else
            radialSpeed = radialSpeed - (linearDriveDeccel * deltaTime);

        //Cap the speed at it's max speed or the speed it needs to be decelerating
        radialSpeed = Math.min(radialSpeed, maxLinearSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * linearDriveDeccel * distRemaining))) + differenceOfSpeeds;
        lastRadialSpeed = radialSpeed;

        //set radial velocity's theta to be the same as dist vector's
        radialVelocity = distVector.div(distVector.norm());
        radialVelocity = radialVelocity.times(radialSpeed);
        return radialVelocity;
    }

    @SuppressWarnings("SuspiciousNameCombination")
    private Vector2d tangentialVectorCalculations(Vector2d distVector, double deltaTime, Vector2d currentVelocity) {
        Vector2d tangentialVelocity;
        double tangentialSpeed;
        double differenceOfSpeeds;

        //Set tangentialSpeed to the tangential portion of the current velocity
        tangentialSpeed = findTangentialSpeed(distVector, currentVelocity);

        //Find the difference between the last tangentialSpeed and the current tangential speed for it can be corrected.
        differenceOfSpeeds = lastTangentialSpeed - tangentialSpeed;
        //If difference of speeds is greater then epsilon, the robot was probably pushed so don't correct.
        if (Math.abs(differenceOfSpeeds) > pushSpeedEpsilon || lastTangentialSpeed == 0)
            differenceOfSpeeds = 0;

        //If tangential speed is positive, decrease until zero, else increase it until zero.
        if (tangentialSpeed > 0) {
            tangentialSpeed = tangentialSpeed + (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.min(tangentialSpeed, 0.0) + differenceOfSpeeds;
        } else if (tangentialSpeed < 0) {
            tangentialSpeed = tangentialSpeed - (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.max(tangentialSpeed, 0.0) + differenceOfSpeeds;
        }

        //rotate distance vector by 90 degrees.
        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);

        return tangentialVelocity;
    }

    //Drive the robot in a straight line.
    public boolean linearPathFollowing(DPoint goal, double deltaTime, double distRemainingAfterLine) {
        PoseVelocity2d currentVelocity;
        Vector2d tangentialVelocity;
        Vector2d radialVelocity;
        Vector2d distVector;
        Vector2d finalLinearVelocity;

        distVector = new Vector2d(goal.x - drive.pose.position.x, goal.y - drive.pose.position.y);

        currentVelocity = drive.pose.times(drive.poseVelocity); //Convert from robot relative to field relative

        radialVelocity = radialVectorCalculations(distVector, deltaTime, currentVelocity.linearVel, distRemainingAfterLine);
        tangentialVelocity = tangentialVectorCalculations(distVector, deltaTime, currentVelocity.linearVel);

        finalLinearVelocity = radialVelocity.plus(tangentialVelocity);

        if (Math.hypot(finalLinearVelocity.x, finalLinearVelocity.y) < linearVelocityEpsilon
                && Math.hypot(distVector.x, distVector.y) < linearDistEpsilon)
            return true;

        drive.setDrivePowers(null, new PoseVelocity2d(finalLinearVelocity, 0));
        return false;
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
    public ArrayList<BezierControl> flattenBezier(BezierControl controlPoints, double epsilon) {
        ArrayList <BezierControl> result = new ArrayList<>();

        DPoint[] p = controlPoints.toArray();
        DPoint[] l = findMidPoint(p);
        DPoint[] q = findMidPoint(l);
        DPoint c = findMidPoint(q)[0];

        double error = Math.max(Math.hypot(p[1].x - p[2].x, p[1].y - p[2].y) - Math.hypot(p[0].x - p[1].x, p[0].y - p[1].y),
                Math.hypot(p[2].x - p[3].x, p[2].y - p[3].y) - Math.hypot(p[1].x - p[2].x, p[1].y - p[2].y));

        if (Math.abs(error) < epsilon) {
            result.add(new BezierControl(p[0], p[1], p[2], p[3]));
            return result;
        } else {
            result.addAll(flattenBezier(new BezierControl(p[0], l[0], q[0], c), epsilon));
            result.addAll(flattenBezier(new BezierControl(c, q[1], l[2], p[3]), epsilon));
            return result;
        }
    }

    //Finds the total distance of a array of linear vectors. start = the first vector that is considered.
    public double findDistance(ArrayList<BezierControl> points, int start) {
        double totalDist = 0;

        for (int i = start; i < points.size(); i++) {
            totalDist += Math.hypot(points.get(i).p0.x - points.get(i).p3.x, points.get(i).p0.y - points.get(i).p3.y);
        }

        return totalDist;
    }

    //Moves the robot along a path described by a cubic bezier curve.
    //controlPoints = the control points of the bezier curve.
    public boolean cubicPathFollowing(BezierControl controlPoints, boolean hasInit) {
        DPoint nextPoint;
        double epsilon = 0.01;
        double distRemaining;
        double currentTime, deltaTime;

        //Approximate the bezier curve with straight lines.
        ArrayList<BezierControl> linearPoints = flattenBezier(controlPoints, epsilon);

        //reset values to zero and store init time.
        if (!hasInit) {
            lastRadialSpeed = 0;
            lastTangentialSpeed = 0;
            currentLine = 0;
            lastTime =  BaseOpMode.TIME.seconds();
        }

        currentTime = BaseOpMode.TIME.seconds();
        deltaTime = currentTime - lastTime;

        //If the robot has completed the last line in the list, return true to show that the movement is done.
        if (currentLine >= linearPoints.size())
            return true;

        //Find the distance remaining after the current line is completed.
        distRemaining = findDistance(linearPoints, currentLine + 1);
        //Store the next point the robot is driving in a line to.
        nextPoint = linearPoints.get(currentLine).p3;
        //Call the code to drive in a line, and if the code has completed, change the current line to the next line.
        if (linearPathFollowing(nextPoint, deltaTime, distRemaining))
            currentLine += 1;

        lastTime = currentTime;
        return false;
    }
}
