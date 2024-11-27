package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.wilyworks.common.WilyWorks;

@Config
public class AutoDriveTo {

    public static double linearDriveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    public static double linearDriveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    public static double maxLinearSpeed = MecanumDrive.PARAMS.maxWheelVel;
    public static double rotationalDriveAccel = MecanumDrive.PARAMS.maxAngAccel;
    public static double rotationalDriveDeccel = -MecanumDrive.PARAMS.maxAngAccel;
    public static double maxRotationalSpeed = MecanumDrive.PARAMS.maxAngVel;
    //public static double currentVX = 6;
    //public static double currentVY = 0.1;

    MecanumDrive drive;
    Vector2d finalLinearVelocity;

    double radialSpeed;
    double lastRotationalSpeed;
    double tangentialSpeed;

    public AutoDriveTo(MecanumDrive drive) {
        this.drive = drive;
        lastTime = 0;
        finalLinearVelocity = new Vector2d(0, 0);
        radialSpeed = 0;
        lastRotationalSpeed = 0;
    }

    public void init(double goalX, double goalY) {
        Vector2d linearVector =  new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y);
        currentVelocity = drive.pose.times(drive.poseVelocity); //Convert from robot relative to field relative
        initTime = BaseOpModeFastBot.TIME.seconds();
        lastRotationalSpeed = currentVelocity.angVel;
        radialSpeed = findRadialSpeed(linearVector, currentVelocity.linearVel);
        tangentialSpeed = findTangentialSpeed(linearVector, currentVelocity.linearVel);
        lastTime = 0;
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

    private Vector2d radialVectorCalculations(Vector2d distVector, double deltaTime) {
        Vector2d radialVelocity;
        double distRemaining;

        //Distance to goal.
        distRemaining = Math.hypot(distVector.x, distVector.y);

        //If radial speed is going toward the goal or is stationary increase it, else decrease it
        if (radialSpeed >= 0)
            radialSpeed = radialSpeed + (linearDriveAccel * deltaTime);
        else
            radialSpeed = radialSpeed - (linearDriveDeccel * deltaTime);

        //Cap the speed at it's max speed or the speed it needs to be decelerating
        radialSpeed = Math.min(radialSpeed, maxLinearSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * linearDriveDeccel * distRemaining)));

        //set radial velocity's theta to be the same as dist vector's
        radialVelocity = distVector.div(distVector.norm());
        radialVelocity = radialVelocity.times(radialSpeed);
        return radialVelocity;
    }

    private Vector2d tangentialVectorCalculations(Vector2d distVector, double deltaTime) {
        Vector2d tangentialVelocity;

        //If tangential speed is positive, decrease until zero, else increase it until zero.
        if (tangentialSpeed > 0) {
            tangentialSpeed = tangentialSpeed + (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.min(tangentialSpeed, 0.0);
        } else if (tangentialSpeed < 0) {
            tangentialSpeed = tangentialSpeed - (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        }

        //rotate distance vector by 90 degrees.
        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);

        return tangentialVelocity;
    }

    public double confineToScope(double num) {
        while (num > Math.PI)
            num -= 2.0 * Math.PI;
        while (num < -Math.PI)
            num += 2.0 * Math.PI;
        return num;
    }

    public static double currentSpeed = 0;

    public double rotationalSpeedCalculations(double goalRotation, double deltaTime) {
        double rotRemaining;
        double differenceOfSpeeds;
        double rotationalSpeed;
        double rotationDirection;
        final double rotationalEpsilon = Math.toRadians(1);
        final double velocityEpsilon = 5.0;
        final double epsilon  = 3;

        rotRemaining = confineToScope(goalRotation - drive.pose.heading.log());

        rotationalSpeed = lastRotationalSpeed;

        if (Math.abs(rotRemaining) < rotationalEpsilon && Math.abs(rotationalSpeed) < velocityEpsilon)
            return 0;

        rotationDirection = Math.signum(rotRemaining);

        rotationalSpeed *= rotationDirection;
        if (rotationalSpeed >= 0) {
            rotationalSpeed += (rotationalDriveAccel * deltaTime);
        } else {
            rotationalSpeed -= (rotationalDriveDeccel * deltaTime);
        }

        rotationalSpeed = Math.min(rotationalSpeed, maxRotationalSpeed);
        rotationalSpeed = Math.min(rotationalSpeed, Math.sqrt(Math.abs(2.0 * rotationalDriveDeccel * rotRemaining)));
        rotationalSpeed = rotationalSpeed * rotationDirection;
        lastRotationalSpeed = rotationalSpeed;

        return rotationalSpeed;
    }

    private void drawVectors(double x, double y, Canvas canvas) {
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y,
                x + drive.pose.position.x, y + drive.pose.position.y);
    }

    PoseVelocity2d currentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    private double initTime = 0, lastTime = 0;
    boolean finished = false;

    public PoseVelocity2d motionProfileWithVector(Vector2d linearVector, double goalRotation, boolean hasInit, TelemetryPacket packet, double deltaT){
        Vector2d tangentialVelocity;
        Vector2d radialVelocity;
        double rotationalSpeed;

        final double velocityEpsilon = 7.5;
        final double distEpsilon = 1;

        Canvas canvas = packet.fieldOverlay();

        radialVelocity = radialVectorCalculations(linearVector, deltaT);
        tangentialVelocity = tangentialVectorCalculations(linearVector, deltaT);

        rotationalSpeed = rotationalSpeedCalculations(goalRotation, deltaT);

        finalLinearVelocity = radialVelocity.plus(tangentialVelocity);

        drawVectors(linearVector.x, linearVector.y, canvas);
        drawVectors(radialVelocity.x, radialVelocity.y, canvas);
        drawVectors(tangentialVelocity.x, tangentialVelocity.y, canvas);
        drawVectors(finalLinearVelocity.x, finalLinearVelocity.y, canvas);

        if (Math.hypot(finalLinearVelocity.x, finalLinearVelocity.y) < velocityEpsilon && Math.abs(linearVector.x) < distEpsilon && Math.abs(linearVector.y) < distEpsilon){
            finalLinearVelocity = new Vector2d(0, 0);
            packet.put("currentX", drive.pose.position.x);
            packet.put("currentY", drive.pose.position.y);
            packet.put("currentTheta", Math.toDegrees(drive.pose.heading.log()));
            packet.put("errorX", 48.0 - drive.pose.position.x);
            packet.put("errorY", -36.0 - drive.pose.position.y);
            packet.put("errorTheta", 180.0 - Math.toDegrees(drive.pose.heading.log()));
        }

        return new PoseVelocity2d(finalLinearVelocity, rotationalSpeed);
    }

    public void linearDriveTo(double goalX, double goalY, double goalRotation, boolean hasDriveToInit, TelemetryPacket packet, double deltaT) {
        PoseVelocity2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x,
                        goalY - drive.pose.position.y),
                goalRotation, hasDriveToInit, packet, deltaT);
        drive.setDrivePowers(null, motionProfileVel, null, null);
    }
}
