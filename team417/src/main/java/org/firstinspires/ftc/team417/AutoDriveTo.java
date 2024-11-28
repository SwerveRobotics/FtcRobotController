package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.HolonomicKinematics;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.team417.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.team417.roadrunner.messages.PoseMessage;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.wilyworks.common.WilyWorks;

/** @noinspection SuspiciousNameCombination*/
@Config
public class AutoDriveTo {

    MecanumDrive drive;
    TelemetryPacket packet;
    Canvas canvas;

    //linear motion constants
    public final double linearDriveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    public final double linearDriveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    public final double maxLinearSpeed = MecanumDrive.PARAMS.maxWheelVel;
    public final double linearVelEpsilon = 7.5;
    public final double linearDistEpsilon = 1;

    //rotational motion constants
    public final double rotationalDriveAccel = Math.PI;
    public final double rotationalDriveDeccel = -Math.PI;
    public final double maxRotationalSpeed = Math.PI;
    public final double rotationalVelEpsilon = Math.toRadians(1);
    public final double rotationalDistEpsilon = Math.toRadians(2.5);

    //target pose
    DPoint goal;
    double goalRotation;

    //current pose
    double radialSpeed;
    double rotationalSpeed;
    double tangentialSpeed;
    DPoint currentPos;
    double currentRot;

    public AutoDriveTo(MecanumDrive drive) {
        this.drive = drive;
    }

    public void init(DPoint goal, double goalRotation) {
        PoseVelocity2d currentVelocity = drive.pose.times(drive.poseVelocity); //Convert from robot relative to field relative

        this.goal = goal;
        this.goalRotation = goalRotation;

        currentPos = DPoint.to(drive.pose.position);
        currentRot = drive.pose.heading.log();

        //find target change in position
        Vector2d deltaDist =  goal.minus(currentPos).toVector2d();
        double deltaRot = confineToScope(goalRotation - currentRot);

        //Convert from velocity vectors to speed scalars relative to goal pose.
        rotationalSpeed = currentVelocity.angVel * Math.signum(deltaRot);
        radialSpeed = findParSpeed(deltaDist, currentVelocity.linearVel);
        tangentialSpeed = findPerpSpeed(deltaDist, currentVelocity.linearVel);
    }

    //returns the speed of the portion of the current vel vector that is parallel to the distance vector.
    private double findParSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.cos(differenceOfTheta);
    }

    //returns the speed of the tangential portion of a vector.
    private double findPerpSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.sin(differenceOfTheta);
    }

    public double confineToScope(double num) {
        while (num > Math.PI)
            num -= 2.0 * Math.PI;
        while (num < -Math.PI)
            num += 2.0 * Math.PI;
        return num;
    }

    public Vector2d linearVelocity(Vector2d distVector, double deltaT) {
        double distRemaining;

        Vector2d radialVelocity;
        Vector2d tangentialVelocity;
        Vector2d linearVelocity;

        
        //Distance to goal.
        distRemaining = distVector.norm();

        //If radial speed is going toward the goal or is stationary increase it, else decrease it
        if (radialSpeed >= 0)
            radialSpeed += linearDriveAccel * deltaT;
        else
            radialSpeed -= linearDriveDeccel * deltaT;

        //Cap the speed at it's max speed or the speed it needs to be decelerating
        radialSpeed = Math.min(radialSpeed, maxLinearSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * linearDriveDeccel * distRemaining)));

        //set radial velocity's theta to be the same as dist vector's
        radialVelocity = distVector.div(distVector.norm()).times(radialSpeed);


        //If tangential speed is positive, decrease until zero, else increase it until zero.
        if (tangentialSpeed > 0) {
            tangentialSpeed += (linearDriveDeccel * deltaT);
            tangentialSpeed = Math.min(tangentialSpeed, 0.0);
        } else if (tangentialSpeed < 0) {
            tangentialSpeed -= (linearDriveDeccel * deltaT);
            tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        }

        //rotate distance vector by 90 degrees.
        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm()).times(tangentialSpeed);


        linearVelocity = tangentialVelocity.plus(radialVelocity);

        drawVectors(linearVelocity.x, linearVelocity.y, canvas);

        if (linearVelocity.norm() < linearVelEpsilon && distVector.norm() < linearDistEpsilon)
            return new Vector2d(0, 0);

        return linearVelocity;
    }

    public double rotationalVelocity(double deltaT) {
        double rotRemaining;
        double rotationalVel;

        rotRemaining = confineToScope(goalRotation - currentRot);

        if (rotationalSpeed >= 0) {
            rotationalSpeed += rotationalDriveAccel * deltaT;
        } else {
            rotationalSpeed -= rotationalDriveDeccel * deltaT;
        }

        rotationalSpeed = Math.min(rotationalSpeed, maxRotationalSpeed);
        rotationalSpeed = Math.min(rotationalSpeed, Math.sqrt(Math.abs(2.0 * rotationalDriveDeccel * rotRemaining)));


        rotationalVel = rotationalSpeed * Math.signum(rotRemaining);

        packet.put("rotationalVel", rotationalVel);

        if (Math.abs(rotRemaining) < rotationalDistEpsilon && Math.abs(rotationalVel) < rotationalVelEpsilon)
            return 0;

        return rotationalVel;
    }

    private void drawVectors(double x, double y, Canvas canvas) {
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y,
                x + drive.pose.position.x, y + drive.pose.position.y);
    }

    public boolean linearDriveTo(double deltaT, TelemetryPacket packet, Canvas canvas) {
        Vector2d currentLinearVel;
        double currentRotVel;
        this.canvas = canvas;
        this.packet = packet;

        DPoint deltaDist = goal.minus(currentPos);

        currentLinearVel = linearVelocity(deltaDist.toVector2d(), deltaT);
        currentRotVel = rotationalVelocity(deltaT);
        PoseVelocity2d motionProfileVel = new PoseVelocity2d(currentLinearVel, rotationalVelocity(deltaT));

        currentPos = currentPos.plus(currentLinearVel.times(deltaT));
        currentRot += currentRotVel * deltaT;

        drive.setDrivePowers(null, null, null, motionProfileVel);

        return motionProfileVel.linearVel.x == 0 && motionProfileVel.linearVel.y == 0 && motionProfileVel.angVel == 0;
    }
}