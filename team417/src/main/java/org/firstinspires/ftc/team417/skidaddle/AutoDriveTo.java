package org.firstinspires.ftc.team417.skidaddle;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.roadrunner.HolonomicKinematics;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/** @noinspection SuspiciousNameCombination*/
@Config
public class AutoDriveTo {

    MecanumDrive drive;
    TelemetryPacket packet;
    Telemetry telemetry;
    Canvas canvas;

    //linear motion constants
    public final double linearDriveAccel = MecanumDrive.PARAMS.maxProfileAccel ; // Was maxProfileAccel * 1
    public final double linearDriveDeccel = MecanumDrive.PARAMS.minProfileAccel - 15; // Was minProfileAccel * 1
    public final double maxLinearSpeed = MecanumDrive.PARAMS.maxWheelVel; // Was maxWheelVel * 1
    public final double linearVelEpsilon = 2;
    public final double linearDistEpsilon = 0.5;

    //rotational motion constants
    public final double rotationalDriveAccel = MecanumDrive.PARAMS.maxAngAccel; // Was maxAngAccel * 1
    public final double rotationalDriveDeccel = -MecanumDrive.PARAMS.maxAngAccel * 0.75; // Was -maxAngAccel * 1
    public final double maxRotationalSpeed = MecanumDrive.PARAMS.maxAngVel; // Was maxAngVel * 1
    public final double rotationalVelEpsilon = Math.toRadians(3);
    public final double rotationalDistEpsilon = Math.toRadians(1);

    //target pose
    DPoint endPos;
    double endRot;

    //current pose
    double radialSpeed;
    double rotationalSpeed;
    double tangentialSpeed;
    DPoint targetPos;
    double targetRot;

    Vector2d lastLinearVel;
    double lastRotVel;
    PoseVelocity2d currentRobotVel;

    double safeDist;

    public AutoDriveTo(MecanumDrive drive) {
        this.drive = drive;
    }

    public void init(DPoint endPos, double endRot, PoseVelocity2d currentPoseVel, Telemetry telemetry) {
        PoseVelocity2d currentVelocity = drive.pose.times(currentPoseVel); //Convert from robot relative to field relative

        this.endPos = endPos;
        this.telemetry = telemetry;
        this.endRot = endRot;

        targetPos = DPoint.to(drive.pose.position);
        targetRot = drive.pose.heading.log();

        //find target change in position
        Vector2d deltaDist =  endPos.minus(targetPos).toVector2d();
        double deltaRot = confineToScope(endRot - targetRot);

        //Convert from velocity vectors to speed scalars relative to goal pose.
        rotationalSpeed = currentVelocity.angVel * Math.signum(deltaRot);
        radialSpeed = findParSpeed(deltaDist, currentVelocity.linearVel);
        tangentialSpeed = findPerpSpeed(deltaDist, currentVelocity.linearVel);

        lastLinearVel = currentPoseVel.linearVel;
        lastRotVel = currentPoseVel.angVel;
    }

    //returns the speed of the portion of the current vel vector that is parallel to the distance vector.
    private double findParSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.y, goalVector.x);
        double currentTheta = Math.atan2(currentVector.y, currentVector.x);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.cos(differenceOfTheta);
    }

    //returns the speed of the tangential portion of a vector.
    private double findPerpSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.y, goalVector.x);
        double currentTheta = Math.atan2(currentVector.y, currentVector.x);
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

        // Make sure the code doesn't go too far
        if (radialSpeed * deltaT > distRemaining) {
            radialSpeed = distRemaining / deltaT;
        }

        //Set radial velocity's theta to be the same as dist vector's
        // Account for the fact that distVector might be zero-length when dividing by its norm()
        if (distVector.norm() == 0) {
            radialVelocity = new Vector2d(0, 0);
        } else {
            radialVelocity = distVector.div(distVector.norm()).times(radialSpeed);
        }

        //If tangential speed is positive, decrease until zero, else increase it until zero.
        if (tangentialSpeed > 0) {
            tangentialSpeed += linearDriveDeccel * deltaT;
            tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        } else if (tangentialSpeed < 0) {
            tangentialSpeed -= linearDriveDeccel * deltaT;
            tangentialSpeed = Math.min(tangentialSpeed, 0.0);
        }

        //rotate distance vector by 90 degrees.
        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);

        // Account for the fact that tangentialVelocity might be zero-length when dividing by its norm()
        if (tangentialVelocity.norm() != 0) {
            tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm()).times(tangentialSpeed);
        }

        linearVelocity = tangentialVelocity.plus(radialVelocity);

        drawVectors(linearVelocity.x, linearVelocity.y, canvas);

        return linearVelocity;
    }

    public double rotationalVelocity(double deltaT) {
        double rotRemaining;
        double rotationalVel;

        rotRemaining = confineToScope(endRot - targetRot);

        if (rotationalSpeed >= 0) {
            rotationalSpeed += rotationalDriveAccel * deltaT;
        } else {
            rotationalSpeed -= rotationalDriveDeccel * deltaT;
        }

        rotationalSpeed = Math.min(rotationalSpeed, maxRotationalSpeed);
        rotationalSpeed = Math.min(rotationalSpeed, Math.sqrt(Math.abs(2.0 * rotationalDriveDeccel * rotRemaining)));

        // Make sure the code doesn't go too far
        if (rotationalSpeed * deltaT > Math.abs(rotRemaining)) {
            rotationalSpeed = Math.abs(rotRemaining) / deltaT;
        }

        rotationalVel = rotationalSpeed * Math.signum(rotRemaining);

        packet.put("rotationalVel", rotationalVel);

        return rotationalVel;
    }

    private void drawVectors(double x, double y, Canvas canvas) {
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y,
                x + drive.pose.position.x, y + drive.pose.position.y);
    }

    public boolean linearDriveTo(PoseVelocity2d currentRobotVel, double deltaT, TelemetryPacket packet, Canvas canvas) {
        Vector2d targetLinVel;
        double targetRotVel;
        this.canvas = canvas;
        this.packet = packet;
        this.currentRobotVel = currentRobotVel;

        DPoint deltaDist = endPos.minus(targetPos);

        targetLinVel = linearVelocity(deltaDist.toVector2d(), deltaT);
        targetRotVel = rotationalVelocity(deltaT);

        Vector2d targetLinAccel = targetLinVel.minus(lastLinearVel).div(deltaT);
        lastLinearVel = targetLinVel;

        double targetRotAccel = confineToScope(targetRotVel - lastRotVel) / deltaT;
        lastRotVel = targetRotVel;

        if (endPos.equals(drive.pose.position, 0.25)) {
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);
            drive.rightBack.setPower(0);
            drive.rightFront.setPower(0);

            return true;
        }

        double[] x = {targetPos.x, targetLinVel.x, targetLinAccel.x };
        double[] y = {targetPos.y, targetLinVel.y, targetLinAccel.y };
        double[] angular = {targetRot, targetRotVel, targetRotAccel};

        setDriveVel(x, y, angular);

        targetPos = targetPos.plus(targetLinVel.times(deltaT));
        targetRot += targetRotVel * deltaT;

        // Give the PID extra time to work and get as close as possible to the exact target location
        return false;
    }

    public void setDriveVel(double[] x, double[] y, double[] angular) {
        Pose2dDual<Time> txWorldTarget = new Pose2dDual<>(
                new Vector2dDual<>(new DualNum<>(x), new DualNum<>(y)),
                Rotation2dDual.exp(new DualNum<>(angular)));

        PoseVelocity2dDual<Time> command = new HolonomicController(
                MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.headingGain,
                MecanumDrive.PARAMS.axialVelGain, MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain
        ).compute(txWorldTarget, drive.pose, currentRobotVel);

        // Enlighten Wily Works as to where we should be:
        WilyWorks.runTo(txWorldTarget.value(), txWorldTarget.velocity().value());

        HolonomicKinematics.WheelVelocities<Time> wheelVels = drive.kinematics.inverse(command);

        double voltage = drive.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

        /*final MotorFeedforward feedforward = new MotorFeedforward(0,
                0,
                0);*/
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

        drive.leftFront.setPower(leftFrontPower);
        drive.leftBack.setPower(leftBackPower);
        drive.rightBack.setPower(rightBackPower);
        drive.rightFront.setPower(rightFrontPower);
    }
}