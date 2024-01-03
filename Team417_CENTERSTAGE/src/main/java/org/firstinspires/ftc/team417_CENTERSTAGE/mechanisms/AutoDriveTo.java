package org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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
    TelemetryPacket packet = new TelemetryPacket();
    Vector2d finalVelocity;

    double lastRadialSpeed;
    double lastRotationalSpeed;
    double lastTangentialSpeed;
    Canvas canvas = packet.fieldOverlay();

    private double listOfItems[];

    public AutoDriveTo(MecanumDrive drive) {
        this.drive = drive;
        lastTime = 0;
        finalVelocity = new Vector2d(0, 0);
        lastRadialSpeed = 0;
        lastRotationalSpeed = 0;
    }

    private void printVectorData(String vectorName, Vector2d vector) {
        packet.put(vectorName + " X", vector.x);
        packet.put(vectorName + " Y", vector.y);
        packet.put(vectorName + " Hypot", Math.hypot(vector.x, vector.y));
        packet.put(vectorName + " Theta", Math.atan2(vector.y, vector.x));
    }

    private double findRadialSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.cos(differenceOfTheta);
    }

    private double findTangentialSpeed(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.x, goalVector.y);
        double currentTheta = Math.atan2(currentVector.x, currentVector.y);
        double differenceOfTheta = goalTheta - currentTheta;

        return Math.hypot(currentVector.x, currentVector.y) * Math.sin(differenceOfTheta);
        //return new Vector2d(Math.cos(goalTheta - (Math.PI / 2.0)) * tangentialVectorMagnitude, Math.sin(goalTheta - (Math.PI / 2.0)) * tangentialVectorMagnitude);
    }

    private Vector2d radialVectorCalculations(Vector2d distVector, double deltaTime) {
        Vector2d radialVelocity;

        double distRemaining;
        double differenceOfSpeeds;
        double radialSpeed;
        final double epsilon = 7.5;

        distRemaining = Math.hypot(distVector.x, distVector.y);
        packet.put("distRemaining", distRemaining);

        radialSpeed = findRadialSpeed(distVector, currentVelocity.linearVel);
        packet.put("radialSpeed1", radialSpeed);

        differenceOfSpeeds = lastRadialSpeed - radialSpeed;
        if (Math.abs(differenceOfSpeeds) > epsilon)
            differenceOfSpeeds = 0;
        packet.put("differenceOfSpeeds", differenceOfSpeeds);

        if (radialSpeed >= 0)
            radialSpeed = radialSpeed + (linearDriveAccel * deltaTime);
        else
            radialSpeed = radialSpeed - (linearDriveDeccel * deltaTime);
        packet.put("radialSpeed2", radialSpeed);

        radialSpeed = Math.min(radialSpeed, maxLinearSpeed);
        packet.put("radialSpeed3", radialSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * linearDriveDeccel * distRemaining))) + differenceOfSpeeds;
        packet.put("radialSpeed4", radialSpeed);
        lastRadialSpeed = radialSpeed;
        packet.put("radialSpeed4", radialSpeed);

        radialVelocity = distVector.div(distVector.norm());
        printVectorData("radialVelocity1", radialVelocity);

        radialVelocity = radialVelocity.times(radialSpeed);
        printVectorData("radialVelocity2", radialVelocity);
        return radialVelocity;
    }

    private Vector2d tangentialVectorCalculations(Vector2d distVector, double deltaTime) {
        Vector2d tangentialVelocity;
        double tangentialSpeed;
        double differenceOfSpeeds;
        final double epsilon  = 7.5;

        tangentialSpeed = findTangentialSpeed(distVector, currentVelocity.linearVel);
        packet.put("tangentialSpeed1", tangentialSpeed);

        differenceOfSpeeds = lastTangentialSpeed - tangentialSpeed;
        if (Math.abs(differenceOfSpeeds) > epsilon)
            differenceOfSpeeds = 0;
        packet.put("differenceOfSpeeds", differenceOfSpeeds);

        if (tangentialSpeed > 0) {
            tangentialSpeed = tangentialSpeed + (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.min(tangentialSpeed, 0.0) + differenceOfSpeeds;
        } else if (tangentialSpeed < 0) {
            tangentialSpeed = tangentialSpeed - (linearDriveDeccel * deltaTime);
            tangentialSpeed = Math.max(tangentialSpeed, 0.0) + differenceOfSpeeds;
        }
        packet.put("tangentialSpeed2", tangentialSpeed);

        tangentialVelocity = new Vector2d(distVector.y, distVector.x * -1);
        printVectorData("tangentialVelocity1", tangentialVelocity);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());
        printVectorData("tangentialVelocity2", tangentialVelocity);

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);
        printVectorData("tangentialVelocity3", tangentialVelocity);

        return tangentialVelocity;
    }

    public double confineToScope(double num) {
        while (num > Math.PI)
            num -= 2.0 * Math.PI;
        while (num < -Math.PI)
            num += 2.0 * Math.PI;
        return num;
    }

    public double findRotationalVel(double goalRotation, double deltaTime) {
        double rotRemaining;
        double differenceOfSpeeds;
        double rotationalSpeed;
        double rotationDirection;
        final double epsilon = 7.5;

        rotRemaining = confineToScope(goalRotation - drive.pose.heading.log());
        packet.put("rotRemaining", rotRemaining);

        rotationalSpeed = currentVelocity.angVel;
        packet.put("rotationalSpeed", rotationalSpeed);

        differenceOfSpeeds = lastRotationalSpeed - rotationalSpeed;
        if (Math.abs(differenceOfSpeeds) > epsilon)
            differenceOfSpeeds = 0;
        packet.put("differenceOfSpeeds", differenceOfSpeeds);

        if (rotRemaining <= 0) {
            rotationalSpeed = rotationalSpeed + (rotationalDriveAccel * deltaTime);
            packet.put("radialSpeed2", rotationalSpeed);
        } else {
            rotationalSpeed = rotationalSpeed - (rotationalDriveAccel * deltaTime);
            packet.put("radialSpeed2", rotationalSpeed);
        }

        //if (rotationalSpeed)


        rotationalSpeed = Math.min(rotationalSpeed, maxRotationalSpeed);
        packet.put("rotationalSpeed", rotationalSpeed);
        rotationalSpeed = Math.min(rotationalSpeed, Math.sqrt(Math.abs(2.0 * rotationalDriveDeccel * rotRemaining))) + differenceOfSpeeds;
        packet.put("rotationalSpeed", rotationalSpeed);
        lastRotationalSpeed = rotationalSpeed;
        packet.put("lastRotationalSpeed", lastRotationalSpeed);

        return rotationalSpeed;
    }

    PoseVelocity2d currentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    private double initTime = 0, lastTime = 0;
    boolean finished = false;

    public Vector2d motionProfileWithVector(Vector2d distVector, boolean hasInit){
        Vector2d tangentialVelocity;
        Vector2d radialVelocity;
        final double velocityEpsilon = 7.5;
        final double distEpsilon = 5;
        double timeSinceInit, deltaTime;

        currentVelocity = drive.pose.times(drive.poseVelocity); //Convert from robot relative to field relative
        //currentVelocity = drive.pose.times(new PoseVelocity2d(new Vector2d(currentVX, currentVY), 0));

        if (hasInit) {
            initTime = BaseOpMode.TIME.seconds();
            lastRadialSpeed = 0;
            lastTangentialSpeed = 0;
            lastRotationalSpeed = 0;
            lastTime = 0;
        }
        printVectorData("currentVelocity", currentVelocity.linearVel);
        packet.put("PoseVelocity",drive.poseVelocity);

        if (Math.hypot(finalVelocity.x, finalVelocity.y) < velocityEpsilon && Math.abs(distVector.x - drive.pose.position.x) < distEpsilon && Math.abs(distVector.y - drive.pose.position.y) < distEpsilon){
            return new Vector2d(0, 0);
        }

        timeSinceInit = BaseOpMode.TIME.seconds() - initTime;
        deltaTime = timeSinceInit - lastTime;

        radialVelocity = radialVectorCalculations(distVector, deltaTime);
        tangentialVelocity = tangentialVectorCalculations(distVector, deltaTime);

        finalVelocity = radialVelocity.plus(tangentialVelocity);

        printVectorData("distVector", distVector);
        printVectorData("finalVelocity", finalVelocity);
        printVectorData("tangentialVelocity", tangentialVelocity);
        printVectorData("radialVelocity", radialVelocity);
        packet.put("initTime", initTime);
        packet.put("hasInit", hasInit);
        packet.put("timeSenseInit", timeSinceInit);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        lastTime = timeSinceInit;

        return finalVelocity;
    }

    public void driveTo(double goalX, double goalY, boolean hasDriveToInit) {
        Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y), hasDriveToInit);
        //Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(-24, 0.1), hasDriveToInit);
        drive.setDrivePowers(null, new PoseVelocity2d(motionProfileVel, 0));
    }
}