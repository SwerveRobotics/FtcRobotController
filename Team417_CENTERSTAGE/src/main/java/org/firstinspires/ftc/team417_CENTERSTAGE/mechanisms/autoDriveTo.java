package org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Pose;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class autoDriveTo {

    public static double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    public static double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    public static double maxDriveVelocity = MecanumDrive.PARAMS.maxWheelVel;

    double initialDist;
    double lastTime;
    MecanumDrive drive;
    TelemetryPacket packet = new TelemetryPacket();

    public autoDriveTo(MecanumDrive drive) {
        this.drive = drive;
        lastTime = 0;
    }

    private void printVectorData(String vectorName, Vector2d vector) {
        packet.put(vectorName + " X", vector.x);
        packet.put(vectorName + " Y", vector.y);
        packet.put(vectorName + " Hypot", Math.hypot(vector.x, vector.y));
        packet.put(vectorName + " Theta1", Math.atan(vector.x / vector.y));
        packet.put(vectorName + " Theta2", Math.atan(vector.y / vector.x));
    }

    private Vector2d findRadialVelocity(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.y, goalVector.x);
        double currentTheta = Math.atan2(currentVector.y, currentVector.x);
        double differenceOfTheta = goalTheta - currentTheta;

        double radialVectorMagnitude = Math.hypot(currentVector.x, currentVector.y) * Math.cos(differenceOfTheta);
        return new Vector2d(Math.cos(goalTheta) * radialVectorMagnitude, Math.sin(goalTheta) * radialVectorMagnitude);
    }

    private Vector2d findTangentialVelocity(Vector2d goalVector, Vector2d currentVector) {
        double goalTheta = Math.atan2(goalVector.y, goalVector.x);
        double currentTheta = Math.atan2(currentVector.y, currentVector.x);
        double differenceOfTheta = goalTheta - currentTheta;

        double tangentialVectorMagnitude = Math.hypot(currentVector.x, currentVector.y) * Math.sin(differenceOfTheta);
        return new Vector2d(Math.cos(goalTheta - (Math.PI / 2.0)) * tangentialVectorMagnitude, Math.sin(goalTheta - (Math.PI / 2.0)) * tangentialVectorMagnitude);
    }

    private Vector2d radialVectorCalculations(Vector2d distVector, double timeSenseInit) {
        Vector2d radialVelocity;
        double radialSpeed;
        double distRemaining;

        distRemaining = Math.hypot(distVector.x, distVector.y);
        packet.put("distRemaining", distRemaining);

        radialVelocity = findRadialVelocity(distVector, currentVelocity);
        printVectorData("radialVelocity1", radialVelocity);

        if (radialVelocity.x > 0)
            radialSpeed = driveAccel * timeSenseInit + Math.hypot(radialVelocity.x, radialVelocity.y);
        else
            radialSpeed = -driveDeccel * timeSenseInit - Math.hypot(radialVelocity.x, radialVelocity.y);
        packet.put("radialSpeed1", radialSpeed);

        radialSpeed = Math.min(radialSpeed, maxDriveVelocity);
        packet.put("radialSpeed2", radialSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * driveDeccel * distRemaining)));
        packet.put("radialSpeed3", radialSpeed);

        radialVelocity = radialVelocity.div(radialVelocity.norm());
        printVectorData("radialVelocity2", radialVelocity);

        radialVelocity = radialVelocity.times(radialSpeed);
        printVectorData("radialVelocity3", radialVelocity);
        return radialVelocity;
    }

    private Vector2d tangentialVectorCalculations(Vector2d distVector, double timeSenseInit) {
        Vector2d tangentialVelocity;
        double tangentialSpeed;

        tangentialVelocity = findTangentialVelocity(distVector, currentVelocity);
        printVectorData("tangentialVelocity1", tangentialVelocity);

        tangentialSpeed = driveDeccel * timeSenseInit + Math.hypot(tangentialVelocity.x, tangentialVelocity.y);
        packet.put("tangentialSpeed1", tangentialSpeed);

        tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        packet.put("tangentialSpeed2", tangentialSpeed);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());
        printVectorData("tangentialVelocity2", tangentialVelocity);

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);
        printVectorData("tangentialVelocity3", tangentialVelocity);

        return tangentialVelocity;
    }

    Vector2d currentVelocity = new Vector2d (0, 0);
    private double initTime = 0;

    public Vector2d motionProfileWithVector(Vector2d distVector, boolean hasInit){
        Vector2d finalVelocity;
        Vector2d tangentialVelocity;
        Vector2d radialVelocity;
        final double epsilon = 0.5;
        double timeSenseInit;

        if (hasInit) {
            initTime = BaseOpMode.TIME.seconds();
            PoseVelocity2d currentCombinedVelocity = drive.poseVelocity;
            currentVelocity = drive.pose.times(currentCombinedVelocity.linearVel);
        }

        timeSenseInit = BaseOpMode.TIME.seconds() - initTime;

        radialVelocity = radialVectorCalculations(distVector, timeSenseInit);
        tangentialVelocity = tangentialVectorCalculations(distVector, timeSenseInit);

        finalVelocity = radialVelocity.plus(tangentialVelocity);

        printVectorData("distVector", distVector);
        printVectorData("finalVelocity", finalVelocity);
        printVectorData("tangentialVelocity", tangentialVelocity);
        printVectorData("radialVelocity", radialVelocity);
        printVectorData("currentVelocity", currentVelocity);
        packet.put("initTime", initTime);
        packet.put("hasInit", hasInit);
        packet.put("epsilon", epsilon);
        packet.put("timeSenseInit", timeSenseInit);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        if (Math.abs(distVector.x - drive.pose.position.x) < epsilon && Math.abs(distVector.y - drive.pose.position.y) < epsilon)
            return new Vector2d(0, 0);
        else
            return finalVelocity;
    }

    public double driveTo(double goalX, double goalY, boolean hasDriveToInit) {
        Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y), hasDriveToInit);
        //drive.setDrivePowers(null, new PoseVelocity2d(motionProfileVel, 0));
        return Math.hypot(motionProfileVel.x, motionProfileVel.y);
    }
}