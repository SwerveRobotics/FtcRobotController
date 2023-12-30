package org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;

@Config
public class AutoDriveTo {

    public static double driveAccel = MecanumDrive.PARAMS.maxProfileAccel;
    public static double driveDeccel = MecanumDrive.PARAMS.minProfileAccel;
    public static double maxDriveVelocity = MecanumDrive.PARAMS.maxWheelVel;
    public static double currentVX = 6;
    public static double currentVY = 0;

    MecanumDrive drive;
    TelemetryPacket packet = new TelemetryPacket();
    Vector2d finalVelocity;

    double radialSpeed;
    double differanceBetweenGoalVAndActualV;
    int findAverageIndex;
    Canvas canvas = packet.fieldOverlay();

    private double listOfItems[];

    public AutoDriveTo(MecanumDrive drive) {
        this.drive = drive;
        lastTime = 0;
        finalVelocity = new Vector2d(0, 0);
        radialSpeed = 0;
        differanceBetweenGoalVAndActualV = 0;
        findAverageIndex = 0;
    }

    private void printVectorData(String vectorName, Vector2d vector) {
        packet.put(vectorName + " X", vector.x);
        packet.put(vectorName + " Y", vector.y);
        packet.put(vectorName + " Hypot", Math.hypot(vector.x, vector.y));
        packet.put(vectorName + " Theta", Math.atan2(vector.y, vector.x));
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

    private double findAverage(double num) {
        listOfItems[findAverageIndex] = num;
        findAverageIndex += 1;
        if (findAverageIndex == listOfItems.length)
            findAverageIndex = 0;
        double sumOfItems = 0;

        for (double listOfItem : listOfItems) {
            sumOfItems += listOfItem;
        }
        return sumOfItems / listOfItems.length;
    }

    private Vector2d radialVectorCalculations(Vector2d distVector, double timeSinceInit, double deltaTime) {
        Vector2d radialVelocity;

        double distRemaining;
        double aveOfDifferences;

        distRemaining = Math.hypot(distVector.x, distVector.y);
        packet.put("distRemaining", distRemaining);

        radialVelocity = findRadialVelocity(distVector, currentVelocity.linearVel);
        printVectorData("radialVelocity1", radialVelocity);

        packet.put("actualSpeed", Math.hypot(radialVelocity.x, radialVelocity.y));

        aveOfDifferences = Math.abs(radialSpeed - Math.hypot(radialVelocity.x, radialVelocity.y));
        packet.put("aveOfDifferences", aveOfDifferences);

        if (radialVelocity.x * distVector.x >= 0)
            radialSpeed = Math.hypot(radialVelocity.x, radialVelocity.y) + (driveAccel * deltaTime) + aveOfDifferences;
        else
            radialSpeed = Math.hypot(radialVelocity.x, radialVelocity.y) - (driveAccel * deltaTime);
        packet.put("radialSpeed1", radialSpeed);

        radialSpeed = Math.min(radialSpeed, maxDriveVelocity);
        packet.put("radialSpeed2", radialSpeed);
        radialSpeed = Math.min(radialSpeed, Math.sqrt(Math.abs(2.0 * driveDeccel * distRemaining)));
        packet.put("radialSpeed3", radialSpeed);

        System.out.println(radialSpeed - Math.hypot(radialVelocity.x, radialVelocity.y));

        if (radialVelocity.x == 0 && radialVelocity.y == 0)
            radialVelocity = distVector.div(distVector.norm());
        else
            radialVelocity = radialVelocity.div(radialVelocity.norm());
        printVectorData("radialVelocity2", radialVelocity);

        radialVelocity = radialVelocity.times(radialSpeed);
        printVectorData("radialVelocity3", radialVelocity);
        return radialVelocity;
    }

    private Vector2d tangentialVectorCalculations(Vector2d distVector, double timeSinceInit) {
        Vector2d tangentialVelocity;
        double tangentialSpeed;

        tangentialVelocity = findTangentialVelocity(distVector, currentVelocity.linearVel);
        printVectorData("tangentialVelocity1", tangentialVelocity);

        tangentialSpeed = driveDeccel * timeSinceInit + Math.hypot(tangentialVelocity.x, tangentialVelocity.y);
        packet.put("tangentialSpeed1", tangentialSpeed);

        tangentialSpeed = Math.max(tangentialSpeed, 0.0);
        packet.put("tangentialSpeed2", tangentialSpeed);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());
        printVectorData("tangentialVelocity2", tangentialVelocity);

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);
        printVectorData("tangentialVelocity3", tangentialVelocity);

        return tangentialVelocity;
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

        //currentVelocity = drive.pose.times(drive.poseVelocity); //Convert from robot relative to field relative
        currentVelocity = drive.pose.times(new PoseVelocity2d(new Vector2d(currentVX, currentVY), 0));

        if (hasInit) {
            initTime = BaseOpMode.TIME.seconds();
            differanceBetweenGoalVAndActualV = 0;
            listOfItems = new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        }
        printVectorData("currentVelocity", currentVelocity.linearVel);
        packet.put("PoseVelocity",drive.poseVelocity);

        if (Math.hypot(finalVelocity.x, finalVelocity.y) < velocityEpsilon && Math.abs(distVector.x - drive.pose.position.x) < distEpsilon && Math.abs(distVector.y - drive.pose.position.y) < distEpsilon){
            return new Vector2d(0, 0);
        }

        timeSinceInit = BaseOpMode.TIME.seconds() - initTime;
        deltaTime = timeSinceInit - lastTime;

        radialVelocity = radialVectorCalculations(distVector, timeSinceInit, deltaTime);
        tangentialVelocity = tangentialVectorCalculations(distVector, deltaTime);

        finalVelocity = radialVelocity;//.plus(tangentialVelocity);

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

    public double driveTo(double goalX, double goalY, boolean hasDriveToInit) {
        //Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y), hasDriveToInit);
        Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(-24, 0.1), hasDriveToInit);
        //drive.setDrivePowers(null, new PoseVelocity2d(motionProfileVel, 0));
        return 0;//Math.hypot(motionProfileVel.x, motionProfileVel.y);
    }
}