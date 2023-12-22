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

    Vector2d normDistVector = new Vector2d(0, 0);
    TelemetryPacket packet = new TelemetryPacket();
    private double initTime = 0;

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

    Vector2d initialVelocity;
    Vector2d currentVelocity = new Vector2d (0, 0);

    public Vector2d motionProfileWithVector(Vector2d distVector, boolean hasInit){
        Vector2d finalVelocityVector;
        Vector2d radialVelocity;
        Vector2d tangentialVelocity;
        double timeSenseInit;
        double distRemaining;
        double speed;
        double epsilon = 0.5;
        double radialSpeed, tangentialSpeed;
        printVectorData("distVector", distVector);

        if (hasInit) {
            initTime = BaseOpMode.TIME.seconds();
            PoseVelocity2d currentCombinedVelocity = drive.poseVelocity;
            currentVelocity = drive.pose.times(currentCombinedVelocity.linearVel);
        }
        packet.put("initTime", initTime);

        timeSenseInit = BaseOpMode.TIME.seconds() - initTime;
        packet.put("timeSenseInit", timeSenseInit);

        radialVelocity = findRadialVelocity(distVector, currentVelocity);
        printVectorData("radialVelocity", radialVelocity);

        if (radialVelocity.x > 0) {
            radialSpeed = Math.hypot(radialVelocity.x, radialVelocity.y);
            speed = driveAccel * timeSenseInit + radialSpeed;
        } else {
            radialSpeed = -Math.hypot(radialVelocity.x, radialVelocity.y);
            speed = -driveDeccel * timeSenseInit + radialSpeed;
        }
        packet.put("radialSpeed", radialSpeed);
        packet.put("accelVelocity", speed);

        speed = Math.min(speed, maxDriveVelocity);
        packet.put("maintainVelocity", speed);

        distRemaining = Math.hypot(distVector.x, distVector.y);
        packet.put("distRemaining", distRemaining);

        speed = Math.min(speed, Math.sqrt(Math.abs(2.0 * driveDeccel * distRemaining)));
        packet.put("decelVelocity", speed);
        packet.put("velocity", speed);

        tangentialVelocity = findTangentialVelocity(distVector, currentVelocity);
        printVectorData("tangentialVelocity1", tangentialVelocity);

        tangentialSpeed = Math.hypot(tangentialVelocity.x, tangentialVelocity.y);
        packet.put("tangentialSpeed1", tangentialSpeed);

        tangentialSpeed += driveDeccel * (timeSenseInit - lastTime);
        packet.put("tangentialSpeed2", tangentialSpeed);

        tangentialSpeed = Math.min(tangentialSpeed, 0.0);
        packet.put("tangentialSpeed3", tangentialSpeed);

        tangentialVelocity = tangentialVelocity.div(tangentialVelocity.norm());
        printVectorData("tangentialVelocity2", tangentialVelocity);

        tangentialVelocity = tangentialVelocity.times(tangentialSpeed);
        printVectorData("tangentialVelocity3", tangentialVelocity);

        normDistVector = distVector.div(distVector.norm());
        printVectorData("normDistVector", normDistVector);

        finalVelocityVector = new Vector2d(normDistVector.x * speed, normDistVector.y * speed);
        printVectorData("velocityVector", finalVelocityVector);

        finalVelocityVector = finalVelocityVector.plus(tangentialVelocity);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        lastTime = timeSenseInit;

        if (Math.abs(distVector.x - drive.pose.position.x) < epsilon && Math.abs(distVector.y - drive.pose.position.y) < epsilon)
            return new Vector2d(0, 0);
        else
            return finalVelocityVector;
    }

    public double driveTo(double goalX, double goalY, boolean hasDriveToInit) {
        Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y), hasDriveToInit);
        drive.setDrivePowers(null, new PoseVelocity2d(motionProfileVel, 0));
        return Math.hypot(motionProfileVel.x, motionProfileVel.y);
    }
}