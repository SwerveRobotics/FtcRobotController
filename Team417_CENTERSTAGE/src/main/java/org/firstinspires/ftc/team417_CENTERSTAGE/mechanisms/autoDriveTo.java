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
    MecanumDrive drive;

    public autoDriveTo(MecanumDrive drive) {
        this.drive = drive;
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

    public Vector2d motionProfileWithVector(Vector2d distVector, boolean hasInit){
        Vector2d velocityVector;
        double timeSenseInit;
        double distRemaining;
        double velocity;
        double epsilon = 0.5;
        printVectorData("distVector", distVector);

        if (hasInit) {
            initTime = BaseOpMode.TIME.seconds();
        }
        packet.put("initTime", initTime);

        normDistVector = distVector.div(distVector.norm());
        printVectorData("normDistVector", normDistVector);

        timeSenseInit = BaseOpMode.TIME.seconds() - initTime;
        packet.put("timeSenseInit", timeSenseInit);

        distRemaining = Math.hypot(distVector.x, distVector.y);
        packet.put("distRemaining", distRemaining);

        velocity = driveAccel * timeSenseInit;
        packet.put("accelVelocity", velocity);

        velocity = Math.min(velocity, maxDriveVelocity);
        packet.put("maintainVelocity", velocity);

         velocity = Math.min(velocity, Math.sqrt(Math.abs(2.0 * driveDeccel * distRemaining)));
        packet.put("decelVelocity", velocity);
        packet.put("velocity", velocity);

        velocityVector = new Vector2d(normDistVector.x * velocity, normDistVector.y * velocity);
        printVectorData("velocityVector", velocityVector);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

        if (Math.abs(distVector.x - drive.pose.position.x) < epsilon && Math.abs(distVector.y - drive.pose.position.y) < epsilon)
            return new Vector2d(0, 0);
        else
            return velocityVector;
    }

    public double driveTo(double goalX, double goalY, boolean hasDriveToInit) {
        Vector2d motionProfileVel = motionProfileWithVector(new Vector2d(goalX - drive.pose.position.x, goalY - drive.pose.position.y), hasDriveToInit);
        drive.setDrivePowers(null, new PoseVelocity2d(motionProfileVel, 0));
        return Math.hypot(motionProfileVel.x, motionProfileVel.y);
    }
}