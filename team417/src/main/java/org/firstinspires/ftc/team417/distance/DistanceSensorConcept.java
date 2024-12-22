package org.firstinspires.ftc.team417.distance;

import static org.firstinspires.ftc.team417.distance.DistanceLocalizer.calculateDistance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.SlowBotTeleOp;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

@TeleOp(name = "Distance", group = "Concept")
@Config
public class DistanceSensorConcept extends SlowBotTeleOp {
    UltrasonicDistanceSensor leftSonic;
    UltrasonicDistanceSensor rightSonic;

    double x = 6.75;
    double y = 7.75;

    final double ANGLE_OF_POSITIVE_CORNER = 0.25 * Math.PI;

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        prepareRobot();

        leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
        rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");

        DistanceSensorInfo leftInfo = new DistanceSensorInfo(-4.5, 7, -Math.PI / 9);
        DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.5, 0, Math.PI / 2);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            double h = drive.otosDriver.getPosition().h;

            double dL = leftSonic.getDistance(DistanceUnit.INCH);
            double dR = rightSonic.getDistance(DistanceUnit.INCH);

            telemetry.addData("X distance", calculateDistance(dR, h, rightInfo, false));
            telemetry.addData("Y distance", calculateDistance(dL, h, leftInfo, true));

            // deltaTime will be the actual current time minus the currentTime of the last loop
            double deltaTime = currentTime() - previousTime;
            previousTime = currentTime();

            controlDrivebaseWithGamepads(true, false, deltaTime);

            controlMechanismsWithGamepads(deltaTime);

            telemeterData();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);
        }
    }

    /** @noinspection unused*/ // Legacy, saved in case needed later
    public void calculateLeft() {
        double heading = -drive.otosDriver.getPosition().h + ANGLE_OF_POSITIVE_CORNER;

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));

        double distance = leftSonic.getDistance(DistanceUnit.INCH);

        telemetry.addData("Raw distance (in)", distance);

        double sensorToWall = distance * Math.cos(heading);

        telemetry.addData("Sensor to wall (in)", sensorToWall);

        double leftCenterToSensor = y * Math.cos(heading + 0.25 * Math.PI);

        telemetry.addData("Left Center to Sensor (in)", leftCenterToSensor);

        double centerToLeftCenter = x * Math.sin(heading + 0.25 * Math.PI);

        telemetry.addData("Center to left center (in)", centerToLeftCenter);

        double totalDistanceFromWall = sensorToWall + leftCenterToSensor + centerToLeftCenter;

        telemetry.addData("Total distance from wall (in)", totalDistanceFromWall);
    }

    /** @noinspection unused*/ // Legacy, saved in case needed later
    public void calculateRight() {
        double heading = -drive.otosDriver.getPosition().h + ANGLE_OF_POSITIVE_CORNER;

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));

        double distance = rightSonic.getDistance(DistanceUnit.INCH);

        telemetry.addData("Raw distance (in)", distance);

        double sensorToWall = distance * Math.cos(heading);

        telemetry.addData("Sensor to wall (in)", sensorToWall);

        double topCenterToSensor = x * Math.cos(heading + 0.25 * Math.PI);

        telemetry.addData("Top Center to Sensor (in)", topCenterToSensor);

        double centerToTopCenter = y * Math.sin(heading + 0.25 * Math.PI);

        telemetry.addData("Center to left center (in)", centerToTopCenter);

        double totalDistanceFromWall = sensorToWall + topCenterToSensor + centerToTopCenter;

        telemetry.addData("Total distance from wall (in)", totalDistanceFromWall);
    }

    double normalizeToFirstQuadrant(double theta) {
        // Normalize to [0, 2π) so we're working with a clean base
        theta = theta % (2 * Math.PI);
        if (theta < 0) {
            theta += 2 * Math.PI;
        }

        // Rotate until it's within [0, π/2]
        theta = theta % (Math.PI / 2);

        return theta;
    }
}

