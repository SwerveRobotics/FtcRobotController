package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.CompetitionTeleOp;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

@TeleOp(name = "Distance", group = "Concept")
@Config
public class DistanceSensorConcept extends CompetitionTeleOp {
    DistanceSensor distanceSensor;

    double x = -6.75;

    double y = 7.75;

    double theta = -Math.PI / 4;

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        initializeHardware();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceLeftCorner");

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            senseDistance();

            controlDrivebaseWithGamepads();

            controlMechanismsWithGamepads();

            telemeterData();

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);
        }
    }

    public void senseDistance() {
        double heading = drive.opticalTracker.getPosition().h;

        double sensorOffset = theta;

        double sensorHeading = heading - sensorOffset;

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));

        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Raw distance (in)", distance);

        double sensorToWall = distance * Math.cos(sensorHeading);

        telemetry.addData("Sensor to wall (in)", sensorToWall);

        double leftCenterToSensor = y * Math.cos(heading);

        telemetry.addData("Left Center to Sensor (in)", leftCenterToSensor);

        double centerToLeftCenter = x * Math.sin(heading);

        telemetry.addData("Center to left center (in)", centerToLeftCenter);

        double totalDistanceFromWall = sensorToWall + leftCenterToSensor + centerToLeftCenter;

        telemetry.addData("Total distance from wall (in)", totalDistanceFromWall);
    }
}
