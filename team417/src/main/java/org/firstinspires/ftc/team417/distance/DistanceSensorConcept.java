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

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        initializeHardware();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceFrontLeft");

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
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Raw distance (in)", distance);

        double heading = drive.opticalTracker.getPosition().h;

        double wallDistance = distance * Math.cos(heading);

        telemetry.addData("Wall distance (in)", wallDistance);
    }
}
