package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.CompetitionTeleOp;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

@TeleOp(name = "Distance", group = "Concept")
@Config
public class DistanceSensorConcept extends CompetitionTeleOp {
    UltrasonicDistanceSensor leftSonic;
    UltrasonicDistanceSensor rightSonic;

    double x = 6.75;
    double y = 7.75;

    final double ANGLE_OF_POSITIVE_CORNER = 0.25 * Math.PI;

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        initializeHardware();

        leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
        rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");

        DistanceSensorInfo leftInfo = new DistanceSensorInfo(-6.75, 7.75, -0.25 * Math.PI);
        DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.75, 7.75, 0.25 * Math.PI);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            double h = drive.opticalTracker.getPosition().h;

            double dL = leftSonic.getDistance(DistanceUnit.INCH);
            double dR = rightSonic.getDistance(DistanceUnit.INCH);

            telemetry.addData("X distance", calculateDistance(dL, h, leftInfo));
            telemetry.addData("Y distance", calculateDistance(dR, h, rightInfo));

            controlDrivebaseWithGamepads(true, false);

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

    /** @noinspection unused*/ // Legacy, saved in case needed later
    public void calculateLeft() {
        double heading = -drive.opticalTracker.getPosition().h + ANGLE_OF_POSITIVE_CORNER;

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
        double heading = -drive.opticalTracker.getPosition().h + ANGLE_OF_POSITIVE_CORNER;

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

    double calculateDistance(double distance, double heading, DistanceSensorInfo info) {
        // Calculate the heading relative to the line that bisects the right angle of the corner
        // heading because I'm using right is positive, where RoadRunner uses left is positive
        double relative = -heading + ANGLE_OF_POSITIVE_CORNER;

        telemetry.addData("Heading", relative);

        telemetry.addData("Raw distance", distance);
        // Distance from the sensor to the wall
        double sensorToWall = distance * Math.cos(relative);

        telemetry.addData("d", sensorToWall);
        // n := distance involving the y-value of the info
        double n = info.getYOffset() * Math.cos(relative - info.getThetaOffset());
        telemetry.addData("n", n);

        // m := distance involving the x-value of the info
        double m = -info.getXOffset() * Math.sin(relative - info.getThetaOffset());
        telemetry.addData("m", m);

        return sensorToWall + n + m;
    }
}

