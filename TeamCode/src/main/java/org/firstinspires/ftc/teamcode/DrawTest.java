package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="FTC Dashboard drawing test")
public class DrawTest extends LinearOpMode {
    void drawRobot(Pose2D pose) {
        final double ROBOT_RADIUS = 9;

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.field().drawGrid(0, 0, 144, 144, 7, 7);
        Canvas c = packet.fieldOverlay();

        // Set the pen attributes:
        c.setStroke("#3F51B5");
        c.setStrokeWidth(1);

        // Set the transform centered about the robot:
        c.setRotation(pose.getHeading(AngleUnit.RADIANS) - Math.PI / 2.0);
        c.setTranslation(pose.getY(DistanceUnit.INCH) - 72, -(pose.getX(DistanceUnit.INCH) - 72));

        // Draw the robot, with the line going out directly in front of the robot:
        c.strokeCircle(0, 0, ROBOT_RADIUS);
        c.strokeLine(0, 0, ROBOT_RADIUS, 0);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Drawing robot...");
            drawRobot(new Pose2D(DistanceUnit.INCH, 0, 72, AngleUnit.DEGREES, 90));
            telemetry.update();
        }
    }
}
