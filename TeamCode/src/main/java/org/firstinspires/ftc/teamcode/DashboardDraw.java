package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;


public class DashboardDraw {
    static void drawRobot(Pose pose) {
        final double ROBOT_RADIUS = 9;

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.field().drawGrid(0, 0, 144, 144, 7, 7);
        Canvas c = packet.fieldOverlay();

        // Set the pen attributes:
        c.setStroke("#3F51B5");
        c.setStrokeWidth(1);

        // Set the transform centered about the robot:
        c.setRotation(pose.heading() - Math.PI / 2.0);
        c.setTranslation(pose.y() - 72, -(pose.x() - 72));

        // Draw the robot, with the line going out directly in front of the robot:
        c.strokeCircle(0, 0, ROBOT_RADIUS);
        c.strokeLine(0, 0, ROBOT_RADIUS, 0);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
