package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
@TeleOp(name = "PathUnitTest")
public class PathUnitTest extends BaseOpMode{
    PathFollowing curveDrive;

    @Override
    public void runOpMode() {
        initializeHardware();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#3F5100");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        MecanumDrive.drawRobot(canvas, drive.pose);
        dashboard.sendTelemetryPacket(packet);

        curveDrive = new PathFollowing(drive, canvas, telemetry);

        drive.updatePoseEstimate();

        Bezier controlPoints= new Bezier(new DPoint(0, 0),
                new DPoint(0, 24), new DPoint(24, 24), new DPoint(24, 0), Constants.LINE_APROX_EPSILON);

        waitForStart();

        curveDrive.init(new DPoint(0, 0));

        while (opModeIsActive()) {
            //curveDrive.cubicDriveTo(controlPoints, false);


            //curveDrive.linearDriveTo(new DPoint(24, 0));
            curveDrive.cubicDriveTo(controlPoints);
            WilyWorks.updateSimulation(Constants.DELTA_T);
            try {
                //Thread.sleep((int) Constants.DELTA_T * 1000);
                Thread.sleep(200);
            } catch (InterruptedException e) {

            }




            //resetIMUIfNeeded();

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            packet.put("x", drive.pose.position.x);
            packet.put("y", drive.pose.position.y);
            packet.put("heading", drive.pose.heading);

            MecanumDrive.drawRobot(canvas, drive.pose);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        // Close drive (release resources)
        drive.close();
    }
}