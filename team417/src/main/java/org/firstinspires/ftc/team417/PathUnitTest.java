package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

@Config
@TeleOp(name = "PathUnitTest")
public class PathUnitTest extends BaseOpModeFastBot{

    private boolean xPressed;
    public double startHeading;

    @Override
    public void runOpMode() {
        prepareRobot(new Pose2d(-0, -48, Math.PI / 2));
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#3F5100");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        dashboard.sendTelemetryPacket(packet);
        drive.updatePoseEstimate();
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);

        AutoDriveTo driveTo = new AutoDriveTo(drive);

        waitForStart();

        while (opModeIsActive()) {

            double deltaT = 0.2;

            if (gamepad1.x && !xPressed) {
                driveTo.init(0, -24);
            }
            if (gamepad1.x) {
                driveTo.linearDriveTo(0, -24, 0, false, packet, deltaT);
            }
            xPressed = gamepad1.x;

            WilyWorks.updateSimulation(deltaT);
            try {
                //Thread.sleep((int) Constants.DELTA_T * 1000);
                Thread.sleep(200);
            } catch (InterruptedException e) {

            }

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            packet.put("x", drive.pose.position.x);
            packet.put("y", drive.pose.position.y);
            packet.put("heading", drive.pose.heading);

            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public void prepareRobot(Pose2d startingPose) {
        drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, startingPose);
        initializeHardware();

        startHeading = startingPose.heading.log();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }
}
