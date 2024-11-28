package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
        prepareRobot(new Pose2d(-0, -48, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();

        drive.updatePoseEstimate();

        AutoDriveTo driveTo = new AutoDriveTo(drive);

        waitForStart();

        boolean pathing = false;

        while (opModeIsActive()) {
            double deltaT = 0.02;

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            if (gamepad1.x && !xPressed && !pathing) {
                driveTo.init(0, 0, Math.PI / 2);
            }
            if (gamepad1.x || pathing) {
                pathing = !driveTo.linearDriveTo(deltaT, packet, canvas);
            } else {
                // Set the drive motor powers according to the gamepad input:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x),
                        -gamepad1.right_stick_x));
            }

            xPressed = gamepad1.x;

            WilyWorks.updateSimulation(deltaT);
            try {
                //Thread.sleep((int) Constants.DELTA_T * 1000);
                Thread.sleep(20);
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
