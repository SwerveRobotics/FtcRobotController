package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
public class CompetitionTeleOp extends BaseOpMode {
    private double speedMultiplier = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                speedMultiplier = 0.25;
            } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
                speedMultiplier = 0.5;
            } else {
                speedMultiplier = 1;
            }

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * speedMultiplier,
                            -gamepad1.left_stick_x * speedMultiplier
                    ),
                    -gamepad1.right_stick_x * speedMultiplier
            ));

            // Update the current pose:
            drive.updatePoseEstimate();

            telemetry.addLine("Running TeleOp!");
            telemetry.addData("Kinematic Type", kinematicType);
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.update();

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
}