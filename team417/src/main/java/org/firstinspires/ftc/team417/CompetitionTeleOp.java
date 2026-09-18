package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class CompetitionTeleOp extends BaseOpMode {

    @Override
    public void runOpMode() {
        initializeHardware();
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");
            telemetry.update();

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            if (gamepad2.left_stick_y > 0.05) {
                intakeMot.setPower(1);
            } else if (gamepad2.left_stick_y < -0.05){
                intakeMot.setPower(-1);
            } else {
                intakeMot.setPower(0);
            }

            // Update the current pose:
            drive.updatePoseEstimate();

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

/*
public double doSLOWMODE() {
    if (gamepad1.right_trigger != 0) {
        return ((-gamepad1.right_trigger + 1)/2) + 0.5;
    } else {
        return 1;
    }
}
public static double halfLinearHalfCubic(double input) {
    return (Math.pow(input, 3) + input) / 2;
}
*/