package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drive-only teleop, all in one file. Only touches the drivetrain and pinpoint
 * (through Pedro's Constants), so it works with a bare robot.
 *
 * Left stick = drive, right stick x = turn, right trigger = slow mode
 * Y = toggle robot-centric / field-centric
 * Dpad left / right = rotate the field-centric zero by 90 degrees
 * Left bumper = reset pose to (0, 0, 0)
 */
@TeleOp(name = "417 Drive Only")
public class DriveOnly extends LinearOpMode {
    private Follower follower;
    private boolean fieldCentric = false;
    private double fieldCentricCorrection = 0; // radians

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // hold right trigger to slow down (1 normally, 0.5 at full trigger)
            double scale = 1;
            if (gamepad1.right_trigger != 0) {
                scale = ((-gamepad1.right_trigger + 1) / 2) + 0.5;
            }

            // stick up is NEGATIVE in the FTC SDK, so these all need the minus sign
            double forward = -gamepad1.left_stick_y * scale;
            double lateral = -gamepad1.left_stick_x * scale;
            double turn = -gamepad1.right_stick_x * scale;

            if (gamepad1.yWasPressed()) {
                fieldCentric = !fieldCentric;
            }

            // one 90 degree step per press (not per loop)
            if (gamepad1.dpadLeftWasPressed()) {
                fieldCentricCorrection += Math.toRadians(90);
            }
            if (gamepad1.dpadRightWasPressed()) {
                fieldCentricCorrection -= Math.toRadians(90);
            }

            if (gamepad1.left_bumper) {
                follower.setPose(new Pose(0, 0, 0));
            }

            if (!fieldCentric) {
                follower.manual(forward, lateral, turn);
            } else {
                DrivePowers powers = ManualDrive.fieldCentric(
                        forward, lateral, turn,
                        follower.pose().heading(), fieldCentricCorrection);
                follower.manual(powers);
            }

            follower.update();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Field-centric", fieldCentric);
            telemetry.addData("FC correction (deg)", Math.toDegrees(fieldCentricCorrection));
            telemetry.addData("Slow mode scale", scale);
            telemetry.update();
        }
    }
}