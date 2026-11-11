package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "417 Drive Only")
public class DriveOnly extends TuneLauncherPIDF {
    private boolean fieldCentric = false;
    private double fieldCentricCorrection = 0; // radians

    @Override
    public void runOpMode() throws InterruptedException {
        initializeDrive();
        follower.setPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            double scale = slowModeScale();
            double forward = -gamepad1.left_stick_y * scale;
            double lateral = -gamepad1.left_stick_x * scale;
            double turn = -gamepad1.right_stick_x * scale;

            if (gamepad1.yWasPressed()) {
                fieldCentric = !fieldCentric;
            }

            // one 90 degree step per press (not per loop)
            // if one press turns it by the wrong amount, check the units ManualDrive expects
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