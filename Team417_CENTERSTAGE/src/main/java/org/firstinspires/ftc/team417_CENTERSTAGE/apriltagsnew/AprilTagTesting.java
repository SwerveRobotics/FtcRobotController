package org.firstinspires.ftc.team417_CENTERSTAGE.apriltagsnew;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Pose;

/*
    This test class is for testing April Tags. It is basically a mecanum drive with April Tags.
*/

@TeleOp (name = "APTesting")
public class AprilTagTesting extends BaseOpMode {
    // Declares instance of our April Tag Pose Estimator
    public AprilTagPoseEstimator myATPoseEstimator;

    @Override
    public void runOpMode() {
        initializeHardware();

        // Turn red light to be off at beginning
        // DigitalChannel object for LEDs makes this counterintuitive, on = false, off = true
        red.setState(true);

        myATPoseEstimator = new AprilTagPoseEstimator(hardwareMap, telemetry);

        drive.pose = new Pose2d(0, 0, 0);

        waitForStart();

        Pose ATPose;

        while (opModeIsActive()) {
            driveUsingControllers();

            drive.updatePoseEstimate();

            telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)",
                    drive.pose.position.x, drive.pose.position.y,
                    Math.toDegrees(drive.pose.heading.log())));

            // Code added to draw the pose
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            c.setStroke("#0000ff");
            drive.drawPoseHistory(c);
            MecanumDrive.drawRobot(c, drive.pose);
            myATPoseEstimator.drawPoseHistory(c);
            ATPose = myATPoseEstimator.robotPoseEstimate;
            if (ATPose != null) {
                MecanumDrive.drawRobot(c, new Pose2d(ATPose.x, ATPose.y, ATPose.theta));
            }
            c.strokeCircle(drive.pose.position.x, drive.pose.position.y, 1);

            myATPoseEstimator.updatePoseEstimate();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);
        }
    }

    public void driveUsingControllers() {
        driveUsingControllers(false);
    }

    public boolean sensitive = false;

    public void driveUsingControllers(boolean curve) {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.425;
            rotSensitivity = 0.68;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x, y, rot;
        if (curve) {
            x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
            y = curveStick(-gamepad1.left_stick_y) * sensitivity;
            rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
        } else {
            x = gamepad1.left_stick_x * strafeConstant * sensitivity;
            y = -gamepad1.left_stick_y * sensitivity;
            rot = gamepad1.right_stick_x * rotSensitivity;
        }
        mecanumDrive(x, y, rot);
    }

    //Adds stick curve to the joysticks
    public double curveStick(double rawSpeed) {
        double logSpeed;
        if (rawSpeed >= 0) {
            logSpeed = Math.pow(rawSpeed, 2);
        } else {
            logSpeed = -Math.pow(rawSpeed, 2);
        }
        return logSpeed;
    }
}