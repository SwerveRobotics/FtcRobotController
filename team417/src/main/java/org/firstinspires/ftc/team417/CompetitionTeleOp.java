package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends BaseOpMode {
    private double speedMultiplier = 1;
    DcMotor armMotor;
    DcMotor bootWheelServo;
    Servo rotationServo;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        bootWheelServo = hardwareMap.get(DcMotor.class, "BootWheelServo");
        rotationServo = hardwareMap.get(Servo.class, "RotationServo");

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

            controlArm();
            controlRotation();
            controlBootWheel();

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

    public void controlArm() {
        armMotor.getCurrentPosition();
        armMotor.setPower(gamepad2.right_stick_y);
    }

    public final double leftRotationServo = 0;
    public final double middleRotationServo = 0.5;
    public final double rightRotationServo = 1;

    public void controlRotation() {
        if (gamepad2.dpad_up) {
            rotationServo.setPosition(middleRotationServo);
        } else if (gamepad2.dpad_left) {
            rotationServo.setPosition(leftRotationServo);
        } else if (gamepad2.dpad_right) {
            rotationServo.setPosition(rightRotationServo);
        }
    }

    public void controlBootWheel() {
        double servoInput = gamepad2.right_trigger - gamepad2.left_trigger;
        bootWheelServo.setPower(servoInput);
    }
}