package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    CRServo intake;
    Servo wrist;
    MecanumDrive drive;

    public boolean hasMechanisms = MecanumDrive.driveParameters == DriveParameters.COMPETITION_ROBOT
        || MecanumDrive.driveParameters == DriveParameters.FASTBOT_MECANUM;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        intake = hardwareMap.get(CRServo.class, "BootWheelServo");
        wrist = hardwareMap.get(Servo.class, "RotationServo");

        if (hasMechanisms) {
            armMotor = hardwareMap.get(DcMotor.class, "arm");
            intake = hardwareMap.get(CRServo.class, "intake");
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

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

        // Original code in method before switch statements
        // armMotor.getCurrentPosition();
        // armMotor.setPower(gamepad2.right_stick_y);

        // These switch statements will make sure the arm only moves if the robot
        // is the competition robot and not the mecanum or x drive robot
        switch (MecanumDrive.driveParameters) {

            // If the robot is the competition robot then the arm motors will engage
            case COMPETITION_ROBOT:
                armMotor.getCurrentPosition();
                armMotor.setPower(gamepad2.right_stick_y);
                break;

            // If the robot is not the competition robot it will not engage the 'arm motor'
            // also updates telemetry data
            case DEVBOT_MECANUM:
                telemetry.addData("Arm Control", "Disabled");
                break;

            case DEVBOT_X:
                telemetry.addData("Arm Control", "Disabled");
                break;

            // Default values will just add a warning into the telemetry object and won't engage arm
            default:
                telemetry.addData("Warning", "Unknown drive parameters: " + MecanumDrive.driveParameters);
                telemetry.addData("Arm Control", "Disabled");
                break;

        }

    }

    public final double leftRotationServo = 0;
    public final double middleRotationServo = 0.5;
    public final double rightRotationServo = 1;
}