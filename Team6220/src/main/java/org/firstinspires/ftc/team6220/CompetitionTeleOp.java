package org.firstinspires.ftc.team6220;

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

import org.firstinspires.ftc.team6220.roadrunner.Drawing;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
public class CompetitionTeleOp extends BaseOpMode {

    private DcMotor armBaseMotor = null;
    private DcMotor slidesMotor = null;
    private CRServo intakeCRServo = null;
    private Servo dumperServo = null;
    private Servo armElbowServo = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armBaseMotor = hardwareMap.get(DcMotor.class,"armBaseMotor");
        slidesMotor = hardwareMap.get(DcMotor.class,"slidesMotor");
        intakeCRServo = hardwareMap.get(CRServo.class,"intakeServo");
        dumperServo = hardwareMap.get(Servo.class,"dumperServo");
        armElbowServo = hardwareMap.get(Servo.class,"armElbowServo");

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);
        ControlManager controls = new ControlManager(gamepad1, gamepad2);
        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");
            telemetry.update();

            controls.update();

            armBaseMotor.setPower(controls.getArmBaseMotorPower());
            slidesMotor.setPower(controls.getSlidesMotorPower());
            intakeCRServo.setPower(controls.getIntakeServoPower());
            dumperServo.setPosition(controls.getDumperServoPosition());
            armElbowServo.setPosition(controls.getArmElbowServoPosition());

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

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
