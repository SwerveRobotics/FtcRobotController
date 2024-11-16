package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team6220.roadrunner.Drawing;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team6220.roadrunner.RobotAction;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="CompSimTeleOp", group="Competition")
public class CompetitionSimTeleOp extends BaseOpMode {

    // Action for controlling the arm in Auto trajectories.

    private DcMotorEx armBaseMotor = null;
    private DcMotorEx slidesMotor = null;
    private CRServo intakeCRServo = null;
    private Servo dumperServo = null;
    private Servo armElbowServo = null;
    private NormalizedColorSensor colorSensor = null;
    private AllianceColor allianceColor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allianceColor = AllianceColor.RED;

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        //colorSensor.setGain(Constants.COLOR_SENSOR_GAIN);

        armBaseMotor = hardwareMap.get(DcMotorEx.class, DRIFTConstants.ARM_BASE_MOTOR_HARDWARE_IDENTIFIER);
        //slidesMotor = hardwareMap.get(DcMotorEx.class,DRIFTConstants.SLIDES_MOTOR_HARDWARE_IDENTIFIER);
        intakeCRServo = hardwareMap.get(CRServo.class, DRIFTConstants.INTAKE_SERVO_HARDWARE_IDENTIFIER);
        //dumperServo = hardwareMap.get(Servo.class,DRIFTConstants.DUMPER_SERVO_HARDWARE_IDENTIFIER);
        armElbowServo = hardwareMap.get(Servo.class,DRIFTConstants.ARM_ELBOW_SERVO_HARDWARE_IDENTIFIER);

        armBaseMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armBaseMotor.setCurrentAlert(5, CurrentUnit.AMPS);
        //slidesMotor.setCurrentAlert(5, CurrentUnit.AMPS);

        armBaseMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armBaseMotor.setTargetPosition(0);
        //slidesMotor.setTargetPosition(0);
        armBaseMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //slidesMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //armBaseMotor.setVelocity(Constants.ARM_BASE_MOTOR_VELOCITY);
        //slidesMotor.setVelocity(Constants.SLIDES_MOTOR_VELOCITY);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);
        ControlManager controls = new ControlManager(gamepad1, gamepad2);
        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");
            //telemetry.addLine("red: " + colorSensor.getNormalizedColors().red);
            //telemetry.addLine("blue: " + colorSensor.getNormalizedColors().blue);

            /*if (isHeldSampleInvalid(allianceColor, colorSensor)) {
                telemetry.addLine("WEEWOO WEEWOO! EJECT IMMEDIATELY");
            }*/

            telemetry.update();

            controls.update();

            armBaseMotor.setPower(1.0);
            armBaseMotor.setTargetPosition(controls.getArmBaseMotorPosition());
            telemetry.addLine("Target Position: " + controls.getArmBaseMotorPosition());
            //slidesMotor.setTargetPosition(controls.getSlidesMotorPosition());
            intakeCRServo.setPower(controls.getIntakeServoPower());
            //dumperServo.setPosition(controls.getDumperServoPosition());
            armElbowServo.setPosition(controls.getArmElbowServoPosition());

            // jank ass speed modifier
            float speedModifier = 1 - (gamepad1.right_trigger * 0.5f);

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * speedModifier,
                            -gamepad1.left_stick_x * speedModifier
                    ),
                    -gamepad1.right_stick_x * speedModifier
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

    private enum AllianceColor {
        RED,
        BLUE
    }

    private boolean isHeldSampleInvalid(AllianceColor allianceColor, NormalizedColorSensor colorSensor) {
        NormalizedRGBA colorSensorValues = colorSensor.getNormalizedColors();

        // code monster that determines if the opposing alliance color is greater, and checks if it's over the threshold
        switch(allianceColor) {
            case RED: {
                if (colorSensorValues.blue > colorSensorValues.red
                        && colorSensorValues.blue > DRIFTConstants.BLUE_COLOR_SENSOR_RANGES[0]) {
                    return true;
                }
            }
            case BLUE: {
                if (colorSensorValues.red > colorSensorValues.blue
                        && colorSensorValues.red > DRIFTConstants.RED_COLOR_SENSOR_RANGES[0]) {
                    return true;
                }
            }
        }
        return false;
    }
}
