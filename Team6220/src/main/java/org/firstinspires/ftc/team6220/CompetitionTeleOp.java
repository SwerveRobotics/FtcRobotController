package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.team6220.roadrunner.Drawing;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
public class CompetitionTeleOp extends BaseOpMode {

    private NormalizedColorSensor colorSensor = null;
    private AllianceColor allianceColor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allianceColor = AllianceColor.RED;

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        //colorSensor.setGain(Constants.COLOR_SENSOR_GAIN);


        initializeHardware();
        //armElbowServo.setPosition(0);

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


            controls.update(getRuntime());

            armBaseMotor.setTargetPosition(controls.getArmBaseMotorPosition());
            slidesMotor.setTargetPosition(controls.getSlidesMotorPosition());
            intakeCRServo.setPower(controls.getIntakeServoPower());
            dumperServo.setPosition(controls.getDumperServoPosition());
            armElbowServo.setPosition(controls.getArmElbowServoPosition());

            // Backup control in case slides stop working mid-game
            if (controls.shouldResetSlideEncoder()) {
                slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // speed modifier
            float speedModifier = 1 - (gamepad1.right_trigger * 0.5f);

            // if transferring, halve drive speed
            if (controls.isTransferring()) {
                speedModifier *= 0.5F;
            }

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

            // update telemetry
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
