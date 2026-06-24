package org.firstinspires.ftc.team417;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;


/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
public class CompetitionTeleopNew extends BaseOpMode {

    @Override
    public void runOpMode() {
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

            // Controls Below
            //Intake -HELP this is probably wrong
            if(gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y < -0.1){
                intakeMot.setPower(gamepad2.left_stick_y);
            }
            else{
                intakeMot.setPower(-0.5);
            }
            //Flywheel on near
            if(gamepad2.dpadDownWasPressed()){
                upperFlywheelMot.setPower(FLYWHEEL_NEAR_SPEED);
                lowerFlywheelMot.setPower(FLYWHEEL_NEAR_SPEED - FLYWHEEL_BACKSPIN);
            }
            //Flywheel on far
            if(gamepad2.dpadUpWasPressed()){
                upperFlywheelMot.setPower(FLYWHEEL_FAR_SPEED);
                lowerFlywheelMot.setPower(FLYWHEEL_FAR_SPEED - FLYWHEEL_BACKSPIN);
            }
            if(gamepad2.dpadRightWasPressed()){
                upperFlywheelMot.setPower(WHEEL_STOP_SPEED);
                lowerFlywheelMot.setPower(WHEEL_STOP_SPEED);
                transferWheelMot.setPower(WHEEL_STOP_SPEED);
            }
            //Transfer Wheel On/Launch button
            if(gamepad2.yWasPressed()){
                transferWheelMot.setPower( TRANSFER_WHEEL_START_SPEED);
            }


        }
    }
}