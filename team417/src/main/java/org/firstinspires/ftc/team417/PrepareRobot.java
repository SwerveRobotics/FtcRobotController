package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class PrepareRobot extends BaseOpModeSlowBot {

    @Override
    public void runOpMode() throws InterruptedException {
        // This sets the pose in the human corner facing the opponents basket
        Pose2d beginPose = new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, -Math.PI / 2);
        initializeHardware(beginPose);
        slideMotor.setVelocity(25);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setPosition(WRIST_IN);
        while(!isStarted()) {
            slideMotor.setPower(gamepad2.right_stick_y);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));

            telemetry.addLine("1. Put the robot in the human corner facing the opponents basket");
            telemetry.addLine("2. Using gamepad 2 retract the slides in all the way");
            telemetry.addLine("3. Once lift is pushed down, press start");
            telemetry.addLine("");

            telemetry.addData("Lift Motor 1 ticks: ", liftMotor1.getCurrentPosition());
            telemetry.addData("Lift Motor 2 ticks: ", liftMotor2.getCurrentPosition());
            telemetry.addData("Lift Motor 1 direction", liftMotor1.getDirection());
            telemetry.addData("Lift Motor 2 direction", liftMotor2.getDirection());

            telemetry.addData("Slide motor ticks: ", slideMotor.getCurrentPosition());
            telemetry.addData("Slide motor velocity: ", slideMotor.getVelocity());
            telemetry.update();
        }
        // Once start is clicked, reset the encoder to make the absolute bottom the home position
        initializeHardware(beginPose);
    }

}
