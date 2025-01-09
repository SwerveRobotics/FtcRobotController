package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class PrepareRobot extends BaseOpModeSlowBot {

    @Override
    public void runOpMode() throws InterruptedException {
        // This sets the pose in the human corner facing the opponents basket
        initializeHardware(new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, -Math.PI / 2));
        slideMotor.setVelocity(25);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setPosition(WRIST_IN);
        while(!isStarted()) {
            slideMotor.setPower(gamepad2.right_stick_y);

            telemetry.addLine("1. Put the robot in the human corner facing the opponents basket");
            telemetry.addLine("2. Using gamepad 2 retract the slides in all the way");
            telemetry.addLine("3. Once lift is pushed down, press start");
            telemetry.addLine("");

            telemetry.addData("Lift Motor 1 ticks: ", liftMotor1.getCurrentPosition());
            telemetry.addData("Lift Motor 2 ticks: ", liftMotor2.getCurrentPosition());

            telemetry.addData("Slide motor ticks: ", slideMotor.getCurrentPosition());
            telemetry.addData("Slide motor velocity: ", slideMotor.getVelocity());
            telemetry.update();
        }
        // Once start is clicked, reset the encoder to make the absolute bottom the home position
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
