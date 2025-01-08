package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class PrepareRobot extends BaseOpModeSlowBot {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Welcome, press start to prepare robot");
        telemetry.update();
        initializeHardware();
        slideMotor.setVelocity(25);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(!isStopRequested()) {
            slideMotor.setPower(gamepad2.right_stick_y);
        }
        telemetry.addData("Lift Motor 1 ticks: ", liftMotor1.getCurrentPosition());
        telemetry.addData("Lift Motor 2 ticks: ", liftMotor2.getCurrentPosition());

        telemetry.addData("Slide motor ticks: ", slideMotor.getCurrentPosition());
        telemetry.addData("Slide motor velocity: ", slideMotor.getVelocity());
        telemetry.update();
    }
}
