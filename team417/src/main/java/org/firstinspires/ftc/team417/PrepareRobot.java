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
    }
}
