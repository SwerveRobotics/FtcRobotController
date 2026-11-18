package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Tester")
public class Servo_testing extends BaseOpMode {

    private double position = 0.5;

    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {

            double joystick = -gamepad1.right_stick_y;
//            if(gamepad1.a){
//                turretServo.setPosition(1.0);
//                turretServo2.setPosition(1.0);
//
//            }
//            else if(gamepad1.b){
//                turretServo.setPosition(0);
//                turretServo2.setPosition(0);
//
//            }
            if (Math.abs(joystick) > 0.05) {
                position += joystick * 0.05/100;
            }

            position = Math.max(0.0, Math.min(1.0, position));

            turretServo.setPosition(position);
            turretServo2.setPosition(position);

            telemetry.addData("Servo 1", turretServo.getPosition());
            telemetry.addData("Servo 2", turretServo2.getPosition());
            telemetry.addData("Position", position);
            telemetry.update();
        }
    }
}