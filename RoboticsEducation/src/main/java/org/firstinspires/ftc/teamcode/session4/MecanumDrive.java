package org.firstinspires.ftc.teamcode.session4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Demonstration of how to write mecanum drive controls for FTC
@TeleOp(name = "Mecanum Drive")
public class MecanumDrive extends LinearOpMode {
    public DcMotor bl, br, fl, fr;

    @Override
    public void runOpMode() {
        bl = initializeMotor("BLMotor", DcMotor.Direction.FORWARD);
        br = initializeMotor("BRMotor", DcMotor.Direction.FORWARD);
        fl = initializeMotor("FLMotor", DcMotor.Direction.FORWARD);
        fr = initializeMotor("FRMotor", DcMotor.Direction.FORWARD);

        telemetry.addLine("Hardware setup complete");
        telemetry.update();

        // Wait for the user to press the start button:
        waitForStart();

        // opModeIsActive is true after the start button is pressed and
        //     until the stop button is pressed.
        while (!opModeIsActive()) {
            // Make the robot move according to the input from the gamepad:
            driveRobot();

            // Clear all the data from the telemetry and replace with the new data:
            telemetry.update();
        }
    }

    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public void mecanumDrive(double x, double y, double rot) {
        // The powers must be divided to ensure the motor powers are set to between -1 and 1.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        // The mechanics of the mecanum wheels cause the robot to be able to move in all directions.
        // These expressions allow us to calculate the power required for each motor according to
        //     the overall desired movement.
        double frontLeftPower = (y + x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        // Telemeter useful debugging data to the driver hub:
        telemetry.addData("Front left motor", fl);
        telemetry.addData("Front right motor", fr);
        telemetry.addData("Back left motor", fl);
        telemetry.addData("Back right motor", fr);

        // Set the motors to the powers determined above:
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    public void driveRobot() {
        // The input here needs to be multiplied by 1.1 because the mecanum drive
        //     covers less distance when strafing.
        double x = gamepad1.left_stick_x * 1.1;

        // The input from the joysticks are y-inverted, so the negative is needed to compensate.
        double y = -gamepad1.left_stick_y;

        // The rotation of the robot is usually controlled by the right stick.
        double rotation = gamepad1.right_stick_x;

        // Telemeter useful debugging data to the driver hub:
        telemetry.addData("Left joystick x", x);
        telemetry.addData("Left joystick y", y);
        telemetry.addData("Right joystick x", rotation);

        // Drive using the controls from the gamepad:
        mecanumDrive(x, y, rotation);
    }
}
