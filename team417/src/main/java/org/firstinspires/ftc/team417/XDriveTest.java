package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "XDriveTest")
public class XDriveTest extends LinearOpMode {
    public DcMotor bl, br, fl, fr;

    @Override
    public void runOpMode() {
        bl = initializeMotor("BLMotor", DcMotor.Direction.FORWARD);
        br = initializeMotor("BRMotor", DcMotor.Direction.FORWARD);
        fl = initializeMotor("FLMotor", DcMotor.Direction.FORWARD);
        fr = initializeMotor("FRMotor", DcMotor.Direction.FORWARD);

        telemetry.addLine("Hardware setup complete");
        telemetry.update();

        waitForStart();

        while (!opModeIsActive()) {
            driveRobot();telemetry.update();
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

    public void xDrive(double x, double y, double rot) {
        // Logic not really that different from a mecanum drive
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        double frontLeftPower = (y + x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        telemetry.addData("Front left motor", fl);
        telemetry.addData("Front right motor", fr);
        telemetry.addData("Back left motor", fl);
        telemetry.addData("Back right motor", fr);

        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    public void driveRobot() {
        double x = gamepad1.left_stick_x;

        double y = -gamepad1.left_stick_y;

        double rotation = gamepad1.right_stick_x;

        telemetry.addData("Left joystick x", x);
        telemetry.addData("Left joystick y", y);
        telemetry.addData("Right joystick x", rotation);

        xDrive(x, y, rotation);
    }
}