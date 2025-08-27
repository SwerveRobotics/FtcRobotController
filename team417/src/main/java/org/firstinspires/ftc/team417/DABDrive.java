package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "DAB")
public class DABDrive extends BaseOpMode {

    // Input state
    private boolean isJoystickLeft = false;
    private boolean isJoystickRight = false;

    private boolean isLBumperPressed = false;
    private boolean isRBumperPressed = false;
    private boolean isLTriggerPressed = false;
    private boolean isRTriggerPressed = false;

    /**
     * Normalize and calculate motor powers based on joystick/dpad input.
     */
    public double[] applyPowerFormula(double x, double y) {
        double denominator = Math.max(Math.abs(x) + Math.abs(y), 1);
        double left = (x + y) / denominator;
        double right = (y - x) / denominator;
        return new double[]{left, right};
    }

    /**
     * Converts D-Pad input into x,y coordinates.
     */
    public double[] takeDpadInput() {
        double x = 0;
        double y = 0;
        if (gamepad1.dpad_right) x += 1;
        if (gamepad1.dpad_left) x -= 1;
        if (gamepad1.dpad_up) y += 1;
        if (gamepad1.dpad_down) y -= 1;
        return new double[]{x, y};
    }

    /**
     * Converts button input (ABXY) into x,y coordinates.
     */
    public double[] takeButtonInput() {
        double x = 0;
        double y = 0;
        if (gamepad1.b) x += 1;
        if (gamepad1.x) x -= 1;
        if (gamepad1.y) y += 1;
        if (gamepad1.a) y -= 1;
        return new double[]{x, y};
    }

    public boolean isDpadPressed() {
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
    }

    /**
     * Decides driving powers based on joysticks, dpad, or buttons.
     */
    public double[] drive() {
        double[] coordinates;

        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
            coordinates = new double[]{gamepad1.left_stick_x, -gamepad1.left_stick_y};
        } else if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            coordinates = new double[]{gamepad1.right_stick_x, -gamepad1.right_stick_y};
        } else if (isDpadPressed()) {
            coordinates = takeDpadInput();
        } else {
            coordinates = takeButtonInput();
        }

        return applyPowerFormula(coordinates[0], coordinates[1]);
    }

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            double[] powers = drive();
            double leftPower = powers[0];
            double rightPower = powers[1];

            // TODO: assign leftPower and rightPower to your drive motors
            // Example: leftMotor.setPower(leftPower); rightMotor.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}
