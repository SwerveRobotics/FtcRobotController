package org.firstinspires.ftc.team8923_CENTERSTAGE;

abstract public class BaseTeleOp extends BaseOpMode {

    // constants for output servo positions (gondola and arms)
    public static double GONDOLA_DOWN_POSITION = 0.6;
    public static double GONDOLA_UP_POSITION = 0.8;
    public static double ARMS_DOWN_POSITION = 0.25;
    public static double ARMS_UP_POSITION = 0.95;

    public void driveMechanism() {
        /* controls:
        hold left bumper: intake runs in, pixel release rotates in
        hold right bumper: intake runs out, pixel release rotates out
        hold Y button: pixel release spins out
        right trigger: intake runs outward
        d-pad up and down: linear slides up and down correspondingly
        B button: gondola flip out
        A button: gondola returns to initialized position (down/in)
        press left joystick down: release drone
         */

        // intake and servo inside gondola rotate together inside when left bumper, out when right bumper
        if (gamepad2.left_bumper) {
            motorIntakeWheels.setPower(0.5);
            servoReleasePixel.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            motorIntakeWheels.setPower(-1.0);
            servoReleasePixel.setPower(-1.0);
        } else {
            motorIntakeWheels.setPower(0.0);
            servoReleasePixel.setPower(0.0);
        }

        // button Y runs pixel release servo without intake running
        if (gamepad2.y) {
            servoReleasePixel.setPower(-1.0);
        } else {
            servoReleasePixel.setPower(0.0);
        }

        // slides controlled by up and down dpad
        if (gamepad2.dpad_up) {
            servoSlideLeft.setPower(1.0);
            servoSlideRight.setPower(-1.0);
        } if (gamepad2.dpad_down) {
            servoSlideLeft.setPower(-1.0);
            servoSlideRight.setPower(1.0);
        } else {
            servoSlideLeft.setPower(0.0);
            servoSlideRight.setPower(0.0);
        }

        // flips arms in
        if (gamepad2.a) {
            servoFlipGondola.setPosition(ARMS_DOWN_POSITION);
            servoRotateGondola.setPosition(GONDOLA_DOWN_POSITION);
            // flips arms out
        } else if (gamepad2.b) {
            servoFlipGondola.setPosition(ARMS_UP_POSITION);
            servoRotateGondola.setPosition(GONDOLA_UP_POSITION);
        }
        // runs intake power proportional to how far right trigger is pressed down
        if (gamepad2.right_trigger != 0.0) {
            motorIntakeWheels.setPower(-gamepad2.right_trigger);
        }

        // release drone with pushing down right joystick, servo should rotate counter-clockwise
        if (gamepad2.right_stick_button) {
            servoReleaseDrone.setPosition(1.0);
        }
    }
}
