package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class Utilities {

    /**
     * checks all the buttons from an array to find an index of one that's pressed
     * @param gamepad gamepad object to read from
     * @param keycodes array of button enums
     * @return index of first pressed button, none found returns -1
     */
    public static int justPressedAny(GamepadEx gamepad, GamepadKeys.Button[] keycodes) {
        for (int i = 0; i < keycodes.length; i++) {
            if (gamepad.wasJustPressed(keycodes[i])) {
                return i;
            }
        }
        return -1;
    }

    /**
     * keeps angles between [-180,180] so that it does not exceed possible imu readings
     * tested in https://www.desmos.com/calculator/igccxromri
     * @param angle angle to limit (degrees)
     * @return equivalent angle between -180 and 180
     */
    public static double limitAngle(double angle) {
        return gooderMod((angle + 180.0), 360.0) - 180.0;
    }
    // does modulus similar to desmos mod function, normal java % is different and doesn't work
    public static double gooderMod(double a, double b) {
        return a - Math.floor(a / b) * b;
    }

    /**
     * finds the shortest distance between two angles in the imu range (-180 to 180)
     * tested in https://www.desmos.com/calculator/bsw432fulz
     * @param current current heading
     * @param target destination heading
     * @return shortest difference current -> target
     */
    public static double shortestDifference(double current, double target) {
        return limitAngle(target - current);
    }

    /**
     * clamp value between a minimum and maximum value
     * @param val value to clamp
     * @param min minimum value allowed
     * @param max maximum value allowed
     * @return clamped value
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    /**
     * shortcut for clamping between -1.0 and 1.0
     * @param val value to clamp
     * @return clamped value
     */
    public static double clamp(double val) {
        return clamp(val, -1.0, 1.0);
    }
}
