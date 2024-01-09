package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/*
This holds some utility methods that we use in teleop and auto.
 */
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

    /**
     * calculate a value between 1 and a given multiplier
     * based on how far down a trigger is pressed down
     * draft: https://www.desmos.com/calculator/y9y0ebnvqz
     * @param triggerInput how far down the trigger is pressed between 0 and 1
     * @param maxSlowMultiplier the greatest slowmode multiplier when fully pressed
     * @return the output multiplier scaled using the trigger input
     */
    public static double getSlowMultiplier(double triggerInput, double maxSlowMultiplier) {
        return (maxSlowMultiplier - 1) * triggerInput + 1;
    }


    // these methods are generalized but are mainly meant for slide presets in teleop:

    /**
     * finds the next pos above current
     * @param pos current position
     * @return next pos, limited to max position
     */
    public static double firstPosAbove(double pos, double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            if (pos < positions[i]) {
                return positions[i];
            }
        }
        // limit to the highest inbar pos
        return positions[positions.length - 1];
    }
    /**
     * finds the next pos below current
     * @param pos current position
     * @return next pos, limited to min position
     */
    public static double firstPosBelow(double pos, double[] positions) {
        for (int i = positions.length - 1; i >= 0; i--) {
            if (pos > positions[i]) {
                return Cpositions[i];
            }
        }
        // limit to the lowest inbar pos
        return positions[0];
    }

    /**
     * find the position above the current, whether between or on a preset position
     * @param pos current position
     * @return next position above
     */
    public static double positionUp(double pos, double[] positions, double tolerance) {
        double above = firstInbarPosAbove(pos, positions);
        // if close enough to next, go one next further
        if (Math.abs(pos - above) < tolerance) {
            return firstInbarPosAbove(above, positions);
        } else {
            return above;
        }
    }
    /**
     * find the position below the current, whether between or on a preset position
     * @param pos current position
     * @return next position below
     */
    public static double positionDown(double pos, double[] positions, double tolerance) {
        double below = firstInbarPosBelow(pos, positions);
        // if close enough to next, go one next further
        if (Math.abs(pos - below) < tolerance) {
            return firstInbarPosBelow(below, positions);
        } else {
            return below;
        }
    }
}
