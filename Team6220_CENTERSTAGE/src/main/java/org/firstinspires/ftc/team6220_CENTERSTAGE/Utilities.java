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


    /**
     * checks if a inbar position is close enough to a target position
     * @param pos current inbar position
     * @param target destination inbar position
     * @return true if within tolerance constant
     */
    public static boolean nearInbarPos(double pos, double target) {
        return Math.abs(pos - target) < Constants.INBAR_POS_TOLERANCE;
    }

    /**
     * finds the next inbar pos above current
     * @param pos current inbar position
     * @return next pos, limited to max position
     */
    public static double firstInbarPosAbove(double pos) {
        for (int i = 0; i < Constants.INBAR_POSITIONS.length; i++) {
            if (pos < Constants.INBAR_POSITIONS[i]) {
                return Constants.INBAR_POSITIONS[i];
            }
        }
        // limit to the highest inbar pos
        return Constants.INBAR_MAX_POSITION;
    }
    /**
     * finds the next inbar pos below current
     * @param pos current inbar position
     * @return next pos, limited to min position
     */
    public static double firstInbarPosBelow(double pos) {
        for (int i = Constants.INBAR_POSITIONS.length - 1; i >= 0; i--) {
            if (pos > Constants.INBAR_POSITIONS[i]) {
                return Constants.INBAR_POSITIONS[i];
            }
        }
        // limit to the lowest inbar pos
        return Constants.INBAR_MIN_POSITION;
    }

    /**
     * find the inbar position above the current, whether between or on a preset position
     * @param pos current inbar position
     * @return next position above
     */
    public static double inbarUp(double pos) {
        double above = firstInbarPosAbove(pos);
        // if close enough to next, go one next further
        if (nearInbarPos(pos, above)) {
            return firstInbarPosAbove(above);
        } else {
            return above;
        }
    }
    /**
     * find the inbar position below the current, whether between or on a preset position
     * @param pos current inbar position
     * @return next position below
     */
    public static double inbarDown(double pos) {
        double below = firstInbarPosBelow(pos);
        // if close enough to next, go one next further
        if (nearInbarPos(pos, below)) {
            return firstInbarPosBelow(below);
        } else {
            return below;
        }
    }
}
