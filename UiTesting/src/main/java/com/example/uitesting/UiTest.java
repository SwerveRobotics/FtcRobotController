/**
 *  This file is a handy place to test your robot's menu logic without needing a robot to
 *  test on.
 */
package com.example.uitesting;

import com.example.uitesting.ui.Gamepad;
import com.example.uitesting.ui.Telemetry;

/**
 * This class is a bit of glue to run your menu program. Don't change this!
 */
public class UiTest {
    public static void main(String[] args) {
        Telemetry telemetry = new Telemetry();
        Gamepad gamepad = new Gamepad();

        // Invoke the menu:
        Config config = new Config();
        config.menu(telemetry, gamepad);
    }
}
/**
 * This is a very simple template for a menu class. You can copy and paste this class to
 * and from your actual robot code.
 */
class Config {
    // Put config state that's set by the menu here. Make these 'static' so that they can be read
    // from both Auton and TeleOp:
    static boolean isRed = false;

    /**
     * Run the menu. The resulting state can be found in the public fields of this class.
     */
    void menu(Telemetry telemetry, Gamepad gamepad) {
        telemetry.addLine("Press A for red, B for blue");
        telemetry.update();

        while (!gamepad.a && !gamepad.b)
            ;

        isRed = gamepad.a;

        telemetry.addLine(String.format(" Result isRed: %s", isRed));
        telemetry.update();
    }
}
