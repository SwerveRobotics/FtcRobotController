/**
 *  This file is a handy place to test your robot's menu logic without needing a robot to
 *  test on.
 */
package com.example.uitesting;

import com.example.uitesting.ui.Gamepad;
import com.example.uitesting.ui.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;

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
    
}
