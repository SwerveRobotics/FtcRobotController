/**
 * This file is a handy place to test your robot's menu logic without needing a robot to
 * test on.
 */
package com.example.simpleconsole.userinterface;

import com.example.simpleconsole.Main;

import java.io.OutputStream;
import java.io.PrintStream;

/**
 * This class is a bit of glue to run your menu program. Don't change this!
 */
public class Robotics {
    public static Telemetry telemetry = new Telemetry();
    public static Gamepad gamepad = new Gamepad();

    public static boolean proceed = true;

    public static void main(String[] args) {
        // Set System.out:
        OutputStream output = telemetry;
        PrintStream print = new PrintStream(output);
        System.setOut(print);

        // Update the screen ever 10 milliseconds, blinking the cursor every 530 ms:
        final double cursorBlinkTime = 1060;
        final double startTime = System.currentTimeMillis();
        Thread update = new Thread(new Runnable() {
            @Override
            public void run() {
                while (proceed) {
                    double elapsedTime = System.currentTimeMillis() - startTime;
                    telemetry.update((elapsedTime % cursorBlinkTime * 2) < cursorBlinkTime);
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                    }
                }
            }
        });
        update.start();

        // Invoke Main:
        System.out.println("--------- START OF PROGRAM ---------\n");
        Main.main(args);
        System.out.println("\n---------- END OF PROGRAM ----------");
    }
}

