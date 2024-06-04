/**
 * This file is a handy place to test your robot's menu logic without needing a robot to
 * test on.
 */
package com.example.simpleconsole.ui;

import com.example.simpleconsole.Main;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/**
 * This class is a bit of glue to run your menu program. Don't change this!
 */
public class Robotics {
    public static Telemetry telemetry = new Telemetry();
    public static Gamepad gamepad = new Gamepad();

    public static boolean proceed = true;

    public static void main(String[] args) {
        // Set output:
        OutputStream output = telemetry;
        PrintStream print = new PrintStream(output);
        System.setOut(print);

        ScheduledExecutorService execServ
                = Executors.newSingleThreadScheduledExecutor();

        Thread update = new Thread(new Runnable() {
            @Override
            public void run() {
                while (proceed) {
                    telemetry.update();

                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.interrupted();
                    }
                }
            }
        });
        update.start();

        // Invoke Main:
        Main.main(args);
    }
}

