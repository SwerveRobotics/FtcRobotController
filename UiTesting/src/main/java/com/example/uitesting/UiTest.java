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
    // Put config state that's set by the menu here. Make these 'static' so that they can be read
    // from both Auton and TeleOp:
    public static boolean isRed = false;
    public static boolean isClose = false;
    public static boolean useAprilTags = false;
    public static boolean useOpenCV = true;
    public static boolean useDriveTo = true;

    static ArrayList<HashMap<Boolean, String>> configStrings;

    /**
     * Initialize the menu.
     */
    static void init() {
        configStrings = new ArrayList<>();

        HashMap<Boolean, String> redStrings = new HashMap<>();
        redStrings.put(true, "RED");
        redStrings.put(false, "BLUE");
        configStrings.add(redStrings);

        HashMap<Boolean, String> closeStrings = new HashMap<>();
        closeStrings.put(true, "CLOSE to");
        closeStrings.put(false, "FAR from");
        configStrings.add(closeStrings);

        HashMap<Boolean, String> openCVStrings = new HashMap<>();
        openCVStrings.put(true, "OPENCV CAMERA");
        openCVStrings.put(false, "DISTANCE SENSOR");
        configStrings.add(openCVStrings);

        HashMap<Boolean, String> driveToStrings = new HashMap<>();
        driveToStrings.put(true, "ENABLING");
        driveToStrings.put(false, "DISABLING");
        configStrings.add(driveToStrings);

        HashMap<Boolean, String> aprilTagStrings = new HashMap<>();
        aprilTagStrings.put(true, "INCORPORATING");
        aprilTagStrings.put(false, "NOT INCORPORATING");
        configStrings.add(aprilTagStrings);
    }

    /**
     * Run the menu. The resulting state can be found in the public fields of this class.
     */
    static void menu(Telemetry telemetry, Gamepad gamepad) {
        init();

        int choice = 0;
        boolean aIsPressed = false;
        boolean upIsPressed = false;
        boolean downIsPressed = false;
        boolean stop = false;
        while (!stop) {
            telemetry.addLine("We are playing on the ");
            telemetry.addLine(String.format("%s%s alliance, positioned ",
                    choice == 0 ? "> " : "   ", configStrings.get(0).get(isRed)));
            telemetry.addLine(String.format("%s%s the backdrop, using ",
                    choice == 1 ? "> " : "   ", configStrings.get(1).get(isClose)));
            telemetry.addLine(String.format("%s%s as our Team Prop Detection, ",
                    choice == 2 ? "> " : "   ", configStrings.get(2).get(useOpenCV)));
            telemetry.addLine(String.format("%s%s drive assistance, and ",
                    choice == 3 ? "> " : "   ", configStrings.get(3).get(useDriveTo)));
            telemetry.addLine(String.format("%s%s information from April Tags into our pose estimate.",
                    choice == 4 ? "> " : "   ", configStrings.get(4).get(useAprilTags)));
            telemetry.update();

            if (gamepad.dpad_up && !upIsPressed && choice > 0) {
                choice--;
            }

            if (gamepad.dpad_down && !downIsPressed && choice < 4) {
                choice++;
            }

            if (gamepad.a && !aIsPressed) {
                switch (choice) {
                    case 0:
                        isRed ^= true;
                        break;
                    case 1:
                        isClose ^= true;
                        break;
                    case 2:
                        useOpenCV ^= true;
                        break;
                    case 3:
                        useDriveTo ^= true;
                        break;
                    case 4:
                        useAprilTags ^= true;
                        break;
                }
            }

            if (gamepad.x) {
                stop = true;
            }

            aIsPressed = gamepad.a;
            upIsPressed = gamepad.dpad_up;
            downIsPressed = gamepad.dpad_down;
        }

        choice = -1;
        telemetry.addLine("We are running the autonomous program while playing on the ");
        telemetry.addLine(String.format("%s%s alliance, positioned ",
                choice == 0 ? "> " : "   ", configStrings.get(0).get(isRed)));
        telemetry.addLine(String.format("%s%s the backdrop, using ",
                choice == 1 ? "> " : "   ", configStrings.get(1).get(isClose)));
        telemetry.addLine(String.format("%s%s as our Team Prop Detection, ",
                choice == 2 ? "> " : "   ", configStrings.get(2).get(useOpenCV)));
        telemetry.addLine(String.format("%s%s drive assistance, and ",
                choice == 3 ? "> " : "   ", configStrings.get(3).get(useDriveTo)));
        telemetry.addLine(String.format("%s%s information from April Tags into our pose estimate.",
                choice == 4 ? "> " : "   ", configStrings.get(4).get(useAprilTags)));
        telemetry.addData("isRed", isRed);
        telemetry.addData("isClose", isClose);
        telemetry.addData("useOpenCV", useOpenCV);
        telemetry.addData("useDriveTo", useDriveTo);
        telemetry.addData("useAprilTags", useAprilTags);
        telemetry.update();
    }
}