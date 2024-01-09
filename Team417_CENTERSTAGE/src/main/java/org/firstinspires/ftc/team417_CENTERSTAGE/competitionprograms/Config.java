package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;

public class Config {
    // Put config state that's set by the menu here. Make these 'static' so that they can be read
    // from both Auton and TeleOp:
    public static boolean isRed = false;
    public static boolean isClose = false;

    public static boolean useOpenCV = true;
    public static boolean doBackdropPixel = true;
    public static boolean useDriveTo = true;
    public static boolean useAprilTags = false;

    public static String summary = "Config menu was not used :(";

    public static Pose2d pose = new Pose2d(0, 0, 0);

    static ArrayList<HashMap<Boolean, String>> configStrings;
    static ArrayList<HashMap<Boolean, String>> summaryStrings;
    static ArrayList<Boolean> defaults;

    /**
     * Initialize the menu.
     */
    static void init() {
        configStrings = new ArrayList<>();

        HashMap<Boolean, String> redStrings = new HashMap<>();
        redStrings.put(true, "\uD83D\uDFE5 RED");
        redStrings.put(false, "\uD83D\uDD35 BLUE");
        configStrings.add(redStrings);

        HashMap<Boolean, String> closeStrings = new HashMap<>();
        closeStrings.put(true, "\u23f1\ufe0f CLOSE to");
        closeStrings.put(false, "\u231b FAR from");
        configStrings.add(closeStrings);

        HashMap<Boolean, String> openCVStrings = new HashMap<>();
        openCVStrings.put(true, "\uD83D\uDCF8 OPENCV CAMERA");
        openCVStrings.put(false, "\uD83D\uDCA5 DISTANCE SENSOR");
        configStrings.add(openCVStrings);

        HashMap<Boolean, String> backdropStrings = new HashMap<>();
        backdropStrings.put(true, "\uD83D\uDFE1 PUTTING");
        backdropStrings.put(false, "\u26d4 NOT PUTTING");
        configStrings.add(backdropStrings);

        HashMap<Boolean, String> driveToStrings = new HashMap<>();
        driveToStrings.put(true, "\uD83D\uDE97 ENABLING");
        driveToStrings.put(false, "\uD83C\uDFC3 DISABLING");
        configStrings.add(driveToStrings);

        HashMap<Boolean, String> aprilTagStrings = new HashMap<>();
        aprilTagStrings.put(true, "\uD83C\uDFFD INCORPORATING");
        aprilTagStrings.put(false, "\u274c NOT INCORPORATING");
        configStrings.add(aprilTagStrings);

        summaryStrings = new ArrayList<>();

        HashMap<Boolean, String> redSummaryStrings = new HashMap<>();
        redSummaryStrings.put(true, "\uD83D\uDFE5 RED");
        redSummaryStrings.put(false, "\uD83D\uDD35 BLUE");
        summaryStrings.add(redSummaryStrings);

        HashMap<Boolean, String> closeSummaryStrings = new HashMap<>();
        closeSummaryStrings.put(true, "\u23f1\ufe0f CLOSE");
        closeSummaryStrings.put(false, "\u231b FAR");
        summaryStrings.add(closeSummaryStrings);

        HashMap<Boolean, String> openCVSummaryStrings = new HashMap<>();
        openCVSummaryStrings.put(true, "\uD83D\uDCF8 OPENCV");
        openCVSummaryStrings.put(false, "\uD83D\uDCA5 DISTANCE");
        summaryStrings.add(openCVSummaryStrings);

        HashMap<Boolean, String> backdropSummaryStrings = new HashMap<>();
        backdropSummaryStrings.put(true, "\uD83D\uDFE1 BACKDROP YES");
        backdropSummaryStrings.put(false, "\u26d4 BACKDROP NO");
        summaryStrings.add(backdropSummaryStrings);

        HashMap<Boolean, String> driveToSummaryStrings = new HashMap<>();
        driveToSummaryStrings.put(true, "\uD83D\uDE97 DRIVE TO YES");
        driveToSummaryStrings.put(false, "\uD83C\uDFC3 DRIVE TO NO");
        summaryStrings.add(driveToSummaryStrings);

        HashMap<Boolean, String> aprilTagSummaryStrings = new HashMap<>();
        aprilTagSummaryStrings.put(true, "\uD83C\uDFFD APRIL TAGS YES");
        aprilTagSummaryStrings.put(false, "\u274c APRIL TAGS NO");
        summaryStrings.add(aprilTagSummaryStrings);

        defaults = new ArrayList<>();
        defaults.add(isRed);
        defaults.add(isClose);
        defaults.add(useOpenCV);
        defaults.add(doBackdropPixel);
        defaults.add(useDriveTo);
        defaults.add(useAprilTags);
    }

    static int choice;
    static boolean aIsPressed;
    static boolean xIsPressed;
    static boolean upIsPressed;
    static boolean downIsPressed;

    /**
     * Run the menu. The resulting state can be found in the public fields of this class.
     */
    static boolean menu(Telemetry telemetry, Gamepad gamepad) {
        boolean end = false;

        telemetry.addLine("We are playing on the ");
        telemetry.addLine(String.format("%s%s alliance, positioned ",
                choice == 0 ? "->" : "  ", configStrings.get(0).get(isRed)));
        telemetry.addLine(String.format("%s%s the backdrop, using ",
                choice == 1 ? "->" : "  ", configStrings.get(1).get(isClose)));
        telemetry.addLine(String.format("%s%s as our Team Prop Detection, ",
                choice == 2 ? "->" : "  ", configStrings.get(2).get(useOpenCV)));
        telemetry.addLine(String.format("%s%s the yellow pixel on the backdrop",
                choice == 3 ? "->" : "  ", configStrings.get(3).get(doBackdropPixel)));
        telemetry.addLine(String.format("%s%s drive assistance to the backboard, and ",
                choice == 4 ? "->" : "  ", configStrings.get(4).get(useDriveTo)));
        telemetry.addLine(String.format("%s%s information from April Tags into our pose estimate (ONLY IN TELEOP).",
                choice == 5 ? "->" : "  ", configStrings.get(5).get(useAprilTags)));

        telemetry.addLine();
        telemetry.addLine("Press D-pad Up and D-Pad Down to choose an item");
        telemetry.addLine("Press A to modify the chosen item");
        telemetry.addLine("Press X to reset to the default configuration");
        telemetry.addLine("Press Start to exit the menu and start robot initialization");

        telemetry.update();

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
                    doBackdropPixel ^= true;
                    break;
                case 4:
                    useDriveTo ^= true;
                    break;
                case 5:
                    useAprilTags ^= true;
                    break;
            }
        }

        if (gamepad.x && !xIsPressed) {
            isRed = defaults.get(0);
            isClose = defaults.get(1);
            useOpenCV = defaults.get(2);
            doBackdropPixel = defaults.get(3);
            useDriveTo = defaults.get(4);
            useAprilTags = defaults.get(5);
        }

        if (gamepad.dpad_up && !upIsPressed && choice > 0) {
            choice--;
        }

        if (gamepad.dpad_down && !downIsPressed && choice < configStrings.size() - 1) {
            choice++;
        }

        if (gamepad.start) {
            end = true;
        }

        aIsPressed = gamepad.a;
        xIsPressed = gamepad.x;
        upIsPressed = gamepad.dpad_up;
        downIsPressed = gamepad.dpad_down;

        summary = String.format(
                "%s — %s — %s — %s — %s — %s",
                summaryStrings.get(0).get(isRed),
                summaryStrings.get(1).get(isClose),
                summaryStrings.get(2).get(useOpenCV),
                summaryStrings.get(3).get(doBackdropPixel),
                summaryStrings.get(4).get(useDriveTo),
                summaryStrings.get(5).get(useAprilTags)
                );

        return end;
    }
}