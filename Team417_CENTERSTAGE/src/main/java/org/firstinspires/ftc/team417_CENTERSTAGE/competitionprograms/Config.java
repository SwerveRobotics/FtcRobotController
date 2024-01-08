package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Config {
    // Put config state that's set by the menu here. Make these 'static' so that they can be read
    // from both Auton and TeleOp:
    public static boolean isRed = false;
    public static boolean isClose = false;
    public static boolean useAprilTags = false;
    public static boolean useOpenCV = true;
    public static boolean useDriveTo = true;

    /**
     * Run the menu. The resulting state can be found in the public fields of this class.
     */
    static void menu(Telemetry telemetry, Gamepad gamepad) {
        telemetry.addLine("Press A for red, B for blue");
        telemetry.update();

        while (!gamepad.a && !gamepad.b)
            ;

        isRed = gamepad.a;

        telemetry.addLine(String.format(" Result isRed: %s", isRed));
        telemetry.update();
    }
}