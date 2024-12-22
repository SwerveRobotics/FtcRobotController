package org.firstinspires.ftc.team417.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

// Concept for the color sensor, not for OpenCV.
@TeleOp(name = "Color Sensor", group = "Concept")
@Config
public class ColorSensorConcept extends LinearOpMode {
    ColorSensor sensor;
    RevBlinkinLedDriver lightStrip;

    @Override
    public void runOpMode() {
        Color color;

        sensor = hardwareMap.get(ColorSensor.class, "color");

        sensor.enableLed(true);

        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "lightStrip");

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

        while (opModeIsActive()) {
            color = senseColor();
            switch (color) {
                case RED:
                    telemetry.addLine("Detecting red!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                case BLUE:
                    telemetry.addLine("Detecting blue!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    break;
                case YELLOW:
                    telemetry.addLine("Detecting yellow!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;
                default:
                    telemetry.addLine("Detecting white!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }

        sensor.enableLed(false);
    }

    public Color senseColor() {
        int a = sensor.alpha();
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        telemetry.addLine(String.format("(a = %d, r = %d, g = %d, b = %d)", a, r, g, b));

        if (a < 200) { // If the detection is too transparent
            return Color.UNDETECTED;
        } else {
            if (r >= g && r >= b) {
                return Color.RED; // Most red
            } else if (g >= r && g >= b) {
                return Color.YELLOW; // Most green
            } else {
                return Color.BLUE; // Most blue
            }
        }
    }
}
