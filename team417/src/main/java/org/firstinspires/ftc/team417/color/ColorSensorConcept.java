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

    public final Lab RED = new Lab(526, -152, 90);
    public final Lab BLUE = new Lab(507, -142, -98);
    public final Lab YELLOW = new Lab(733, -337, 340);

    @Override
    public void runOpMode() {
        Color color;

        sensor = hardwareMap.get(ColorSensor.class, "color");

        sensor.enableLed(false);

        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "indicatorLed");

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

        sensor.enableLed(true);

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
                    telemetry.addLine("Detecting nothing!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }

        sensor.enableLed(false);
    }

    // Get color based on ARGB
    Color senseColor() {
        int a = sensor.alpha();
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        Lab labColor = ColorConverter.rgbToLab(r, g, b);

        telemetry.addData("Lab color", labColor.toString());

        if (labColor.equals(YELLOW, 20)) {
            return Color.YELLOW;
        }

        if (labColor.equals(RED, 20)) {
            return Color.RED;
        }

        if (labColor.equals(BLUE, 20)) {
            return Color.BLUE;
        }

        return Color.UNDETECTED;
    }

    public static Color analyzeDominance(int alpha, int red, int green, int blue) {
        // Apply alpha channel to RGB values
        double alphaFactor = alpha / 255.0;
        int adjustedRed = (int) (red * alphaFactor);
        int adjustedGreen = (int) (green * alphaFactor);
        int adjustedBlue = (int) (blue * alphaFactor);

        // Calculate yellow intensity (average of red and green)
        int yellow = (adjustedRed + adjustedGreen) / 2;

        // Find the maximum value
        int maxValue = Math.max(Math.max(adjustedRed, adjustedGreen),
                Math.max(adjustedBlue, yellow));

        if (maxValue == yellow) return Color.YELLOW;
        if (maxValue == adjustedRed) return Color.RED;
        if (maxValue == adjustedGreen) return Color.UNDETECTED;
        return Color.BLUE;
    }
}
