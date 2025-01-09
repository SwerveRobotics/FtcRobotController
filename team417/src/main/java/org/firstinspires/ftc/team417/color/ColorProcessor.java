package org.firstinspires.ftc.team417.color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorProcessor {
    ColorSensor sensor;
    RevBlinkinLedDriver lightStrip;

    public ColorProcessor(ColorSensor sensor, RevBlinkinLedDriver lightStrip) {
        this.sensor = sensor;
        this.lightStrip = lightStrip;
    }

    // Detect the color and update the light strip
    public Color update() {
        Color color = Color.UNDETECTED;
        if (sensor != null) {
            color = senseColor();
        }
        if (lightStrip != null) {
            switch (color) {
                case RED:
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                case BLUE:
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    break;
                case YELLOW:
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;
                default:
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }

        return color;
    }

    // Get color based on ARGB
    Color senseColor() {
        int a = sensor.alpha();
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        return analyzeDominance(a, r, g, b);
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
