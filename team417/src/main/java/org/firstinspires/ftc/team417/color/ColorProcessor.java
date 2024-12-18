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
