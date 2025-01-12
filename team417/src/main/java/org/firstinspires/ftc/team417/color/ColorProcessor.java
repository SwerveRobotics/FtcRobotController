package org.firstinspires.ftc.team417.color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorProcessor {
    NormalizedColorSensor sensor;
    RevBlinkinLedDriver lightStrip;

    public static final float GAIN = 17f;

    public static final Lab YELLOW = new Lab(78, -43, 42);
    public static final Lab BLUE = new Lab(48, -14, -22);
    public static final Lab RED = new Lab(50, -15, 18);

    public ColorProcessor(NormalizedColorSensor sensor, RevBlinkinLedDriver lightStrip) {
        this.sensor = sensor;
        sensor.setGain(GAIN);
        this.lightStrip = lightStrip;
        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
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

    public static final double LAB_EPSILON = 20;

    // Get color based on ARGB
    Color senseColor() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double a = rgba.alpha;
        double r = rgba.red;
        double g = rgba.green;
        double b = rgba.blue;

        double[] labArray = ColorConverter.rgbToLab(r, g, b);

        Lab labColor = new Lab(labArray[0], labArray[1], labArray[2]);

        if (labColor.equals(YELLOW, LAB_EPSILON)) {
            return Color.YELLOW;
        }

        if (labColor.equals(RED, LAB_EPSILON)) {
            return Color.RED;
        }

        if (labColor.equals(BLUE, LAB_EPSILON)) {
            return Color.BLUE;
        }

        return Color.UNDETECTED;
    }
}
