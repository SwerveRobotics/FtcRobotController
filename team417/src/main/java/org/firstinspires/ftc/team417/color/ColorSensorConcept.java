package org.firstinspires.ftc.team417.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

// Concept for the color sensor, not for OpenCV.
@TeleOp(name = "Color Sensor", group = "Concept")
@Config
public class ColorSensorConcept extends LinearOpMode {
    NormalizedColorSensor sensor;
    RevBlinkinLedDriver lightStrip;

    @Override
    public void runOpMode() {
        Color color;

        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "indicatorLed");

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
                    telemetry.addLine("Detecting nothing!!!");
                    lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            telemetry.addData("Color", color);
            telemetry.update();
        }
    }

    // Get color based on ARGB
    Color senseColor() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double a = rgba.alpha;
        double r = rgba.red;
        double g = rgba.green;
        double b = rgba.blue;

        telemetry.addLine(String.format("ARGB color: {%f, %f, %f, %f}", a, r, g, b));

        double[] labArray = ColorConverter.rgbToLab(r, g, b);

        Lab labColor = new Lab(labArray[0], labArray[1], labArray[2]);

        telemetry.addData("Lab color", labColor.toString());

        if (labColor.equals(ColorProcessor.YELLOW, ColorProcessor.LAB_EPSILON)) {
            return Color.YELLOW;
        }

        if (labColor.equals(ColorProcessor.RED, ColorProcessor.LAB_EPSILON)) {
            return Color.RED;
        }

        if (labColor.equals(ColorProcessor.BLUE, ColorProcessor.LAB_EPSILON)) {
            return Color.BLUE;
        }

        return Color.UNDETECTED;
    }
}
