package org.firstinspires.ftc.team417.utils;

import static java.lang.System.nanoTime;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.CoolColorDetector;

@TeleOp(name = "Twin Color Sensor Test", group = "Sensor")
public class TwinColorTest extends LinearOpMode {
    final double VELOCITY = 30;
    double time = 0;

    // Shape the input:
    double halfLinearHalfCubic(double input){
        return (Math.pow(input, 3) + input) / 2;
    }

    double process(String id, NormalizedColorSensor sensor, double gain, double stick) {
        // Update the gain value according to the stick velocity:
        double dt = Math.min(nanoTime() * 1e-9 - time, 0.200);
        time = nanoTime() * 1e-9;
        gain += halfLinearHalfCubic(stick) * dt * VELOCITY;
        sensor.setGain((float) gain);

        // Get the distance and colors:
        double distance = ((DistanceSensor) sensor).getDistance(DistanceUnit.MM);
        NormalizedRGBA colors = sensor.getNormalizedColors();
        String rgbString = String.format("<font color='#%06x'>\u25a0</font>",
                colors.toColor() & 0xffffff);
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        // This will take just the Hue value and draw it at full brightness:
        int hueColor = Color.HSVToColor(new float[]{hsv[0], 1, 1});
        String hueString = String.format("<font color='#%06x'>\u25a0</font>",
                hueColor & 0xffffff);

        telemetry.addLine(String.format("%s(%.1f): %.1fmm %s H: %.1f (%s) SV: %.2f, %.2f",
                id, gain, distance, rgbString, hsv[0], hueString, hsv[1], hsv[2]));
        Log.d("TwinColorTest", String.format("%s(%.1f): %.1fmm 0x%06x, H: %.1f SV: %.2f, %.2f",
                id, gain, distance, colors.toColor(), hsv[0], hsv[1], hsv[2]));
        return gain;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor sensor1;
        NormalizedColorSensor sensor2;
        double gain1 = CoolColorDetector.GAIN; // Brushlands
        double gain2 = CoolColorDetector.GAIN; // REV

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        sensor1 = hardwareMap.get(NormalizedColorSensor .class, "sensorColor1"); // Brushlands
        sensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensorColor2"); // REV

        // Wait for the start button to be pressed.
        telemetry.addLine("Ready to start Twin Color Sensor test.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Gains are in parentheses. Left stick to adjust gain #1, right stick for gain #2.\n");
            gain1 = process("1", sensor1, gain1, -gamepad1.left_stick_y);
            gain2 = process("2", sensor2, gain2, -gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
