package org.firstinspires.ftc.team417;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.team417.color.ColorConverter;
import org.firstinspires.ftc.team417.color.Lab;

@Autonomous(name = "Calibrate")
public class Calibrate extends LinearOpMode {
    NormalizedColorSensor sensor;
    RevBlinkinLedDriver lightStrip;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        calibrateColor();

        // TODO: Calibrate camera
    }

    boolean a1IsPressed = false;

    private void calibrateColor() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "indicatorLed");

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        print("Please insert a YELLOW Sample into the BOWG." +
                "\nPress A when finished.");

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab yellow = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        print("Please insert a BLUE Sample into the BOWG." +
                "\nPress A when finished." +
                "Your LAB values for YELLOW are: " + yellow);

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab blue = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        print("Please insert a RED Sample into the BOWG." +
                "\nPress A when finished." +
                "Your LAB values for BLUE are: " + blue);

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab red = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        print("Calibration for the color sensor is complete." +
                "\nPress A when finished." +
                "Your LAB values for RED are: " + red);

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }
    }

    private Lab getLab() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        double a = rgba.alpha;
        double r = rgba.red;
        double g = rgba.green;
        double b = rgba.blue;

        double[] labArray = ColorConverter.rgbToLab(r, g, b);

        return new Lab(labArray[0], labArray[1], labArray[2]);
    }

    private void print(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }
}
