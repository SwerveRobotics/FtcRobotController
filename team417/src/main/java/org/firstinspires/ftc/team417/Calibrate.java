package org.firstinspires.ftc.team417;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        float gain = calibrateColorGain();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        print("Please insert a YELLOW Sample into the BOWG." +
                "\nPress A when finished.");

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab yellow = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        print("Please insert a BLUE Sample into the BOWG." +
                "\nPress A when finished.");

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab blue = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        print("Please insert a RED Sample into the BOWG." +
                "\nPress A when finished.");

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }

        Lab red = getLab();

        lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

        print("Calibration for the color sensor is complete. Congratulations!" +
                "\nPress A when to stop the program." +
                "\nYour LAB values for BLUE are: " + blue +
                "\nYour LAB values for YELLOW are: " + yellow +
                "\nYour LAB values for RED are: " + red +
                "\nYour gain value is: " + gain);

        while (!(gamepad1.a && !a1IsPressed)) {
            a1IsPressed = gamepad1.a;
        }
    }

    public float calibrateColorGain() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed;

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }

        // Loop until we are asked to stop
        while (gamepad1.y) {
            // Explain basic gain information via telemetry
            telemetry.addLine("Please insert a YELLOW sample into the BOWG.");
            telemetry.addLine("Your goal is to get one of R, G, and B close to about 0.8.\n");
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005f;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005f;
            }

            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            sensor.setGain(gain);

            // Check the status of the X button on the gamepad
            xButtonCurrentlyPressed = gamepad1.x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
                    if (sensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight) sensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = sensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (sensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) sensor).getDistance(DistanceUnit.CM));
            }

            telemetry.addLine("Press Y when finished.");

            telemetry.update();
        }

        return gain;
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
