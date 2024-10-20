package org.firstinspires.ftc.team417.concepts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

enum Color {
    RED,
    YELLOW,
    BLUE,
    UNDETECTED
}

// Concept for the color sensor, not for OpenCV.
@TeleOp(name = "Color Sensor", group = "Concept")
@Config
public class ColorSensorConcept extends LinearOpMode {
    ColorSensor sensor;
    Color color;

    @Override
    public void runOpMode() {
        sensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {
            int a = sensor.alpha();
            int r = sensor.red();
            int g = sensor.green();
            int b = sensor.blue();

            if (a < 200) { // If the detection is too transparent
                color = Color.UNDETECTED;
            } else {
                if (r >= g && r >= b) {
                    color = Color.RED; // Most red
                } else if (g >= r && g >= b) {
                    color = Color.YELLOW; // Most green
                } else {
                    color = Color.BLUE; // Most blue
                }
            }

            telemetry.addLine(String.format("(a, r, g, b) = (%d, %d, %d, %d)", a, r, g, b));
            telemetry.addData("Color", color);
            telemetry.update();
        }
    }
}
