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

    @Override
    public void runOpMode() {
        Color color;

        sensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {
            color = senseColor();
            telemetry.addData("Color", color);
            telemetry.update();
        }
    }

    public Color senseColor() {
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
