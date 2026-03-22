package org.firstinspires.ftc.team417;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import android.graphics.Color;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class CoolColorDetector {
    public static double MAXIMUM_DISTANCE = 30;

    public static double MINIMUM_DISTANCE = 21;
    public static double PURPLE_MIN_HUE = 190;
    public static double PURPLE_MAX_HUE = 320; //increased from 235 to loosen requirement
    public static double GREEN_MIN_HUE = 100; //decreased from 150 to loosen requirement
    public static double GREEN_MAX_HUE = 170;
    public static double MIN_VALUE = 0.25;
    public static float GAIN = 85f; // adjust for brightness


    Telemetry telemetry;
    private NormalizedColorSensor sensor1;
    private NormalizedColorSensor sensor2;

    public CoolColorDetector(HardwareMap map, Telemetry telemetry) {
        sensor1 = map.get(NormalizedColorSensor.class, "sensorColor1");
        sensor2 = map.get(NormalizedColorSensor.class, "sensorColor2");
        sensor1.setGain(GAIN);
        sensor2.setGain(GAIN);
        this.telemetry = telemetry;
    }

    // --- Convert a sensor to ONE PixelColor ---
    @SuppressLint("DefaultLocale")
    // --- Use logic comparing both sensors ---
    PixelColor detectArtifactColor() {
        double distance1 = ((DistanceSensor) sensor1).getDistance(DistanceUnit.MM);
        double distance2 = ((DistanceSensor) sensor2).getDistance(DistanceUnit.MM);
        double minDistance;
        NormalizedColorSensor sensor;

//        if (distance1 < MAXIMUM_DISTANCE && distance1 > MINIMUM_DISTANCE && distance2 > MINIMUM_DISTANCE) {
//            sensor = sensor1;
//            minDistance = distance1;
//        } else {
//            sensor = sensor2;
//            minDistance = distance2;
//        }

//        if (minDistance > MAXIMUM_DISTANCE || minDistance < MINIMUM_DISTANCE) {
//            String string = String.format(" %.1f, %.1f\"", distance1, distance2);
//            telemetry.addLine(string);
//            Log.d("CoolColorDetector", string);
//            return PixelColor.NONE;
//        }
        NormalizedRGBA sensor1Color = sensor1.getNormalizedColors();
        NormalizedRGBA sensor2Color = sensor2.getNormalizedColors();

        float[] hsv1 = new float[3];
        Color.colorToHSV(sensor1Color.toColor(), hsv1);
        float hue1 = hsv1[0];
        float value1 = hsv1[2];

        float[] hsv2 = new float[3];
        Color.colorToHSV(sensor2Color.toColor(), hsv2);
        float hue2 = hsv2[0];
        float value2 = hsv2[2];

        float averagedHue = (hue1 + hue2)/2;

//        String colorCube = String.format("<font color='#%06x'>\u25a0\u25a0\u25a0</font>",
//                averagedColor.toColor() & 0xffffff);

        PixelColor result = PixelColor.PURPLE;
            if (averagedHue > GREEN_MIN_HUE && averagedHue < GREEN_MAX_HUE) { //range determined from testing
                result = PixelColor.GREEN;
            } else if (averagedHue >= PURPLE_MIN_HUE && averagedHue <= PURPLE_MAX_HUE) { //range determined from testing
                result = PixelColor.PURPLE;
            }

//        String string = String.format("%.1f/%.1fmm %s H: %.1f V: %.2f %s",
//                distance1, distance2, colorCube, hue, value, result);

//        telemetry.log().add(string);
//        telemetry.addLine(string);
//        Log.d("CoolColorDetector", string);
        // Return the result that was decided in the if statements above
        return result;


    }

    public void testTelemetry() {

        telemetry.addData("Chosen Position", detectArtifactColor());
        telemetry.update();
    }
}

