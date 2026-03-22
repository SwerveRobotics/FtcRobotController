package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Wily Works normalized color sensor implementation.
 */
public class WilyNormalizedColorSensor extends WilyHardwareDevice implements NormalizedColorSensor, DistanceSensor {
    @Override
    public NormalizedRGBA getNormalizedColors() {
        return new NormalizedRGBA();
    }

    @Override
    public float getGain() {
        return 0;
    }

    @Override
    public void setGain(float newGain) {
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return 0;
    }
}
