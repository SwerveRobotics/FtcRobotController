package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Wily Works distance sensor implementation.
 */
public class WilyDistanceSensor extends WilyHardwareDevice implements DistanceSensor {
    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromMm(65535);
    } // Distance when not responding
}
