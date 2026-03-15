package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Wily Works distance sensor implementation.
 */
@I2cDeviceType
public class WilyDistanceSensor extends WilyHardwareDevice implements DistanceSensor {
    @Override
    public double getDistance(DistanceUnit unit) {
        return unit.fromMm(65535);
    } // Distance when not responding
}
