package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

/**
 * Wily Works AnalogInput implementation.
 */
@AnalogSensorType
@DeviceProperties(name = "@string/configTypeAnalogInput", xmlTag = "AnalogInput", builtIn = true)
public class WilyAnalogInput extends AnalogInput {
    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public double getMaxVoltage() {
        return 0;
    }
}
