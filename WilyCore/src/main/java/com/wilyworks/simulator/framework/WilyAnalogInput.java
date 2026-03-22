package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Wily Works AnalogInput implementation.
 */
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
