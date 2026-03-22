package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Wily Works voltage sensor implementation.
 */
public class WilyVoltageSensor extends WilyHardwareDevice implements VoltageSensor {
    @Override
    public double getVoltage() {
        return 13.0;
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor";
    }
}
