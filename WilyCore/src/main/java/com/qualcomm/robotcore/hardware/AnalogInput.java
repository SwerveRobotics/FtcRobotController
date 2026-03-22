package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

/**
 * Control a single analog device
 */
@DeviceProperties(name = "@string/configTypeAnalogInput", xmlTag = "AnalogInput", builtIn = true)
public class AnalogInput implements HardwareDevice {
    @Override public Manufacturer getManufacturer()  { return Manufacturer.Other; }

    public double getVoltage() { return 0; }

    public double getMaxVoltage() { return 0; }

    @Override
    public String getDeviceName() { return ""; }

    @Override
    public String getConnectionInfo() { return ""; }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
        // take no action
    }
}
