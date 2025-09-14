package com.wilyworks.simulator.framework;


import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Wily Works device subclass implementation.
 */
public class WilyHardwareDevice implements HardwareDevice {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() { }

    @Override
    public void close() { }
}
