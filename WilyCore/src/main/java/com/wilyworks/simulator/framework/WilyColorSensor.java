package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Wily Works color sensor implementation.
 */
public class WilyColorSensor extends WilyHardwareDevice implements ColorSensor {
    @Override
    public int red() {
        return 0;
    }

    @Override
    public int green() {
        return 0;
    }

    @Override
    public int blue() {
        return 0;
    }

    @Override
    public int alpha() {
        return 0;
    }

    @Override
    public int argb() {
        return 0;
    }

    @Override
    public void enableLed(boolean enable) {
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
    }

    @Override
    public I2cAddr getI2cAddress() {
        return null;
    }
}
