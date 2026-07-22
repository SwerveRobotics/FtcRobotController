package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Wily Works DigitalChannel implementation.
 */
public class WilyDigitalChannel extends WilyHardwareDevice implements DigitalChannel {
    boolean state;

    @Override
    public Mode getMode() {
        return null;
    }

    @Override
    public void setMode(Mode mode) {
    }

    @Override
    public boolean getState() {
        return state;
    }

    @Override
    public void setState(boolean state) {
        this.state = state;
    }
}
