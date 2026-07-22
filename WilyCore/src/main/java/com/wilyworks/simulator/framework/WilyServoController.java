package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Wily Works ServoController implementation.
 */
public class WilyServoController extends WilyHardwareDevice implements ServoController {
    @Override
    public void pwmEnable() {
    }

    @Override
    public void pwmDisable() {
    }

    @Override
    public PwmStatus getPwmStatus() {
        return PwmStatus.DISABLED;
    }

    @Override
    public void setServoPosition(int servo, double position) {
    }

    @Override
    public double getServoPosition(int servo) {
        return 0;
    }

    @Override
    public void close() {
    }
}
