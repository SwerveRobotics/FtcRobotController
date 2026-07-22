package com.wilyworks.simulator.framework.mechsim;

import com.wilyworks.simulator.framework.WilyDcMotorEx;

// Let us ramp up the launcher motor velocity.
public class LaunchMotor extends WilyDcMotorEx {
    double targetVelocity;
    double actualVelocity;

    @Override
    public void setVelocity(double angularRate) {
        targetVelocity = angularRate;
    }

    @Override
    public double getVelocity() {
        return actualVelocity;
    }
}
