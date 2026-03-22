package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Wily Works Servo implementation.
 */
public class WilyServo extends WilyHardwareDevice implements Servo {
    double position;

    @Override
    public ServoController getController() {
        return new WilyServoController();
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPosition(double position) {
        this.position = Math.max(0, Math.min(1, position));
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void scaleRange(double min, double max) {

    }
}
