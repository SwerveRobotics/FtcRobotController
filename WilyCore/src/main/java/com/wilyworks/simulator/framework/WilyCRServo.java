package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Wily Works CRServo implementation.
 */
public class WilyCRServo extends WilyHardwareDevice implements CRServo {
    double power;
    Direction direction;

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
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }
}
