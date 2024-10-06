package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlManager {

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    private double armBaseMotorPower;
    private double slidesMotorPower;
    private double intakeServoPower;
    private double dumperServoPosition;
    private double armElbowServoPosition;


    public ControlManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update() {
        if(gamepad2.a) {
            armElbowServoPosition =  Constants.ARM_ELBOW_SERVO_PRESET_POSITION_OVER_BARRIER;
        }
        if(gamepad2.b) {
            armElbowServoPosition = Constants.ARM_ELBOW_SERVO_PRESET_POSITION_INTAKE_TO_GROUND;
        }
        if(gamepad2.x) {
            armBaseMotorPower = Constants.ARM_BASE_MOTOR_POWER_BACK_TO_GROUND;

        }
    }

    public double getArmBaseMotorPower() {
        return armBaseMotorPower;
    }

    public double getSlidesMotorPower() {
        return slidesMotorPower;
    }

    public double getIntakeServoPower() {
        return intakeServoPower;
    }

    public double getDumperServoPosition() {
        return dumperServoPosition;
    }

    public double getArmElbowServoPosition() {
        return armElbowServoPosition;
    }
}
