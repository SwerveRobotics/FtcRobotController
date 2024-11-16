package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlManager {

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    private int armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_INIT;
    private int slidesMotorPosition;
    private double intakeServoPower;
    private double dumperServoPosition;
    private double armElbowServoPosition;


    public ControlManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update() {
        if(gamepad2.a && !gamepad2.b) {
            armElbowServoPosition =  DRIFTConstants.ARM_ELBOW_SERVO_POSITION_OVER_BAR;
        }
        if(gamepad2.b && !gamepad2.a) {
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_GROUND;
        }
        if(gamepad2.x && !gamepad2.y) {
            armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_GROUND;
        }
        if (gamepad2.y && !gamepad2.x) {
            armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_INIT;
        }
        intakeServoPower = -gamepad2.left_stick_y;
        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            slidesMotorPosition = DRIFTConstants.SLIDES_MOTOR_GROUND_POSITION;
        }
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            slidesMotorPosition = DRIFTConstants.SLIDES_MOTOR_POSITION_ONE;
        }
        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_OVER_BAR;

        }
    }

    public int getArmBaseMotorPosition() {
        return armBaseMotorPosition;
    }

    public int getSlidesMotorPosition() {
        return slidesMotorPosition;
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
