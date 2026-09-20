package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

abstract public class BaseOpMode extends LinearOpMode {
    // Constants
    public static double MOTOR_D_VALUE = 1;

    //Motors/Servos
    public static double LAUNCHER_SPEED = 500;
    public static double LAUNCHER_BACKSPIN = 150;
    public static double WHEEL_STOP_SPEED = 0;
    public static double TRANSFER_WHEEL_START_SPEED = 312;
    public static double INTAKE_SPEED_MULTIPLIER = 1000;
    public static double TRANSFER_SPEED = 100;

    protected DcMotorEx transferWheelMot;
    protected DcMotorEx lowerFlywheelMot;
    protected DcMotorEx upperFlywheelMot;
    protected DcMotorEx intakeMot;

    public void initializeHardware() {
        // Hardware map initialization
        upperFlywheelMot = hardwareMap.get(DcMotorEx.class, "motULauncher");
        lowerFlywheelMot = hardwareMap.get(DcMotorEx.class, "motLLauncher");
        transferWheelMot = hardwareMap.get(DcMotorEx.class, "feedLaunch");
        intakeMot = hardwareMap.get(DcMotorEx.class, "motIntake");

        // Initializing motor behaviors
        upperFlywheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lowerFlywheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transferWheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        upperFlywheelMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerFlywheelMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PID coefficients for flywheel
        upperFlywheelMot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));
        lowerFlywheelMot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));

        // Set directions of motors
        upperFlywheelMot.setDirection(DcMotor.Direction.REVERSE);
        lowerFlywheelMot.setDirection(DcMotor.Direction.REVERSE);
        transferWheelMot.setDirection(DcMotor.Direction.REVERSE);
        intakeMot.setDirection(DcMotor.Direction.FORWARD);

    }

}
