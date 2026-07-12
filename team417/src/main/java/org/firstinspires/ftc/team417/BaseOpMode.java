package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {
    LimelightAprilTagDetector detector;

    public static double ROBOT_WIDTH = 16.15;
    public static double ROBOT_LENGTH = 16.5;
    public static double MOTOR_D_VALUE = 1;

    // TODO: tune constants in ftc dashboard
    public static double FLYWHEEL_NEAR_SPEED = 650;
    public static double FLYWHEEL_FAR_SPEED = 850;
    public static double FLYWHEEL_BACKSPIN = 150;
    public static double WHEEL_STOP_SPEED = 0;
    public static double TRANSFER_WHEEL_START_SPEED = 312;
    public static double REVERSE_INTAKE_SPEED = 500;
    public static double INTAKE_SPEED_MULTIPLIER = 1000;


    protected DcMotorEx upperFlywheelMot;
    protected DcMotorEx lowerFlywheelMot;
    protected DcMotorEx transferWheelMot;
    protected DcMotorEx intakeMot;

    void initializeHardware() {
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
    void setFlywheelVelocity(double velocity) {
        upperFlywheelMot.setVelocity(velocity-FLYWHEEL_BACKSPIN);
        lowerFlywheelMot.setVelocity(velocity+FLYWHEEL_BACKSPIN);
    }

}


