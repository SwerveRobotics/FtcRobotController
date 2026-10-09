package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

abstract public class BaseOpMode extends LinearOpMode {
    // Constants
    public static double MOTOR_D_VALUE = 1;

    //Motors/Servos
    public static double LAUNCHER_SPEED = 500;
    public static double LAUNCHER_BACKSPIN = 150;
    public static double LAUNCHER_TOPSPIN = 150;
    public static double WHEEL_STOP_SPEED = 0;
    public static double TRANSFER_WHEEL_START_SPEED = 312;
    public static double INTAKE_SPEED_MULTIPLIER = 1000;
    public static double TRANSFER_SPEED = 100;
    public static double BALLS_TO_LAUNCH_TIME = 10;
    public static double FOUR_BALLS_TIME = 10;
    public static double PARTNER_LAUNCH_TIME= 10;
    protected DcMotorEx transferWheelMot;
    protected DcMotorEx lowerFlywheelMot;
    protected DcMotorEx upperFlywheelMot;
    protected DcMotorEx intakeMot;
    protected CRServo leftIntakeServo;
    protected CRServo rightIntakeServo;

    protected DcMotorEx FLMotor;
    protected DcMotorEx FRMotor;
    protected DcMotorEx BLMotor;
    protected DcMotorEx BRMotor;

    public void initializeHardware() {
        // Hardware map initialization
        //upperFlywheelMot = hardwareMap.get(DcMotorEx.class, "motULauncher");
        //lowerFlywheelMot = hardwareMap.get(DcMotorEx.class, "motLLauncher");
//        transferWheelMot = hardwareMap.get(DcMotorEx.class, "feedLaunch");
        intakeMot = hardwareMap.get(DcMotorEx.class, "motIntake");

        rightIntakeServo = hardwareMap.get(CRServo.class, "rightservo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftservo");

        FLMotor = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FRMotor = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BackRight");


        // Initializing motor behaviors
        //upperFlywheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //lowerFlywheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        transferWheelMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        //upperFlywheelMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lowerFlywheelMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set PID coefficients for flywheel
        //upperFlywheelMot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));
        //lowerFlywheelMot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));

        // Set directions of motors
        //upperFlywheelMot.setDirection(DcMotor.Direction.REVERSE);
        //lowerFlywheelMot.setDirection(DcMotor.Direction.REVERSE);
//        transferWheelMot.setDirection(DcMotor.Direction.REVERSE);
        intakeMot.setDirection(DcMotor.Direction.FORWARD);

        leftIntakeServo.setDirection(CRServo.Direction.REVERSE);

        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);


    }

}
