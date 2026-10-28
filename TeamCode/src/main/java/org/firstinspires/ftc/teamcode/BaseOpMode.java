package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
abstract public class BaseOpMode extends LinearOpMode {

    //motors;
    protected DcMotorEx launchMotor;
    protected DcMotorEx intakeMotor;
    protected Servo gate;
    protected Servo flowerRamp;
    //protected Limelight3A limelight3A;



    //constants
    public static double LAUNCH_TIME = 500.0;
    //TODO: uh
    public static double LAUNCHER_SPEED = 1400;//1250
    public static double INTAKE_SPEED = 1.0;
    public static double GATE_OPEN = .8;
    public static double GATE_CLOSE = 0.52;
    public static double RAMP_UP = 0.14;//.14
    public static double RAMP_DOWN = 0.86;

    //autoaim TODO: tune these values

    public static double KP = 15;
    public static double KI = 0;
    public static double KD = 0;
    public static double KF = 16.3;


    public void initializeHardware() {
        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        launchMotor = hardwareMap.get(DcMotorEx.class, "motLaunch");
        gate = hardwareMap.get(Servo.class, "alliGATEr");
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");

        //limelight camera
        //limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        //set motors to be energized
        intakeMotor.setMotorEnable();
        launchMotor.setMotorEnable();

        //Run using encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));

        //what happens when stopping
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //set direction
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
