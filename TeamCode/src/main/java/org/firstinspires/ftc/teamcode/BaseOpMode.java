package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Config
abstract public class BaseOpMode extends LinearOpMode {

    //motors;
    protected DcMotorEx launchMotor;
    protected DcMotorEx intakeMotor;
    protected Servo gate;
    protected Servo flowerRamp;



    //constants
    public static double LAUNCH_TIME = 500.0;
    public static double LAUNCHER_SPEED = 500.0;
    public static double INTAKE_SPEED = 500.0;
    public static double GATE_OPEN = 1.0;
    public static double GATE_CLOSE = 0.0;
    public static double RAMP_UP = 0.39;
    public static double RAMP_DOWN = 1.0;


    public void initializeHardware() {
        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        launchMotor = hardwareMap.get(DcMotorEx.class, "motLaunch");
        gate = hardwareMap.get(Servo.class, "alliGATEr");
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");

        //set motors to be energized
        intakeMotor.setMotorEnable();
        launchMotor.setMotorEnable();
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //what happens when stopping
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //set velocity
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set direction
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
