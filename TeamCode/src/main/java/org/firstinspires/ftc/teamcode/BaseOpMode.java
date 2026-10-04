package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

abstract public class BaseOpMode extends LinearOpMode {

    //motors;
    protected DcMotorEx launchMotor;
    protected DcMotorEx intakeMotor;


    //constants
    public static double LAUNCHER_SPEED = 1.0;
    public static double INTAKE_SPEED = 1.0;
    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSE = 0.0;


    public void initializeHardware() {
        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motIntake");
        launchMotor = hardwareMap.get(DcMotorEx.class, "motLaunch");

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
