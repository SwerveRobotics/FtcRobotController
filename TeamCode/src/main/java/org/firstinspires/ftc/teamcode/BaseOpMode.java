package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;


@Config
abstract public class BaseOpMode extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE,
    }

    //motors;
    protected DcMotorEx launchMotor;
    protected DcMotorEx intakeMotor;
    protected Servo gate;
    protected Servo flowerRamp;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;



    //constants
    public static double LAUNCH_TIME = 350.0;
    //TODO: uh
    public static double LAUNCHER_SPEED = 1360;//1250
    public static double AUTO_LUANCHER_TIME = 3000;
    public static double INTAKE_SPEED = 1.0;
    public static double GATE_OPEN = .8;
    public static double GATE_CLOSE = 0.52;
    public static double RAMP_UP = 0.3;//.14
    public static double RAMP_DOWN = 0.88;

    //autoaim TODO: tune these values

    public static double KP = 150;
    public static double KI = 0;
    public static double KD = 5;
    public static double KF = 15;
    // For seeing if hive is down or not
    public boolean HIVE_DOWN = true;


    protected boolean IS_HIVE_SCORABLE () {
        LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                boolean foundScorableTarget = false;
                double targetErrorX = 0;
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    foundScorableTarget = true;
                    targetErrorX = tag.getTargetXDegrees();
                    telemetry.addData("Horizontal Error (tx)", targetErrorX);

                }
                if (!foundScorableTarget ) {
                    telemetry.addData("status","Target Not Found");
                }
                return foundScorableTarget;
            }
            else {
                telemetry.addData("status", "No Valid Targets Visible");
            }
            return false;
    }


    public void initializeHardware() {
        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        launchMotor = hardwareMap.get(DcMotorEx.class, "motLaunch");
        gate = hardwareMap.get(Servo.class, "alliGATEr");
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        //limelight camera

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

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
