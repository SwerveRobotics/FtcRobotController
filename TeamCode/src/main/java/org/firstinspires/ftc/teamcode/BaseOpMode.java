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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
    protected DcMotorEx transferMotor;
    protected Servo turretServo;
    protected Servo gate;
    protected Servo flowerRamp;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Alliance chosenAlliance;



    //constants
    public static double LAUNCH_TIME = 350.0;
    //TODO: uh
    public static double LAUNCHER_SPEED = 1360;//1250
    public static double AUTO_LAUNCHER_TIME = 3000;
    public static double INTAKE_SPEED = 1.0;
    public static double GATE_OPEN = .8;
    public static double GATE_CLOSE = 0.52;

    public static double TURRET_LEFT_END;
    public static double TURRET_RIGHT_END;
    public static double RAMP_UP = 0.3;//.14
    public static double RAMP_DOWN = 0.88;

    //autoaim TODO: tune these values

    public static double KP = 150;
    public static double KI = 0;
    public static double KD = 5;
    public static double KF = 15;
    // For seeing if hive is down or not
    public boolean hiveDown = true;



    protected boolean isHiveScorable () {
        LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                boolean foundScorableTarget = false;
                double targetErrorX = 0;
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    double distMM = distanceToTag(tag);
                    if (distMM > 850) { //dist of 800 or less is down, dist of 900 or more is up
                        if (tagBelongs(chosenAlliance, id)) {
                            foundScorableTarget = true;
                        }
                    }
                    targetErrorX = tag.getTargetXDegrees();
                    telemetry.addData("April Tag ID", id);
                    telemetry.addData("Horizontal Error (tx)", targetErrorX);
                    telemetry.addData("Distance to camera", distanceToTag(tag));

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
    protected double distanceToTag(LLResultTypes.FiducialResult tag) {
        Position x = tag.getTargetPoseCameraSpace().getPosition();
        return distanceToOrigin(x);
    }
    protected double distanceToPosition(Position x, Position y) {
        return Math.sqrt((x.x-y.x)*(x.x-y.x)+(x.y-y.y)*(x.y-y.y)+(x.z-y.z)*(x.z-y.z));
    }
    protected double distanceToOrigin(Position x) {
        return Math.sqrt(x.x*x.x + x.y*x.y + x.z*x.z);
    }
    protected boolean tagBelongs(Alliance team, int id) {
        boolean tagBlue = id > 37;
        boolean teamBlue = team.equals(Alliance.BLUE);
        return tagBlue == teamBlue;
    }

    public void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }


    public void initializeHardware() {
        initializeLimelight();

        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        launchMotor = hardwareMap.get(DcMotorEx.class, "motLaunch");
        gate = hardwareMap.get(Servo.class, "alliGATEr");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        //limelight camera


        //set motors to be energized
        intakeMotor.setMotorEnable();
        launchMotor.setMotorEnable();
        transferMotor.setMotorEnable();

        //Run using encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));

        //what happens when stopping
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //FUNCTION ASSUMES THAT THE SERVO'S LIMITS ARE FROM ZERO TO 360(we checked with hardware and they said it's right)
    public void turnTurretAbsolute(double robotHeading, double targetHeading) { //takes [0,360], [0,360]
        double turretRelative = targetHeading - robotHeading;
        if (turretRelative < 0) {
            turretRelative += 360;
        }
        turnTurretRelative(turretRelative);
    }
    public void turnTurretRelative(double targetHeading) {
        turretServo.setPosition(targetHeading/360);
        telemetry.addData("Target Relative Turret Position", targetHeading);
    }

}
