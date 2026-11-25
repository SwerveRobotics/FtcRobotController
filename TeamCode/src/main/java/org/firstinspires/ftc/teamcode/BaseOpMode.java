package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
//import com.google.blocks.ftcrobotcontroller.runtime.HardwareAccess;
//import com.google.blocks.ftcrobotcontroller.runtime.Limelight3AAccess;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.procedures.Turret;

import java.util.List;
import java.util.Set;


@Config
abstract public class BaseOpMode extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }
    public enum UpCell {
        AUDIENCE,
        SCORING_TABLE,
        UNKNOWN }
    //motors;
    protected DcMotorEx launchMotor1;
    protected DcMotorEx launchMotor2;
    protected Servo turretServo;
    protected Servo turretServo2 ;
    protected DcMotorEx intakeMotor;
    protected DcMotorEx transferMotor;
    public double BLUE_GOAL_X = 84;
    public double RED_GOAL_X = 58;
    public double GOAL_Y_HIGH = 86.5;
    public double GOAL_Y_LOW = 54.5;

    protected Servo gate;
    //protected Servo flowerRamp;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Alliance chosenAlliance;
    protected Follower follower;
//67

    protected UpCell upCell = UpCell.UNKNOWN; //which cell is up, step 1 sets this
    //to TEST turret servo in teleop
    public Turret turret;
    //constants
    public static double LAUNCH_TIME = 1000.0;
    public static double INTAKE_SPEED = 1.0;


    public static double VELOCITY_1 = 950*1.4;
    public static double VELOCITY_2 = 600*1.4;
    public static double TRANSFER_INTAKE_SPEED = 0.5;

    public static double GATE_OPEN = 0;
    public static double GATE_CLOSED = 0.13;

    public static double TRANSFER_SPEED = 1.0;//1


    public static double RAMP_UP = .7;//.14
    public static double RAMP_DOWN = .23;

    //autoaim TODO: tune these values

    public static double KP = 150;
    public static double KI = 0;
    public static double KD = 5;
    public static double KF = 15;
    // For seeing if hive is down or not
    public boolean hiveDown = true;

    public static boolean AUDIENCE_CELL_IS_HIGH_Y = true; //TODO which y is which cell

    public final List<Integer> RED_LIST = List.of(30, 31, 32, 33, 34, 35, 36, 37);
    public final List<Integer> BLUE_LIST =List.of(38, 39, 40, 41, 42, 43, 44, 45);
//---------------------------------------------------------------
    //public Limelight3AAccess extends limelight;
    protected boolean IS_HIVE_SCORABLE () {
        LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                boolean foundScorableTarget = false;
                double targetErrorX = 0;
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    double distMM = distanceToTag(tag);
                    if (distMM > 0.85) { //dist of 800 or less is down, dist of 900 or more is up
                        if (tagBelongs(chosenAlliance, id)) {
                            foundScorableTarget = true;
                        } else {
                            telemetry.addLine("wrong team detected");
                        }
                    }
                    targetErrorX = tag.getTargetXDegrees();
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

//    public void initializeLimelight() {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
//        limelight.start();
//    }


    //step 2: pick the goal point based on which cell is up


    //keeps an angle between -180 and 180
    private static double wrap180(double deg) {
        double d = deg % 360;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }
    private double[] getGoalPoint() {
        if (chosenAlliance == null || upCell == UpCell.UNKNOWN) return null; //dont know yettt
        double x = (chosenAlliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        boolean high = (upCell == UpCell.AUDIENCE) == AUDIENCE_CELL_IS_HIGH_Y;
        return new double[]{x,
                high ? GOAL_Y_HIGH : GOAL_Y_LOW};
    }

    //steps 3 and 4: where the turret should point (servo degrees), NaN if we cant aim yet
    private double getRoughAim(double poseX, double poseY, double headingRad) {
        Set<List<Integer>> ignoredTags;
        if (chosenAlliance == Alliance.BLUE) {
            ignoredTags = Set.of(RED_LIST);
        } else if (chosenAlliance == Alliance.RED) {
            ignoredTags = Set.of(BLUE_LIST);
        }
        //limits not set yet, dont move

        double[] goal = getGoalPoint();
        if (goal == null) return Double.NaN;

        //angle from the robot to the goal on the field
        double goalAngle = Math.toDegrees(Math.atan2(goal[1] - poseY, goal[0] - poseX));
//        //make it relative to the robot, then to where the turret shoots at center67
//        double relative = wrap180(goalAngle - Math.toDegrees(headingRad) - TURRET_STRAIGHT_OFFSET_DEG);
//        double desired = TURRET_CENTER_DEG + TURRET_DIR * relative;
//
//        //limits, a little inside the real stops
//        double minDeg = Math.min(TURRET_LEFT_END, TURRET_RIGHT_END) + TURRET_LIMIT_MARGIN_DEG;
//        double maxDeg = Math.max(TURRET_LEFT_END, TURRET_RIGHT_END) - TURRET_LIMIT_MARGIN_DEG;
//
//        //same angle can show up as desired, desired + 360, desired - 360
//        //pick the one that fits and is closest to where the turret is
//        turretAtLimit = false;
//        double best = Double.NaN;
//        for (int k = -1; k <= 1; k++) {
//            double cand = desired + 360 * k;
//            if (cand >= minDeg && cand <= maxDeg) {
//                if (Double.isNaN(best) || Math.abs(cand - turretCommandedDeg) < Math.abs(best - turretCommandedDeg)) {
//                    best = cand;
//                }
//            }
//        }
//        //nothing fits = goal is in the dead zone, so go to the closest limit
//        if (Double.isNaN(best)) {
//            best = (Math.abs(wrap180(desired - minDeg)) <= Math.abs(wrap180(desired - maxDeg))) ? minDeg : maxDeg;
//            turretAtLimit = true;
//        }
        //return best;
        return goalAngle;
    }

    public void initializeHardware() {
        //initializeLimelight();
        launchMotor1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launchMotor2 = hardwareMap.get(DcMotorEx.class, "launch2");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turret = new Turret(launchMotor1, launchMotor2, turretServo, turretServo2);
        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");

        gate = hardwareMap.get(Servo.class, "gate");
        //flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        //limelight camera

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //set motors to be energized
        intakeMotor.setMotorEnable();
        launchMotor1.setMotorEnable();
        launchMotor2.setMotorEnable();

        transferMotor.setMotorEnable();

        //Run using encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));
        launchMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));

        //what happens when stopping
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    //remembers what we last sent to the hub so we only resend when a value changes
    public void intakeAndTransfer(boolean on) {
        if (on) {
            intakeMotor.setPower(INTAKE_SPEED);
            transferMotor.setPower(0.5 * TRANSFER_SPEED);
        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }
    }
    public void printTelemetry() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake speed", intakeMotor.getVelocity());
        telemetry.addData("Transfer speed", transferMotor.getPower());
        telemetry.addData("Average Turret Speed", turret.avgSpeed() );
        telemetry.addData("launch Motor 1: ", launchMotor1.getVelocity());

        telemetry.addData("launch Motor 2: ", launchMotor2.getVelocity());
        telemetry.addData("Turret position", turretServo.getPosition());
        telemetry.addData("Gate Position", (gate.getPosition() == GATE_OPEN)?"Open":"Closed");
        //telemetry.addData("Ramp Position",flowerRamp.getPosition() == RAMP_UP ? "Up":"Down");
        //telemetry.addData("Is Hive Scorable", IS_HIVE_SCORABLE());
    }
}
