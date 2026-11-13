package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Config
abstract public class BaseOpMode extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE,
    }
    public enum UpCell {
        AUDIENCE,
        SCORING_TABLE,
        UNKNOWN }
    //motors;
    protected DcMotorEx launchMotor1;
    protected DcMotorEx launchMotor2;
    protected DcMotorEx intakeMotor;
    protected DcMotorEx transferMotor;
    protected Servo turretServo;
    protected Servo gate;
    protected Servo flowerRamp;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Alliance chosenAlliance;
    protected Follower follower;


    protected UpCell upCell = UpCell.UNKNOWN; //which cell is up, step 1 sets this
    protected boolean turretAutoAim = true;
    private double turretCommandedDeg = 180;
    private double turretTargetDeg = 180;
    private boolean turretAtLimit = false;
    private final ElapsedTime turretTimer = new ElapsedTime();
    //constants
    public static double LAUNCH_TIME = 350.0;
    //TODO: uh
    public static double LAUNCHER_SPEED = 1360;//1250
    public static double AUTO_LAUNCHER_TIME = 3000;
    public static double INTAKE_SPEED = 1.0;
    public static double GATE_OPEN = .8;
    public static double TRANSFER_SPEED = 1500;


    public static double RAMP_UP = 0.3;//.14
    public static double RAMP_DOWN = 0.88;

    //autoaim TODO: tune these values

    public static double KP = 150;
    public static double KI = 0;
    public static double KD = 5;
    public static double KF = 15;
    // For seeing if hive is down or not
    public boolean hiveDown = true;
//-------------------turret constants-----------------------------------------------
    //turret stuff, all in servo degrees (servo 0 to 1 = 0 to 360)
    public static double TURRET_LEFT_END;
    public static double TURRET_RIGHT_END;
    public static double TURRET_CENTER_DEG = 180; //servo angle when the turret is straight
    public static double TURRET_STRAIGHT_OFFSET_DEG = 180; //where the turret shoots at center, from robot front, ccw is positive. 180 = out the back
    public static double TURRET_DIR = 1; //flip to -1 if it turns the wrong way
    public static double TURRET_LIMIT_MARGIN_DEG = 5; //stay this far inside the real stops
    public static double TURRET_MAX_SPEED_DEG_PER_SEC = 300;

    //goal points in field inches, TODO check these
    public static double BLUE_GOAL_X = 84;
    public static double RED_GOAL_X = 58;
    public static double GOAL_Y_HIGH = 86.5;
    public static double GOAL_Y_LOW = 54.5;
    public static boolean AUDIENCE_CELL_IS_HIGH_Y = true; //TODO which y is which cell
//---------------------------------------------------------------
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

    public void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    //FUNCTION ASSUMES THAT THE SERVO'S LIMITS ARE FROM ZERO TO 360(we checked with hardware and they said it's right)


    //step 2: pick the goal point based on which cell is up
    private double[] getGoalPoint() {
        if (chosenAlliance == null || upCell == UpCell.UNKNOWN) return null; //dont know yettt
        double x = (chosenAlliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        boolean high = (upCell == UpCell.AUDIENCE) == AUDIENCE_CELL_IS_HIGH_Y;
        return new double[]{x,
                high ? GOAL_Y_HIGH : GOAL_Y_LOW};
    }

    //keeps an angle between -180 and 180
    private static double wrap180(double deg) {
        double d = deg % 360;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }

    //steps 3 and 4: where the turret should point (servo degrees), NaN if we cant aim yet
    private double getRoughAim(double poseX, double poseY, double headingRad) {
        //limits not set yet, dont move
        if (Math.abs(TURRET_RIGHT_END - TURRET_LEFT_END) < 20) return Double.NaN;

        double[] goal = getGoalPoint();
        if (goal == null) return Double.NaN;

        //angle from the robot to the goal on the field
        double goalAngle = Math.toDegrees(Math.atan2(goal[1] - poseY, goal[0] - poseX));
        //make it relative to the robot, then to where the turret shoots at center
        double relative = wrap180(goalAngle - Math.toDegrees(headingRad) - TURRET_STRAIGHT_OFFSET_DEG);
        double desired = TURRET_CENTER_DEG + TURRET_DIR * relative;

        //limits, a little inside the real stops
        double minDeg = Math.min(TURRET_LEFT_END, TURRET_RIGHT_END) + TURRET_LIMIT_MARGIN_DEG;
        double maxDeg = Math.max(TURRET_LEFT_END, TURRET_RIGHT_END) - TURRET_LIMIT_MARGIN_DEG;

    //same angle can show up as desired, desired + 360, desired - 360
        //pick the one that fits and is closest to where the turret is
        turretAtLimit = false;
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double cand = desired + 360 * k;
            if (cand >= minDeg && cand <= maxDeg) {
                if (Double.isNaN(best) || Math.abs(cand - turretCommandedDeg) < Math.abs(best - turretCommandedDeg)) {
                    best = cand;
                }
            }
        }
        //nothing fits = goal is in the dead zone, so go to the closest limit
        if (Double.isNaN(best)) {
            best = (Math.abs(wrap180(desired - minDeg)) <= Math.abs(wrap180(desired - maxDeg))) ? minDeg : maxDeg;
            turretAtLimit = true;
        }
        return best;
    }

    //step 6: call this every loop
    protected void updateTurret(double poseX, double poseY, double headingRad) {
        double dt = turretTimer.seconds();
        turretTimer.reset();
        if (dt <= 0 || dt > 0.25) dt = 0.02; //first loop or a lag spike

        //if auto aim is off or we cant aim, just hold where we are
        turretTargetDeg = turretCommandedDeg;
        if (turretAutoAim) {
            double rough = getRoughAim(poseX, poseY, headingRad);
            if (!Double.isNaN(rough)) turretTargetDeg = rough;
        }

        //move toward the target but not faster than the max speed
        double maxStep = TURRET_MAX_SPEED_DEG_PER_SEC * dt;
        double diff = turretTargetDeg - turretCommandedDeg;
        if (Math.abs(diff) <= maxStep) {
            turretCommandedDeg = turretTargetDeg;
        } else {
            turretCommandedDeg += Math.signum(diff) * maxStep;
        }

        turretServo.setPosition(Math.max(0, Math.min(1, turretCommandedDeg / 360.0)));
    }
    public void initializeHardware() {
        initializeLimelight();

        //initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        launchMotor1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launchMotor2 = hardwareMap.get(DcMotorEx.class, "launch2");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        //limelight camera

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //set motors to be energized
        intakeMotor.setMotorEnable();
        launchMotor1.setMotorEnable();
        launchMotor2.setMotorEnable();

        transferMotor.setMotorEnable();

        //Run using encoder
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
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
        turretServo.setPosition(targetHeading / 360);
        telemetry.addData("Target Relative Turret Position", targetHeading);


        //starts the turret and center degree
        turretCommandedDeg = TURRET_CENTER_DEG;
        turretServo.setPosition(TURRET_CENTER_DEG / 360.0);
        turretTimer.reset(); //for time
    }
    public void setTurretVelocity(double velocity) {
        launchMotor1.setVelocity(velocity);
        launchMotor2.setVelocity(velocity);
    }
    public boolean turretIsFast() {
        return launchMotor1.getVelocity() > 0.95 * LAUNCHER_SPEED && launchMotor2.getVelocity() > 0.95*LAUNCHER_SPEED;
    }
    public double avgTurretSpeed() {
        return (launchMotor2.getVelocity() + launchMotor1.getVelocity())/2.0;
    }
    public void intakeAndTransfer(boolean on) {
        if (on) {
            intakeMotor.setVelocity(INTAKE_SPEED);
            transferMotor.setVelocity(TRANSFER_SPEED);
        } else {
            intakeMotor.setVelocity(0);
            transferMotor.setVelocity(0);
        }
    }
    public void printTelemetry() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake speed", intakeMotor.getVelocity());
        telemetry.addData("Transfer speed", transferMotor.getVelocity());
        telemetry.addData("Average Turret Speed", avgTurretSpeed() );
        telemetry.addData("Turret position:", turretServo.getPosition());
        telemetry.addData("Ramp Position",flowerRamp.getPosition() == RAMP_UP ? "Up":"Down");
        telemetry.addData("Is Hive Scorable", IS_HIVE_SCORABLE());
    }
}
