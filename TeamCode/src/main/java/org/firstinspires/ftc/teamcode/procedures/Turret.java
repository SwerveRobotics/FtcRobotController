package org.firstinspires.ftc.teamcode.procedures;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;
@Config
public class Turret { //67
    protected DcMotorEx launchMotor1;
    protected DcMotorEx launchMotor2;
    protected Servo turretServo;
    protected double TURRET_MAX_SPEED_DEG_PER_SEC;
    protected Servo turretServo2;
    public double SERVO_POSITION_FORWARD = 0.417;
    public double SERVO_POSITION_BACKWARD = 1;
    public double LEFT_END_DEGREES = -135;
    public double RIGHT_END_DEGREES = 180;
    public double VELOCITY_1 = 950*1.4;
    public double VELOCITY_2 = 600*1.4;
    public double TURRET_LEFT_END = 0.0;
    public double TURRET_RIGHT_END = 1.0;
    protected boolean turretAutoAim = true;
    private double turretCommandedDeg = 180;
    private double turretTargetDeg = 180;
    private boolean turretAtLimit = false;
    private final ElapsedTime turretTimer = new ElapsedTime();

    public double KP = 150;
    public double KI = 0;
    public double KD = 5;
    public double KF = 15;

    public Turret(DcMotorEx launch1, DcMotorEx launch2, Servo servo1, Servo servo2){
        launchMotor1 = launch1;
        launchMotor2 = launch2;
        turretServo = servo1;
        turretServo2 = servo2;
    }
    public void updateTurret(double poseX, double poseY, double headingRad) {
        double dt = turretTimer.seconds();
        turretTimer.reset();
        if (dt <= 0 || dt > 0.25) dt = 0.02; //first loop or a lag spike

        //if auto aim is off or we cant aim, just hold where we are
//        turretTargetDeg = turretCommandedDeg;
//        if (turretAutoAim) {
//            double rough = getRoughAim(poseX, poseY, headingRad);
//            if (!Double.isNaN(rough)) turretTargetDeg = rough;
//        }

        //move toward the target but not faster than the max speed
        double maxStep = TURRET_MAX_SPEED_DEG_PER_SEC * dt;
        double diff = turretTargetDeg - turretCommandedDeg;
        if (Math.abs(diff) <= maxStep) {
            turretCommandedDeg = turretTargetDeg;
        } else {
            turretCommandedDeg += Math.signum(diff) * maxStep;
        }
        servoPosition(Math.max(0, Math.min(1, angleToServoPos(turretCommandedDeg))));
    }
    public double angleToServoPos(double angle) {
        if (angle > 225) {
            angle -= 360;
        }
        double progress = (angle + 135)/(315);
        return Math.max(Math.min(TURRET_RIGHT_END,progress*(TURRET_RIGHT_END-TURRET_LEFT_END) + TURRET_LEFT_END), TURRET_LEFT_END);
    }
    public double servoPosToAngle(double servoPos){
        double rawAngle = -135 + 315 * (servoPos -TURRET_LEFT_END)/(TURRET_RIGHT_END-TURRET_LEFT_END);
        if (rawAngle < 0 ) {
            rawAngle += 360;
        }
        return rawAngle;
    }
    public void turnAbsolute(double robotHeading, double targetHeading) { //takes [0,360], [0,360]
        double turretRelative = targetHeading - robotHeading;
        if (turretRelative < 0) {
            turretRelative += 360;
        }
        turnRelative(turretRelative);
    }
    public void turnRelative(double targetHeading) {
        //starts the turret and center degree
        //turretCommandedDeg = servoPosToAngle(turretServo.getPosition());
        servoPosition(angleToServoPos(targetHeading));
//        turretTargetDeg = targetHeading;
//        turretTimer.reset(); //for time
    }
    public void servoPosition(double position) {
        turretServo.setPosition(position);
        turretServo2.setPosition(position);
    }
    private double lastKP = Double.NaN, lastKI = Double.NaN, lastKD = Double.NaN, lastKF = Double.NaN;
    public void updateLauncherPidf() {
        if (KP != lastKP || KI != lastKI || KD != lastKD || KF != lastKF) {
            PIDFCoefficients pidf = new PIDFCoefficients(KP, KI, KD, KF);
            launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            launchMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastKP = KP; lastKI = KI; lastKD = KD; lastKF = KF;
        }
    }
    public double avgSpeed() {
        return (launchMotor2.getVelocity() + launchMotor1.getVelocity())/2.0;
    }
    public boolean isFast () {
        if (VELOCITY_1 <= 0 || VELOCITY_2 <= 0) return false;
        return launchMotor1.getVelocity() > 0.95 * VELOCITY_1
                && launchMotor2.getVelocity() > 0.95 * VELOCITY_2;
    }
    public void faceForward() {
        servoPosition(SERVO_POSITION_FORWARD);
    }
    public void faceBackward() {
        servoPosition(SERVO_POSITION_BACKWARD);
    }
    public void setVelocity(double velocity1, double velocity2) {
        launchMotor1.setVelocity(velocity1);
        launchMotor2.setVelocity(velocity2);
    }

}
