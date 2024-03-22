package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


abstract public class BaseOpMode extends LinearOpMode {

    static public double GONDOLA_INITIALIZED_ANGLE = 0.6;
    static public double ARMS_INITIALIZED_ANGLE = 0.25;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    static final double TICKS_PER_REVOLUTION = 537.6;
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 4.0;
    static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;

    public IMU imu;
    public double startAngle;
    public static final double TURNING_KP = 0.008;
    public static final int ROBOT_HEADING_TOLERANCE_DEGREES = 1;
    public static final double MAXIMUM_TURN_POWER_AUTONOMOUS = 0.7;
    public static final double MINIMUM_TURN_POWER = 0.05;

    public double referenceAngle;
    CRServo servoSlideRight;

    CRServo servoSlideLeft;

    DcMotor motorIntakeWheels;

    Servo servoFlipGondola;

    CRServo servoReleasePixel;

    Servo servoRotateGondola;

    Servo servoReleaseDrone;

    public void initHardware() {
        // drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        servoSlideLeft = hardwareMap.crservo.get("servoSlideLeft");
        servoSlideRight = hardwareMap.crservo.get("servoSlideRight");

        motorIntakeWheels = hardwareMap.dcMotor.get("motorIntakeWheels");

        servoFlipGondola = hardwareMap.servo.get("servoFlipGondola");

        servoReleasePixel = hardwareMap.crservo.get("servoReleasePixel");

        servoReleaseDrone = hardwareMap.servo.get("servoReleaseDrone");

        servoRotateGondola = hardwareMap.servo.get("servoRotateGondola");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorIntakeWheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntakeWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeWheels.setDirection(DcMotor.Direction.FORWARD);

        // initializes the gondola and arm servos (same positions as when they're lowered during tele-op)
        servoFlipGondola.setPosition(ARMS_INITIALIZED_ANGLE);
        servoRotateGondola.setPosition(GONDOLA_INITIALIZED_ANGLE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        sleep(2000);
    }

    // mecanum drive method
    public void driveMecanum(double x, double y, double turning) {
        motorFL.setPower(x + y + turning);
        motorFR.setPower(y - x - turning);
        motorBL.setPower(y - x + turning);
        motorBR.setPower(x + y - turning);
    }

}
