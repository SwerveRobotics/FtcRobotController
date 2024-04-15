package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms.ArmMechanism;
import org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms.AutoDriveTo;
import org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal.Bezier;
import org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal.DPoint;
import org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal.PathFollowing;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
public abstract class BaseTeleOp extends BaseOpMode {
    // Set to false for competitions to remove lags
    public static final boolean TESTING = true;
    public static final boolean USING_AUTO_DRIVE_TO = false;

    private ArmMechanism arm;

    ElapsedTime time = new ElapsedTime();
    AutoDriveTo autoDrive;
    PathFollowing curveDrive;

    private final double LINE_APROX_EPSILON = 0.01;
    @Override
    public void runOpMode() {
        initializeHardware();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#3F5100");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        MecanumDrive.drawRobot(canvas, drive.pose);
        dashboard.sendTelemetryPacket(packet);

        /*if (armMotor != null) {
            arm = new ArmMechanism(gamepad2, armMotor, dumperServo, distSensor);
            resetDumper();
            droneServo.setPosition(droneServoHoldingPos);
        }*/

        curveDrive = new PathFollowing(drive, canvas, packet, telemetry);

        waitForStart();

        /*if (dumperWheelServo != null)
            dumperWheelServo.setPower(-1.0);*/

        drive.updatePoseEstimate();

        Bezier controlPoints= new Bezier(new DPoint(0, 0),
                new DPoint(0, 0), new DPoint(24, 24), new DPoint(24, 24), LINE_APROX_EPSILON);
        curveDrive.cubicDriveTo(controlPoints, true);

        while (opModeIsActive()) {
            curveDrive.cubicDriveTo(controlPoints, false);

            //resetIMUIfNeeded();

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            MecanumDrive.drawRobot(canvas, drive.pose);
            dashboard.sendTelemetryPacket(packet);

            /*if (armMotor != null && dumperServo != null && gateServo != null) {
                outputUsingControllers();
                intakeUsingControllers();
                telemetry.addData("ArmMotor", armMotor.getCurrentPosition());
                telemetry.addData("DumperServo", dumperServo.getPosition());
                telemetry.addData("GateServo", gateServo.getPosition());
            }
            */
            telemetry.update();
        }

        // Close drive (release resources)
        drive.close();
    }

    boolean leftBumperIsPressed = false;

    public void resetIMUIfNeeded() {
        if (gamepad1.left_bumper && !leftBumperIsPressed) {
            IMU.Parameters parameters;
            if (drive.isDevBot) {
                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            } else {
                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)); }
            drive.imu.initialize(parameters);
        }
        leftBumperIsPressed = gamepad1.left_bumper;
    }

    public boolean sensitive = false;

    public void driveUsingControllers() {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
        double y = curveStick(-gamepad1.left_stick_y) * sensitivity;
        double rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;

        mecanumDrive(x, y, rot);
    }
    boolean usingStick = true;
    boolean dPadUpPressed;

    public void driveUsingControllers(boolean curve) {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;
        double x, y, rot;
        double deadZone = 0.02;
        boolean driveToInit = false;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        if (gamepad1.dpad_up) {
            usingStick = false;
            if (!dPadUpPressed)
                driveToInit = true;
        } else if (Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.left_stick_x) > deadZone ||
                   Math.abs(gamepad1.right_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone)
            usingStick = true;


        telemetry.addData("hasDriveToInit", driveToInit);

        if (usingStick) {
            if (curve) {
                x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
                y = curveStick(-gamepad1.left_stick_y) * sensitivity;
                rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
            } else {
                x = gamepad1.left_stick_x * strafeConstant * sensitivity;
                y = -gamepad1.left_stick_y * sensitivity;
                rot = gamepad1.right_stick_x * rotSensitivity;
            }
            mecanumDrive(x, y, rot);
        } else if (USING_AUTO_DRIVE_TO)
            autoDrive.linearDriveTo(48, -36, Math.toRadians(180), driveToInit);

        dPadUpPressed = gamepad1.dpad_up;
    }

    public void intakeUsingControllers() {
        double speed = gamepad1.left_trigger - gamepad1.right_trigger;
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Intake Speed", speed);
        runIntakeMechanism(speed);
    }

    public void outputUsingControllers() {
        controlDumperUsingControllers();
        controlDumperWheelUsingControllers();
        controlGateUsingControllers();
        if (arm != null)
            arm.armControl();
        controlDroneLauncher();
    }

    private boolean testDroneLauncher = true;
    private boolean droneLauncherInited = false;
    private double droneLauncherInitTime;
    public static double droneServoReleasedPos = 0.5;
    public static double droneServoHoldingPos = 0.08;

    private void controlDroneLauncher() {
        if (!droneLauncherInited) {
            droneLauncherInitTime = time.milliseconds();
            droneLauncherInited = true;
        }

        if (testDroneLauncher || time.milliseconds() - droneLauncherInitTime > 90000.0)
        {
            if (gamepad1.x)
                droneServo.setPosition(droneServoReleasedPos);
        }
    }

    public boolean dumperDumped = false;
    public boolean bIsPressed = false;
    public boolean yIsPressed = false;

    public void controlDumperUsingControllers() {
        if (!bIsPressed && gamepad2.b) {
            if (!dumperDumped) {
                dumpDumper();
            } else {
                resetDumper();
            }
            dumperDumped = !dumperDumped;
        }

        bIsPressed = gamepad2.b;

        if (!yIsPressed && gamepad2.y) {
            if (!dumperDumped) {
                tiltDumper();
            } else {
                resetDumper();
            }
            dumperDumped = !dumperDumped;
        }

        yIsPressed = gamepad2.y;
    }

    boolean downIsPressed = false;
    boolean wheelOn = true;
    public void controlDumperWheelUsingControllers() {
        if (!downIsPressed && gamepad1.dpad_down) {
            if (!wheelOn) {
                dumperWheelServo.setPower(-1);
                wheelOn = true;
            } else {
                dumperWheelServo.setPower(0);
                wheelOn = false;
            }
        }

        downIsPressed = gamepad1.dpad_down;
    }

    public boolean gateOpen = false;
    public boolean aIsPressed = false;

    public void controlGateUsingControllers() {
        if (!aIsPressed && gamepad2.a){
            if (gateOpen) {
                closeGate();
            } else {
                openGate();
            }
            gateOpen = !gateOpen;
        }
        aIsPressed = gamepad2.a;
    }

    //Adds stick curve to the joysticks
    public double curveStick(double rawSpeed) {
        double logSpeed;
        if (rawSpeed >= 0) {
            logSpeed = Math.pow(rawSpeed, 2);
        } else {
            logSpeed = -Math.pow(rawSpeed, 2);
        }
        return logSpeed;
    }
}
