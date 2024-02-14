package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagPoseEstimator;
import org.firstinspires.ftc.team417_CENTERSTAGE.mechanisms.ArmMechanism;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
public abstract class BaseTeleOp extends BaseOpMode {
    // Set to false for competitions to remove lags
    public static final boolean TESTING = true;

    private ArmMechanism arm;

    ElapsedTime time = new ElapsedTime();

    public static boolean USE_APRIL_TAGS = false;

    public AprilTagPoseEstimator myATPoseEstimator;

    public static double droneServoReleasedPos = 1;
    public static double droneServoHoldingPos = 0.473;

    public void runTeleOp(boolean driveTo, boolean aprilTags, Pose2d pose, int armPosition) {
        initializeHardware();

        USE_APRIL_TAGS = aprilTags;

        drive.pose = new Pose2d(pose.position.x, pose.position.y, pose.heading.log());

        // Initialize April Tags (if enabled)
        if (USE_APRIL_TAGS) {
            myATPoseEstimator = new AprilTagPoseEstimator(hardwareMap, telemetry);

            // Pass an April Tag Helper object so drive can add twists to it (See MecanumDrive for
            //     updatePoseEstimate() an explanation on twists)
            drive.setATLHelper(myATPoseEstimator.myAprilTagLatencyHelper);
        }

        if (armMotor != null) {
            arm = new ArmMechanism(gamepad2, armMotor, dumperServo);
            arm.currentArmLocation = armPosition;
            resetDumper();
            droneServo.setPosition(droneServoHoldingPos);
        }

        waitForStart();

        if (dumperWheelServo != null)
            dumperWheelServo.setPower(-1.0);


        while (opModeIsActive()) {
            double startOfLoop = time.nanoseconds();

            telemetry.addLine(org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.summary);

            driveUsingControllers(false);

            drive.updatePoseEstimate();

            telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)",
                    drive.pose.position.x, drive.pose.position.y,
                   Math.toDegrees(drive.pose.heading.log())));

            // Code added to draw the pose, use only when testing
            if (TESTING) {
                TelemetryPacket p = new TelemetryPacket();
                Canvas c = p.fieldOverlay();
                c.setStroke("#3F5100");
                MecanumDrive.drawRobot(c, drive.pose);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.sendTelemetryPacket(p);
            }

            if (armMotor != null && dumperServo != null && gateServo != null) {
                outputUsingControllers();
                intakeUsingControllers();
                telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                telemetry.addData("Dumper Position", dumperServo.getPosition());
                telemetry.addData("Gate Position", gateServo.getPosition());
            }

            double elapsedTimeInLoop = time.nanoseconds() - startOfLoop;

            telemetry.addData("Loop Time", Integer.toString((int) (elapsedTimeInLoop * 1e-6)));

            if (USE_APRIL_TAGS) {
                myATPoseEstimator.updatePoseEstimate();
            }

            telemetry.update();
        }
    }

    boolean leftBumperIsPressed = false;

    public boolean sensitive = false;

    public void driveUsingControllers() {
        driveUsingControllers(false);
    }

    public void driveUsingControllers(boolean curve) {
        sensitive = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.425;
            rotSensitivity = 0.68;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x, y, rot;
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

    public static boolean droneLaunched = false;

    public static boolean xIsPressed = false;
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
        double speed = -gamepad1.right_trigger;
        dumperWheelServo.setPower(speed);
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
