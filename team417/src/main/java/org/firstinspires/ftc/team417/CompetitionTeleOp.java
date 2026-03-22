package org.firstinspires.ftc.team417;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class CompetitionTeleOp extends BaseOpMode {

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    private Twist2d lastError = new Twist2d(new Vector2d(0, 0), 0);

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Pose2d beginPose;
        if (TransferState.pose != null) {
            beginPose = TransferState.pose;
        } else {
            beginPose = new Pose2d(0, 0, 0);
        }

        CompetitionAuto.Alliance alliance;
        if (TransferState.chosenAlliance != null) {
            alliance = TransferState.chosenAlliance;
        } else {
            alliance = CompetitionAuto.Alliance.BLUE;
        }

        PixelColor[] storedColors;
        if (TransferState.storedColors != null) {
            storedColors = TransferState.storedColors;
        } else {
            storedColors = new PixelColor[] {PixelColor.NONE, PixelColor.NONE, PixelColor.NONE};
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);
        PixelColor[] preloads = new PixelColor[]{PixelColor.NONE, PixelColor.NONE, PixelColor.NONE};
        MechGlob mechGlob = ComplexMechGlob.create(hardwareMap, telemetry, storedColors);
        AmazingAutoAim amazingAutoAim = null;
        detector = new LimelightAprilTagDetector(hardwareMap, drive);

        detector.poseCorrectEnabled = TransferState.usePoseCorrection;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        //Variable for auto aim
        double amountToTurn;
        // Initialize motors, servos, LEDs

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");

            if (gamepad1.rightBumperWasPressed()) {
                amazingAutoAim = new AmazingAutoAim(telemetry, alliance);
            }

            if (gamepad1.right_bumper && TransferState.trustPose) {
                amountToTurn = -amazingAutoAim.get(drive.pose);
            } else {
                amountToTurn = halfLinearHalfCubic(-gamepad1.right_stick_x);
            }

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            halfLinearHalfCubic(-gamepad1.left_stick_y * doSLOWMODE()),
                            halfLinearHalfCubic(-gamepad1.left_stick_x * doSLOWMODE())

                    ),
                    amountToTurn

            ));

            // Update the current pose:
            drive.updatePoseEstimate();

            detector.updateRobotYaw(drive.pose.heading.log());

            telemetry.addData("Pose trusted", TransferState.trustPose ? "✅" : "❌");
            telemetry.addLine(String.format("Last pose correction %s (%.2f\", %.2f\")",
                    detector.lastWithinRange ? "✅" : "❌", detector.lastXDistance, detector.lastYDistance));
            telemetry.addLine(String.format("Last manual reset (%.2f\", %.2f\", %.2f°)",
                    lastError.line.x, lastError.line.y,
                    AngleUnit.normalizeDegrees(Math.toDegrees(lastError.angle))));

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);

            Pose2d cameraPose = detector.detectRobotPose();
            if (cameraPose != null) {
                packet.fieldOverlay().setStroke("#3FB578");
                Drawing.drawRobot(packet.fieldOverlay(), cameraPose);
            }

            MecanumDrive.sendTelemetryPacket(packet);

            //add slowbot teleop controls here
            if (gamepad2.yWasPressed()) {
                mechGlob.launch(RequestedColor.EITHER, detector);
            } else if (gamepad2.bWasPressed()) {
                mechGlob.launch(RequestedColor.PURPLE, detector);
            } else if (gamepad2.aWasPressed()) {
                mechGlob.launch(RequestedColor.GREEN, detector);
            }
            if (gamepad2.dpadUpWasPressed()) {
                mechGlob.setLaunchVelocity(LaunchDistance.FAR);
            } else if (gamepad2.dpadDownWasPressed()) {
                mechGlob.setLaunchVelocity(LaunchDistance.NEAR);
            } else if (gamepad2.dpadRightWasPressed()) {
                // turns off the flywheels
                mechGlob.setLaunchVelocity(LaunchDistance.OFF);
            //Manually stop the transfer process
            } else if (gamepad2.rightBumperWasPressed()) {
                mechGlob.stopLaunch();
            }

            mechGlob.ohCrap(gamepad2.right_trigger > 0);
            if (gamepad2.backWasPressed()) {
                telemetry.log().clear();
            }

            if (gamepad1.guideWasPressed()) {
                Pose2d lastPose = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log());

                // Reset pose to alliance's human player zone
                if (alliance == CompetitionAuto.Alliance.RED) {
                    drive.setPose(new Pose2d(72 - ROBOT_WIDTH/2, -72 + ROBOT_LENGTH/2, Math.toRadians(-90)));
                } else {
                    drive.setPose(new Pose2d(72 - ROBOT_WIDTH/2, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
                }

                TransferState.trustPose = true;

                // Remember what the error was
                lastError = drive.pose.minus(lastPose);
            }

            mechGlob.intake(gamepad2.left_stick_y);
            mechGlob.update();

            PixelColor slot0 = mechGlob.getSlotColor(0);
            PixelColor slot1 = mechGlob.getSlotColor(1);
            PixelColor slot2 = mechGlob.getSlotColor(2);

            telemetry.addData("Slot0: ", slot0);
            telemetry.addData("Slot1: ", slot1);
            telemetry.addData("Slot2: ", slot2);

            telemetry.update();
        }

        detector.close();
    }



    public double doSLOWMODE() {
        if (gamepad1.right_trigger != 0) {
            return -gamepad1.right_trigger + 1.1;
        } else {
            return 1;
        }
    }

    public static double halfLinearHalfCubic(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }
}

class AmazingAutoAim {
    Telemetry telemetry = null;
    // Constants to tune in FTC dashboard
    public static double KP = 1.5;
    public static double KI = 0;
    public static double KD = 0.1;
    double targetX;
    double targetY;
    PIDController pid;

    AmazingAutoAim(Telemetry telemetry, CompetitionAuto.Alliance alliance) {
        this.telemetry = telemetry;

        if (alliance == CompetitionAuto.Alliance.RED) {
            targetX = -65;
            targetY = 55;
        } else {
            targetX = -65;
            targetY = -55;
        }
        pid = new PIDController(KP, KI, KD);

    }

    public double get(Pose2d pose) {
        double deltaY = targetY - pose.position.y;
        double deltaX = targetX - pose.position.x;

        double beta = Math.atan2(deltaY, deltaX);
        double alpha = pose.heading.toDouble();
        double angle = beta - alpha;
        double normalizedAngle = AngleUnit.normalizeRadians(angle);

        double pidOutput = pid.calculate(normalizedAngle);

        if (pidOutput  <= -1) {
            return -1;
        } else if (pidOutput >= 1){
            return 1;
        } else {
            return pidOutput;
        }



    }

}


class PIDController {

    private double kP;
    private double kI;
    private double kD;

    private double setpoint;
    private double previousError = 0;
    private double integral = 0;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    private long lastTimestamp = System.nanoTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * Calculates the PID output based on the current process variable.
     */
    public double calculate(double currentValue) {
        long now = System.nanoTime();
        double dt = (now - lastTimestamp) / 1e9; // seconds
        lastTimestamp = now;

        double error = setpoint - currentValue;

        // Integral with basic anti-windup
        integral += error * dt;

        // Derivative
        double derivative = (error - previousError) / dt;

        // PID Output
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output
        output = Math.max(outputMin, Math.min(outputMax, output));

        previousError = error;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        lastTimestamp = System.nanoTime();
    }
}



