package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/** IMPORTS FOR AUTO-AIM**/
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import java.util.Comparator;


/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
public class CompetitionTeleopNew extends BaseOpMode {
    // TODO: update deadzone  and use in intake if statement.
    public static double JOYSTICK_DEADZONE = 0.1;
    @Override
    public void runOpMode() {
        initializeHardware();
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);
        // AUTO-AIM Variables

        // GET THE LIMELIGHT HARDWARE
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // CREATE AUTO-AIM OBJECT (null until button pressed)
        VisionAutoAim amazingAutoAim = null;

        // SET ALLIANCE COLOR (change to RED if red alliance)
        CompetitionAuto.Alliance alliance = CompetitionAuto.Alliance.BLUE;

        // VARIABLE TO STORE HOW MUCH TO TURN
        double amountToTurn = 0;

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");
            telemetry.update();
            // AUTO-AIM CONTROLS
            // if right bumper was pressed create auto aim object
            if (gamepad1.rightBumperWasPressed()) {
                amazingAutoAim = new VisionAutoAim(telemetry, limelight, alliance);
            }

            // IF RIGHT BUMPER IS HELD AND AUTO-AIM EXISTS, USE VISION
            if (gamepad1.right_bumper && amazingAutoAim != null) {
                // GET TURN POWER FROM LIMELIGHT
                amountToTurn = amazingAutoAim.get();
                telemetry.addData("AutoAim", "ON");
            } else {
                // ELSE USE MANUAL JOYSTICK CONTROL
                amountToTurn = -gamepad1.right_stick_x;
                telemetry.addData("AutoAim", "OFF");
            }



            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    amountToTurn
            ));


            // Update the current pose:
            drive.updatePoseEstimate();
            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);

            // intake controls
            if (gamepad2.left_stick_y == 0) {
                intakeMot.setVelocity(REVERSE_INTAKE_SPEED);
            } else {
                intakeMot.setVelocity(gamepad2.left_stick_y * INTAKE_SPEED_MULTIPLIER);
            }
            //launch controls
            if (gamepad2.dpadUpWasPressed()) {
                upperFlywheelMot.setVelocity(FLYWHEEL_FAR_SPEED);
                lowerFlywheelMot.setVelocity(FLYWHEEL_FAR_SPEED);
            } else if (gamepad2.dpadDownWasPressed()) {
                upperFlywheelMot.setVelocity(FLYWHEEL_NEAR_SPEED);
                lowerFlywheelMot.setVelocity(FLYWHEEL_NEAR_SPEED);
            } else if (gamepad2.dpadRightWasPressed()) {
                // turns off the flywheels
                upperFlywheelMot.setVelocity(WHEEL_STOP_SPEED);
                lowerFlywheelMot.setVelocity(WHEEL_STOP_SPEED);
            }
            // Fire shot
            if (gamepad2.yWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_WHEEL_START_SPEED);
            } else if (gamepad2.bWasPressed()) {
                transferWheelMot.setVelocity(WHEEL_STOP_SPEED);
            }
        }
    }
}

class VisionAutoAim {
    Telemetry telemetry = null;

    // PID TURNING CONSTANTS
    public static double KP = 1.5;  // makes it faster the further off center it is
    public static double KI = 0;    // helps correct errors that stay over time
    public static double KD = 0.1;  // smooths out the turning motion to prevent overshooting

    Limelight3A limelight;
    CompetitionAuto.Alliance alliance;
    PIDControllerNEW pid;

    // Set up the auto-aim system with the limelight camera and alliance color
    VisionAutoAim(Telemetry telemetry, Limelight3A limelight, CompetitionAuto.Alliance alliance) {
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.alliance = alliance;
        pid = new PIDControllerNEW(KP, KI, KD);
    }
    // MAIN AUTO-AIM METHOD
    // This method looks at the camera and returns how much the robot should turn

    public double get() {
        // get updated limelight data
        LLResult result = limelight.getLatestResult();

        //if no data dont turn
        if (result == null || !result.isValid()) {
            return 0;
        }

        // Get all detected April Tags
        List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();

        // REMOVE TAGS THAT AREN'T THE GOAL (IDs 21, 22, 23 are the obelisk goal tags)
        detections.removeIf(d -> d.getFiducialId() != 21 && d.getFiducialId() != 22 && d.getFiducialId() != 23);

        if (detections.isEmpty()) {
            return 0;
        }

        // Pick alliance based on tag detected
        LLResultTypes.FiducialResult target = null;
        if (alliance == CompetitionAuto.Alliance.RED) {
            // RED
            target = detections.stream()
                    .min(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetXDegrees))
                    .orElse(null);
        } else {
            // BLUE
            target = detections.stream()
                    .max(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetXDegrees))
                    .orElse(null);
        }

        if (target == null) return 0;

        // horizontal offset
        // (negative = tag is to the left, positive = tag is to the right)
        double tx = target.getTargetXDegrees();

        // (goal is to get tx to 0, which means centered)
        double pidOutput = pid.calculate(tx, 0);

        // SEND DEBUG INFO TO TELEMETRY
        telemetry.addData("tx", tx);
        telemetry.addData("pidOutput", pidOutput);

        // Make sure the output stays between -1 and 1, then send it back
        return Math.max(-1, Math.min(1, pidOutput));
    }
}

class PIDControllerNEW {
    private final double kP;
    private final double kI;
    private final double kD;
    private double setpoint;
    private double previousError = 0;
    private double integral = 0;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;
    private long lastTimestamp = System.nanoTime();


    public PIDControllerNEW(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    //Calculate how much to turn (simplified version that assumes we want to hit 0)
    public double calculate(double currentValue) {
        return calculate(currentValue, 0);
    }

    // Calculate how much to turn given a target and current value
    public double calculate(double currentValue, double setpoint) {
        // time passed since last calculation
        long now = System.nanoTime();
        double dt = (now - lastTimestamp) / 1e9;
        lastTimestamp = now;

        // Calculate error (how different from target)
        double error = setpoint - currentValue;

        // Keep track of how long the error has been present
        // (helps eliminate small persistent errors)
        integral += error * dt;

        // jow quickly error is changing
        double derivative = (error - previousError) / dt;

        // Combine the three components to calculate the turn power
        // P makes it respond quickly, I makes it more accurate, D smooths it out
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // keep it between limits
        output = Math.max(outputMin, Math.min(outputMax, output));

        //keep track of error
        previousError = error;

        return output;
    }
}
