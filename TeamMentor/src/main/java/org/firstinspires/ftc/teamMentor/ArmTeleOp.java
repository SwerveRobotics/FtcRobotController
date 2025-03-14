package org.firstinspires.ftc.teamMentor;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

import java.util.LinkedList;

/**
 * Constants describing the field.
 */
class FieldSpecs {
    static final double ABUTMENT_X = -15.5; // X coordinate of the submersible abutment, measure using Calibrator
    static final double SUBMERSIBLE_WIDTH = 27.5; // Inches
    static final double SUBMERSIBLE_LENGTH = 44.5; // Inches
    static final double HALF_SUBMERSIBLE_WIDTH = SUBMERSIBLE_WIDTH / 2;
    static final double HALF_SUBMERSIBLE_LENGTH = SUBMERSIBLE_LENGTH / 2;
    static final double SUBMERSIBLE_X = -15; // Robot has to stay to the left of this, in inches
    static final double SUBMERSIBLE_Y = 21; // Robot has to stay below this y value, in inches

    static final Segment[] COLLISION_WALLS = {
            new Segment(18, 24, 24, 24),
            new Segment(18, -24, 24, -24),
            new Segment(-18, 24, -24, 24),
            new Segment(-18, -24, -24, -24),
    };
}

/**
 * State persisted from Auto to TeleOp.
 */
class Orientation {
    static final int RED = 0;
    static final int BLUE = 1;
    static final int AUDIENCE = 2;
    //----------------------------
    static final int COUNT = 3;

    static final String[] Descriptors = { "Red", "Blue", "Audience" };

    static int state = RED; // Default to red unless changed by the driver
    static double getRotation() {
        return (state != AUDIENCE) ? Math.PI / 2 : 0; // Audience view is rotated 90 degrees
    }
}

/**
 * State enumerations.
 */
enum AutoPilotState {
    NONE,
    BASKET,
    PICKUP
}

@TeleOp(name = "TeleOp", group = "MentorBot")
public class ArmTeleOp extends LinearOpMode {
    final double EXTENSION_INCHES_PER_SECOND = 10; // Claw movement speed for direct user control
    final double FOCUS_INCHES_PER_SECOND = 20; // Focus movement speed for direct user control

    LinkedList<Action> actions = new LinkedList<>(); // List of active actions
    double distance = 13; // Current claw distance
    double height = 0; // Current claw height
    Point pickupPoint = new Point(0, 0); // Current pickup point within submersible

    // Shape the stick input for more precision at slow speeds:
    static double shapeStick(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Poser poser = Poser.getPoser(hardwareMap, telemetry, gamepad1, null);
        Arm arm = new Arm(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, poser, telemetry, gamepad1);

        telemetry.addLine("Ready to start!");
        telemetry.update();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        double previousTime = nanoTime() / 1e9; // Time, in seconds, of previous loop iteration
        double startTime = nanoTime() / 1e9; // Start time of the TeleOp, in seconds
        boolean markedEnd = false; // True if the end of the TeleOp has been marked
        AutoPilotState autoPilotState = AutoPilotState.NONE;
        AutoPilotAction autoPilotAction = null;
        boolean openClaw = true;
        double wristAngle = 0;
        boolean fieldRelative = true; // True if the robot is field-relative, false if it's robot-relative

        boolean aWasPressed = false;
        boolean bWasPressed = false;
        boolean xWasPressed = false;
        boolean yWasPressed = false;
        boolean rightStickWasPressed = false;
        boolean leftShoulderWasPressed = false;
        boolean rightShoulderWasPressed = false;
        boolean guideWasPressed = false;
        boolean startWasPressed = false;
        LinkedList<Double> guidePressTimes = new LinkedList<>();
        LinkedList<Double> startPressTimes = new LinkedList<>();
        while (opModeIsActive()) {
            double time = nanoTime() / 1e9;
            double dt = time - previousTime;
            previousTime = time;

            Stats.beginLoop(Stats.Mode.REGULAR);
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Update the current pose:
            drive.updatePoseEstimate(false);
            StringBuilder builder = new StringBuilder();
            builder.append(String.format("Time: <big><big><big>%.1f</big></big></big>s, ", time - startTime));
            builder.append(String.format("orientation: <big><big><big>%s</big></big></big>, ",
                    Orientation.Descriptors[Orientation.state]));
            builder.append(String.format("field-relative: <big><big><big>%s</big></big></big>, ",
                    fieldRelative ? "On" : "Off"));
            builder.append(String.format("pose confidence: <big><big><big>%s</big></big></big>",
                    poser.getConfidence()));
            telemetry.addLine(builder.toString());

            // Rumble at the end of the match:
            if ((time - startTime > 120) && (!markedEnd)) {
                gamepad1.rumble(500);
                markedEnd = true;
            }

            // Process the gamepad/home button:
            if ((gamepad1.guide) && (!guideWasPressed)) {
                guidePressTimes.add(time);
            }
            guideWasPressed = gamepad1.guide;

            // While the guide button is held, show the error of the pose estimate:
            if (gamepad1.guide) {
                // The robot's x value when it's abutted to the submersible, facing in:
                final double xSubmersible = -16 - WilyConfig.ROBOT_LENGTH/2;
                Pose2d pose = poser.getPose();
                if (poser.getConfidence() == Poser.Confidence.HIGH) {
                    telemetry.addLine("Pose error, assuming robot is abutted to the submersible:");
                    telemetry.addLine(String.format("&emsp;Heading error: %.2f\u00b0", Math.toDegrees(pose.heading.log())));
                    telemetry.addLine(String.format("&emsp;X error: %.2f\"", pose.position.x - xSubmersible));
                }

                // Remove all guide press times from more than 2 seconds ago:
                while ((!guidePressTimes.isEmpty()) && (time - guidePressTimes.getFirst() > 2)) {
                    guidePressTimes.removeFirst();
                }

                // If we have 3 guide presses within 2 seconds (i.e., a triple tap), reset the pose:
                if (guidePressTimes.size() >= 3) {
                    gamepad1.rumble(100);
                    poser.setPose(new Pose2d(xSubmersible,0, 0), Poser.Confidence.LOW);
                    guidePressTimes.clear();
                    startTime = time;
                }
            }

            // Start button controls field-relative driving and the
            if ((gamepad1.start) && (!startWasPressed)) {
                startPressTimes.add(time);
                fieldRelative = !fieldRelative; // Toggle field-relative driving
            }
            startWasPressed = gamepad1.start;
            if (gamepad1.start) {
                // Remove all start press times from more than 2 seconds ago:
                while ((!startPressTimes.isEmpty()) && (time - startPressTimes.getFirst() > 2)) {
                    startPressTimes.removeFirst();
                }
                // If we have 3 start presses within 2 seconds (i.e., a triple tap), change the
                // field orientation:
                if (startPressTimes.size() >= 3) {
                    Orientation.state = (Orientation.state + 1) % Orientation.COUNT;
                    startPressTimes.clear();
                    fieldRelative = true;
                }
            }

            // Gamepad1 dpad moves the claw:
            double dx = (gamepad1.dpad_right) ? 1 : ((gamepad1.dpad_left) ? -1 : 0);
            double dy = (gamepad1.dpad_up) ? 1 : ((gamepad1.dpad_down) ? -1 : 0);
            double newDistance = distance + dx * EXTENSION_INCHES_PER_SECOND * dt;
            double newHeight = height + dy * EXTENSION_INCHES_PER_SECOND * dt;
            newHeight = Math.max(0, newHeight); // Don't let the claw dig into the floor
            newDistance = Math.max(0, newDistance);
            if (Arm.computeReach(newDistance, newHeight) != null) {
                distance = newDistance;
                height = newHeight;
            }

            // Gamepad1 touchpad provides absolute positioning for the pickup focus point:
            if (gamepad1.touchpad_finger_1) {
                telemetry.addLine(String.format("Touchpad: %.2f, %.2f",
                        gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y));
                pickupPoint = new Point(
                    gamepad1.touchpad_finger_1_x * FieldSpecs.HALF_SUBMERSIBLE_WIDTH,
                    gamepad1.touchpad_finger_1_y * FieldSpecs.HALF_SUBMERSIBLE_LENGTH);
            }

            // The gamepad2 left stick provides relative positioning:
            pickupPoint = new Point(pickupPoint.x, pickupPoint.y);
            pickupPoint.x += gamepad2.left_stick_x * FOCUS_INCHES_PER_SECOND * dt;
            pickupPoint.y += -gamepad2.left_stick_y * FOCUS_INCHES_PER_SECOND * dt;
            pickupPoint.x = Math.max(-FieldSpecs.HALF_SUBMERSIBLE_WIDTH, Math.min(FieldSpecs.HALF_SUBMERSIBLE_WIDTH, pickupPoint.x));
            pickupPoint.y = Math.max(-FieldSpecs.HALF_SUBMERSIBLE_LENGTH, Math.min(FieldSpecs.HALF_SUBMERSIBLE_LENGTH, pickupPoint.y));

            // Draw the pickup point cross-hairs:
            final double SIZE = 2;
            canvas.setStroke("#808080");
            canvas.strokeLine(pickupPoint.x - SIZE, pickupPoint.y, pickupPoint.x + SIZE, pickupPoint.y);
            canvas.strokeLine(pickupPoint.x, pickupPoint.y - SIZE, pickupPoint.x, pickupPoint.y + SIZE);

            // Left trigger on either gamepad pulls the robot to the basket, right trigger to the
            // submersible:
            double triggerSum = gamepad1.right_trigger - gamepad1.left_trigger;
            if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0)) {
                triggerSum = gamepad2.right_trigger - gamepad2.left_trigger;
            }

            // Invoke the autopilot only when confident in the pose estimate:
            if ((triggerSum != 0) && (poser.getConfidence() == Poser.Confidence.HIGH)) {
                if ((triggerSum > 0) && (autoPilotState != AutoPilotState.PICKUP)) {
                    autoPilotState = AutoPilotState.PICKUP;
                    Autopilot autopilot = new Autopilot(drive, telemetry, poser, ()->gamepad1.rumble(200));
                    autoPilotAction = new SubmersibleAutoPilotAction(telemetry, arm, autopilot, () -> pickupPoint);
                } else if ((triggerSum < 0) && (autoPilotState != AutoPilotState.BASKET)) {
                    autoPilotState = AutoPilotState.BASKET;
                    Autopilot autopilot = new Autopilot(drive, telemetry, poser, ()->gamepad1.rumble(200));
                    autoPilotAction = new BasketAutoPilotAction(arm, autopilot);
                }
            } else {
                autoPilotAction = null;
                autoPilotState = AutoPilotState.NONE;

                // Adjust the reach if it's changed:
                if ((dx != 0) || (dy != 0)) {
                    actions.add(new ReachAction(arm, ReachAction.State.PICKUP, distance, height));
                }

                // A button moves arm to the start position:
                if ((gamepad1.a) && (!aWasPressed))
                    actions.add(new ReachAction(arm, ReachAction.State.START));
                aWasPressed = gamepad1.a;

                // B button moves arm to the pickup position:
                if ((gamepad1.b) && (!bWasPressed))
                    actions.add(new ReachAction(arm, ReachAction.State.PICKUP, distance, height));
                bWasPressed = gamepad1.b;

                // X button moves arm to the home position:
                if ((gamepad1.x) && (!xWasPressed))
                    actions.add(new ReachAction(arm, ReachAction.State.HOME));
                xWasPressed = gamepad1.x;

                // Y button moves arm to the basket position:
                if ((gamepad1.y) && (!yWasPressed))
                    actions.add(new ReachAction(arm, ReachAction.State.HIGH_BASKET));
                yWasPressed = gamepad1.y;

                // Right stick button toggles the claw open/closed:
                if ((gamepad1.right_stick_button) && (!rightStickWasPressed)) {
                    openClaw = !openClaw;
                    actions.add(new ClawAction(arm, openClaw));
                }
                rightStickWasPressed = gamepad1.right_stick_button;

                // Left and right bumpers adjust the wrist angle:
                if ((gamepad1.left_bumper) && (!leftShoulderWasPressed)) {
                    wristAngle += Math.PI / 4;
                    if (wristAngle > Math.PI)
                        wristAngle -= 2 * Math.PI;
                    actions.add(new WristAction(arm, wristAngle));
                }
                leftShoulderWasPressed = gamepad1.left_bumper;
                if ((gamepad1.right_bumper) && (!rightShoulderWasPressed)) {
                    wristAngle -= Math.PI / 4;
                    if (wristAngle < -Math.PI)
                        wristAngle += 2 * Math.PI;
                    actions.add(new WristAction(arm, wristAngle));
                }
                rightShoulderWasPressed = gamepad1.right_bumper;

                // Gamepad1 triggers control the turret:
                actions.add(new TurretAction(arm, gamepad2.right_trigger - gamepad2.left_trigger)); // NOTE: Driver 2
            }

            arm.update(drive.pose, canvas);
            poser.draw(canvas);

            telemetry.addLine(String.format("Pickup point: %.2f, %.2f", pickupPoint.x, pickupPoint.y));
            telemetry.addLine(String.format("Pickup error: %.2f, %.2f",
                    Math.abs(pickupPoint.x - arm.model.computedPickupPoint.x),
                    Math.abs(pickupPoint.y - arm.model.computedPickupPoint.y)));

            // Run all of our active Road Runner actions, removing any that return false:
            actions.removeIf(action -> !action.run(packet));

            // Set the drive motor powers according to the gamepad input:
            if (autoPilotAction == null) {
                PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                        shapeStick(-gamepad1.left_stick_y),
                        shapeStick(-gamepad1.left_stick_x)),
                        shapeStick(-gamepad1.right_stick_x));

                if ((fieldRelative) && (poser.getConfidence() != Poser.Confidence.NONE)) {
                    // Flip the field orientation for the audience view. Blue and red use
                    // the same rotation because we cheat and use the same field orientation
                    // relative to those sides. The only difference between red and blue
                    // is the color of the field elements, with corresponding impacts on
                    // computer vision.
                    double rotation = -poser.getPose().heading.log() + Math.PI + Orientation.getRotation();
                    Point adjustedInput = new Point(-velocity.linearVel.x, -velocity.linearVel.y);
                    velocity = new PoseVelocity2d(adjustedInput.rotate(rotation).vector2d(), velocity.angVel);
                }
                drive.setDrivePowers(velocity);
                poser.setTarget(null); // Inform poser that there's no target
            } else {
                Vector2d velocity = new Vector2d(
                        shapeStick(gamepad1.left_stick_x),
                        shapeStick(-gamepad1.left_stick_y));
                autoPilotAction.setUserVelocity(velocity);
                if (!autoPilotAction.run(packet)) {
                    autoPilotAction = null; // The action is done, don't run it again
                }
            }

            // Spew the stats:
            telemetry.addLine(Stats.getString());

            // We're done!
            MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
