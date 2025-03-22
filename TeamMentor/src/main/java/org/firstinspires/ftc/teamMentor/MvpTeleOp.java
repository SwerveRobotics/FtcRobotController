package org.firstinspires.ftc.teamMentor;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

import java.util.LinkedList;

@TeleOp(name = "TeleOp", group = "MentorBot")
public class MvpTeleOp extends LinearOpMode {
    final double EXTENSION_INCHES_PER_SECOND = 10; // Claw movement speed for direct user control
    final double MAX_HEIGHT = 15; // Max claw inches above the floor
    final double DEFAULT_HEIGHT = 12; // Default claw height, in inches

    double wristAngle = 0; // Radians
    double height = DEFAULT_HEIGHT; // Height of claw about floor during pickup, in inches
    boolean clawOpen = false; // Claw open/close state, starts closed

    // Shape the stick input for more precision at slow speeds:
    static double shapeStick(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        LinkedList<Action> actions = new LinkedList<>(); // List of active actions

        Poser poser = Poser.getPoser(hardwareMap, telemetry, gamepad1, null);
        Arm arm = new Arm(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, poser, telemetry, gamepad1);
        Ui ui = new Ui(telemetry, gamepad1, gamepad2);

        ui.out("Ready to start!");
        ui.update();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        double previousTime = nanoTime() / 1e9; // Time, in seconds, of previous loop iteration
        while (opModeIsActive()) {
            double time = nanoTime() / 1e9;
            double dt = time - previousTime;
            previousTime = time;

            Stats.beginLoop(Stats.Mode.REGULAR);
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Driver 2
            ui.setGamepad(1); // @@@@@@@@@@@@@@@@@@@@@@@@@@
            height -= ui.gamepad().right_stick_y * EXTENSION_INCHES_PER_SECOND * dt; // Invert the y axis
            height = Math.max(0, Math.min(height, MAX_HEIGHT)); // Don't let the claw dig into the floor

            if (ui.a()) {
                clawOpen = !clawOpen; // Toggle claw open/close
                actions.add(new ClawAction(arm, clawOpen));
            }
            if (ui.dpadRight() || ui.dpadLeft()) {
                actions.add(new ReachAction(arm, ReachAction.State.USER_PICKUP, () -> height));
            }
            if (ui.dpadUp()) {
                actions.add(new ReachAction(arm, ReachAction.State.LOW_BASKET));
                height = DEFAULT_HEIGHT;
            }
            if (ui.dpadDown()) {
                actions.add(new ReachAction(arm, ReachAction.State.START));
                height = DEFAULT_HEIGHT;
            }
            if (ui.rightBumper()) {
                wristAngle = arm.offsetWristAngle(wristAngle, -Math.PI / 4);
                actions.add(new WristAction(arm, wristAngle));
            }
            if (ui.leftBumper()) {
                wristAngle = arm.offsetWristAngle(wristAngle, Math.PI / 4);
                actions.add(new WristAction(arm, wristAngle));
            }

            // Driver 1.
            ui.setGamepad(1);
            // By default, the robot runs slow at 60% of max speed, for precision driving.
            // The left trigger proportionally decreases the speed down to 30% of max speed,
            // the right trigger up to 100%.
            double speedMultiplier = 0.6 + (ui.gamepad().right_trigger * 0.4) - (ui.gamepad().left_trigger * 0.3);
            PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                    shapeStick(-ui.gamepad().left_stick_y * speedMultiplier),
                    shapeStick(-ui.gamepad().left_stick_x * speedMultiplier)),
                    shapeStick(-ui.gamepad().right_stick_x * speedMultiplier));

            drive.setDrivePowers(velocity);
            arm.update(poser.getPose(), canvas);
            poser.update();

            // Run all of our active Road Runner actions, removing any that return false:
            actions.removeIf(action -> !action.run(packet));

            poser.draw(canvas);

            // Spew the stats:
            ui.out(Stats.getString());

            // We're done!
            MecanumDrive.sendTelemetryPacket(packet);
            ui.update();
        }
    }
}
