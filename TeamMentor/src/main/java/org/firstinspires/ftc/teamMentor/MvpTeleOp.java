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
    final double MAX_HEIGHT = 8; // Max claw inches above the floor

    double wristAngle = 0; // Radians
    double height = 4; // Height of claw about floor during pickup, in inches

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
            ui.setGamepad(2);

            height += gamepad2.left_stick_y * EXTENSION_INCHES_PER_SECOND * dt;
            height = Math.max(0, Math.min(height, MAX_HEIGHT)); // Don't let the claw dig into the floor

            if (ui.a())
                actions.add(new ReachAction(arm, ReachAction.State.USER_PICKUP, () -> height));
            if (ui.b())
                actions.add(new ReachAction(arm, ReachAction.State.LOW_BASKET));
            if (ui.x())
                actions.add(new ReachAction(arm, ReachAction.State.START));
            if (ui.rightBumper()) {
                wristAngle = Util.normalizeAngle(wristAngle - Math.PI / 4);
                actions.add(new WristAction(arm, wristAngle));
            }
            if (ui.leftBumper()) {
                wristAngle = Util.normalizeAngle(wristAngle + Math.PI / 4);
                actions.add(new WristAction(arm, wristAngle));
            }
            if (ui.rightTrigger()) {
                actions.add(new ClawAction(arm, false)); // Close claw
            }
            if (ui.leftTrigger()) {
                actions.add(new ClawAction(arm, true)); // Open claw
            }

            // Driver 1.
            ui.setGamepad(1);
            // The right trigger slows the robot for precision driving. When fully pressed,
            // it slows down to a factor of 0.3 (30% of full speed):
            double speedMultiplier = 1.0 - (gamepad1.right_trigger * 0.7);
            PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                    shapeStick(-gamepad1.left_stick_y * speedMultiplier),
                    shapeStick(-gamepad1.left_stick_x * speedMultiplier)),
                    shapeStick(-gamepad1.right_stick_x * speedMultiplier));

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
