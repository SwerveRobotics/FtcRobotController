package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

@Disabled
@Autonomous(name="Autonomous", group="Mentor", preselectTeleOp="TeleOp")
public class ArmAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-48 + Specs.Robot.LENGTH / 2,
                -72 + Specs.Robot.WIDTH / 2,
                Math.PI);
        Poser poser = Poser.getPoser(hardwareMap, telemetry, gamepad1, startPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, poser, telemetry, gamepad1);
        Arm arm = new Arm(hardwareMap, telemetry);
        Autopilot autopilot = new Autopilot(drive, telemetry, poser, ()->{});

        telemetry.addLine("Ready to start autonomous! Position the robot, please.");
        telemetry.update();

        // Let the robot be driven into position:
        while (!isStarted()) {
            PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                    ArmTeleOp.shapeStick(-gamepad1.left_stick_y),
                    ArmTeleOp.shapeStick(-gamepad1.left_stick_x)),
                    ArmTeleOp.shapeStick(-gamepad1.right_stick_x));
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            drive.updatePoseEstimate();
            drive.setDrivePowers(velocity);
            poser.draw(packet.fieldOverlay());
            MecanumDrive.sendTelemetryPacket(packet);
            if (gamepad1.guide)
                drive.setPose(startPose);
        }
        drive.setPose(startPose); // Don't forget to reset the pose

        Action autoAction = new SequentialAction(
                // @@@ Autopilot can only abut once per instantiation

                // Drop preload at the basket:
                new BasketAutoPilotAction(arm, autopilot),
                new ClawAction(arm, true),

                // Pickup the first sample and deposit it:
                new ParallelAction(
                        new DriveToAction(autopilot, new Pose2d(-48, -48, Math.toRadians(90))),
                        new KinematicReachAction(arm, KinematicReachAction.State.PICKUP, 30, 4)),
                new KinematicReachAction(arm, KinematicReachAction.State.PICKUP, 30, 0),
                new ClawAction(arm, false),
                new BasketAutoPilotAction(arm, autopilot),
                new ClawAction(arm, true));

//                // Search in the submersible:
//                copilot.search(),
//                copilot.basket(),
//                arm.openClaw(true)



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            Stats.beginLoop(Stats.Mode.REGULAR);
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Update the current pose:
            drive.updatePoseEstimate();
            if (!autoAction.run(packet)) {
                break;
            }

            Canvas canvas = packet.fieldOverlay();
            arm.update(drive.pose, canvas);
            poser.draw(canvas);

            MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
        Stats.suspend();
    }
}
