package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="ScoringAutoMiddlePushbot", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class ScoringAutoMiddlePushbot extends BaseOpMode {


    @Override
    public void runOpMode() {

        // overridden in subclasses, middle by default
        Pose2d startingPose = getInitializationPose();

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, startingPose);

        Action middleScoringTrajectory = drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(55, 60))
                .endTrajectory()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(45, 10, Math.toRadians(-90)),  Math.toRadians(-40))
                .endTrajectory()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-140)),  Math.toRadians(40))
                .endTrajectory()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(55, 10, Math.toRadians(-90)),  Math.toRadians(-40))
                .endTrajectory()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-140)),  Math.toRadians(40))
                .endTrajectory()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(62, 10, Math.toRadians(-90)),  Math.toRadians(40))
                .endTrajectory()
                .strafeTo(new Vector2d(62, 55))
                //.setTangent(Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(62, 55, Math.toRadians(-140)),  Math.toRadians(40))
                //.strafeTo(new Vector2d(35, 60))
                //.strafeTo(new Vector2d(35, 15))
                //.strafeTo(new Vector2d(45, 15))
                //.splineToConstantHeading(new Vector2d(55, 60), Math.toRadians(0))

//                .strafeTo(new Vector2d(36, 60))
//                .endTrajectory()
//                .setTangent(Math.toRadians(-90))
//                .splineTo(new Vector2d(36, 12), Math.toRadians(-90))
////                .strafeTo(new Vector2d(48, 12))
//                .splineToLinearHeading(new Pose2d(48, 12, -130), -130)
//                .endTrajectory()
//                .setTangent(Math.toRadians(-130))
//                .splineTo(new Vector2d(55, 50), Math.toRadians(-130))
                .build();
        Action trajectoryAction = middleScoringTrajectory;

        // Get a preview of the trajectory's path:
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();

            // Draw the preview and then run the next step of the trajectory on top:
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            more = trajectoryAction.run(packet);

            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    private void textMenuUpdateUntilComplete(TextMenu textMenu, MenuInput input) {
        while (!textMenu.isCompleted() && !isStopRequested()) {
            for (String line : textMenu.toListOfStrings()) {
                telemetry.addLine(line);
            }
            telemetry.update();

            input.update(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.dpad_left, gamepad1.dpad_right,
                    gamepad1.dpad_down, gamepad1.dpad_up,
                    gamepad1.a
            );
            textMenu.updateWithInput(input);
            sleep(17);
        }
    }

    private <E extends Enum> void runIfNotNull (Class<E> enumClass, Runnable runnable) {
        if (enumClass != null) {

        }
    }

    protected Pose2d getInitializationPose() {
        return Constants.MIDDLE_STARTING_POSE;
    }
}
