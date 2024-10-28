package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name = "Auto", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);

        Action trajectoryAction;

        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!
        if (false) {
            trajectoryAction = drive.actionBuilder(new Pose2d(-24.00, 60.00, Math.PI * 1.5))
                    .splineToLinearHeading(new Pose2d(-48.00, 60.00, Math.PI * 1.5), Math.PI)
                    .splineToLinearHeading(new Pose2d(-60.00, 48.00, Math.PI * 1.6), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(-60.00, 60.00, Math.PI * 1.5), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(-60.00, 48.00, Math.PI * 1.5), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(-60.00, 60.00, Math.PI * 1.5), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(-60.00, 48.00, Math.PI * 1.4), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(-60.00, 60.00, Math.PI * 1.5), Math.PI / 2)
                    .build();
        } else {
            trajectoryAction = drive.actionBuilder(new Pose2d(24.00, 60.00, Math.PI * 1.5))
                    .splineToLinearHeading(new Pose2d(48.00, 60.00, Math.PI * 1.5), 0)
                    .splineToLinearHeading(new Pose2d(60.00, 48.00, Math.PI * 1.4), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(60.00, 60.00, Math.PI * 1.25), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(60.00, 48.00, Math.PI * 1.5), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(60.00, 60.00, Math.PI * 1.25), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(60.00, 48.00, Math.PI * 1.6), Math.PI / 2)
                    .splineToLinearHeading(new Pose2d(60.00, 60.00, Math.PI * 1.25), Math.PI / 2)
                    .build();
        }

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
            telemetry.addData("Kinematic Type", kinematicType);

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
}
