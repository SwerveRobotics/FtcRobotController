package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.team417.roadrunner.HolonomicDrive;
import org.firstinspires.ftc.team417.roadrunner.KinematicType;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="Auto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {
    public static final KinematicType kinematicType = KinematicType.MECANUM;

    @Override
    public void runOpMode() {
        // Send all telemetry data to both the Driver Hub and FTC Dashboard:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        HolonomicDrive drive = new HolonomicDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);

        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!
        Action trajectoryAction = drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build();

        // Get a preview of the trajectory's path:
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = HolonomicDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        HolonomicDrive.sendTelemetryPacket(packet);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");
            telemetry.update();

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = HolonomicDrive.getTelemetryPacket();

            // Draw the preview and then run the next step of the trajectory on top:
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            more = trajectoryAction.run(packet);

            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                HolonomicDrive.sendTelemetryPacket(packet);
        }
    }
}
