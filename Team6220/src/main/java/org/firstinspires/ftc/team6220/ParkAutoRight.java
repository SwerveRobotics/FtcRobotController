package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="ParkAutoRight", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class ParkAutoRight extends BaseOpMode {

    @Override
    public void runOpMode() {

        Pose2d rightPose = Constants.RIGHT_STARTING_POSE;

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, rightPose);

        // Create right parking trajectory
        Action  rightParkingTrajectory = drive.actionBuilder(rightPose)
                .splineTo(new Vector2d(-60, 60), (Math.PI))
                .build();
        Action trajectoryAction = rightParkingTrajectory;

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
}