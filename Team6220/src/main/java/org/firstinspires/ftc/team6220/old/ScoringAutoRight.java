package org.firstinspires.ftc.team6220.old;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220.BaseOpMode;
import org.firstinspires.ftc.team6220.DRIFTConstants;
import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;
import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */

@Disabled
@Autonomous(name="ScoringAutoRight", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class ScoringAutoRight extends BaseOpMode {


    @Override
    public void runOpMode() {

        Pose2d rightPose = DRIFTConstants.RIGHT_STARTING_POSE;


        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, rightPose);

        Action rightScoringTrajectory = drive.actionBuilder(rightPose)
                .splineTo(new Vector2d(48, 36), (3*Math.PI)/2)
                .endTrajectory()
                .splineTo(new Vector2d(48, 50), (5*Math.PI)/4) //scored a sample
                .splineTo(new Vector2d(25, 10),(3*Math.PI)/2)
                .splineTo(new Vector2d(28, 10),(1*Math.PI)/1)
                .build();

        Action trajectoryAction = rightScoringTrajectory;

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
}