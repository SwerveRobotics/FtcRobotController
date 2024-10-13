package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.MenuSelection;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;

import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="Auto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {

    private AutonomousEnums.AllianceColor allianceColor;
    private AutonomousEnums.AutoStartPosition autoStartPosition;
    private AutonomousEnums.AutoType autoType;
    private AutonomousEnums.ParkPosition parkPosition;
    private AutonomousEnums.SpikeMarkPickupAmount pickupAmount;
    private AutonomousEnums.SpikeMarkSide spikeMarkSide;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(0, 60, (3*Math.PI)/2);
        Pose2d beginPose1 = new Pose2d(-20, 60, (3*Math.PI)/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);

        // TextMenu implementation yoinked from valsei's GitHub
        TextMenu startingConditionMenu = new TextMenu();
        MenuInput input = new MenuInput(MenuInput.InputType.CONTROLLER);

        startingConditionMenu.add("Select Starting Conditions")
                .add("Alliance Color: ")
                .add("alliance_color", AutonomousEnums.AllianceColor.class)
                .add("Start Position: ")
                .add("start_position", AutonomousEnums.AutoStartPosition.class)
                .add("Auto Type: ")
                .add("auto_type", AutonomousEnums.AutoType.class)
                .add("confirm_selection", new MenuFinishedButton());

        // yoinked from valsei's github
        while (!startingConditionMenu.isCompleted() && !isStopRequested()) {
            for (String line : startingConditionMenu.toListOfStrings()) {
                telemetry.addLine(line);
            }
            telemetry.update();

            input.update(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.dpad_left, gamepad1.dpad_right,
                    gamepad1.dpad_down, gamepad1.dpad_up,
                    gamepad1.a
            );
            startingConditionMenu.updateWithInput(input);
            sleep(17);
        }

        allianceColor = startingConditionMenu.getResult(AutonomousEnums.AllianceColor.class, "alliance_color");
        autoStartPosition = startingConditionMenu.getResult(AutonomousEnums.AutoStartPosition.class, "start_position");
        autoType = startingConditionMenu.getResult(AutonomousEnums.AutoType.class, "auto_type");

        TextMenu scoringSelectionMenu = new TextMenu();

        scoringSelectionMenu.add("Scoring Settings: ")
                .add("SpikeMark Side: ")
                .addEnumConditional("spikemark_side", AutonomousEnums.SpikeMarkSide.class, autoType.equals(AutonomousEnums.AutoType.BASKET))
                .add("Sample Pickup Quantity:")
                .addEnumConditional("sample_pickup_quantity", AutonomousEnums.SpikeMarkPickupAmount.class, autoType.equals(AutonomousEnums.AutoType.BASKET))
                .add("Park Position: ")
                .add("park_position", AutonomousEnums.ParkPosition.class)
                .add("confirm_selection", new MenuFinishedButton());

        while (!scoringSelectionMenu.isCompleted() && !isStopRequested()) {
            for (String line : scoringSelectionMenu.toListOfStrings()) {
                telemetry.addLine(line);
            }
            telemetry.update();

            input.update(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.dpad_left, gamepad1.dpad_right,
                    gamepad1.dpad_down, gamepad1.dpad_up,
                    gamepad1.a
            );
            scoringSelectionMenu.updateWithInput(input);
            sleep(17);
        }

        if (autoType.equals(AutonomousEnums.AutoType.BASKET)) {
            spikeMarkSide = scoringSelectionMenu.getResult(AutonomousEnums.SpikeMarkSide.class, "spikemark_side");
            pickupAmount = scoringSelectionMenu.getResult(AutonomousEnums.SpikeMarkPickupAmount.class, "sample_pickup_quantity");
        }
        parkPosition = scoringSelectionMenu.getResult(AutonomousEnums.ParkPosition.class, "park_position");

        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!
        Action trajectoryAction1     = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(48, 36), (3*Math.PI)/2)
                .endTrajectory()
                .splineToSplineHeading(new Pose2d(48, 50, Math.PI/4), (3*Math.PI)/2)
                .build();
        Action trajectoryAction2 = drive.actionBuilder(beginPose1)
                .splineTo(new Vector2d(48, 36), (3*Math.PI)/2)
                .splineToSplineHeading(new Pose2d(48, 50, Math.PI/4), (3*Math.PI)/2)
                .build();
        Action trajectoryAction = trajectoryAction2;

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
            telemetry.update();

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();

            // Draw the preview and then run the next step of the trajectory on top:
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            more = trajectoryAction.run(packet);

            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                MecanumDrive.sendTelemetryPacket(packet);
        }
    }
}
