package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team6220.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;

import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team6220.roadrunner.RobotAction;

import java.util.Objects;

@Autonomous(name="TextInputAuto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {

    // defaults so it doesnt explode if you skip the text menu
    private AutonomousEnums.AutoStartPosition autoStartPosition = null;
    private AutonomousEnums.AutoType autoType = null;
    private AutonomousEnums.ParkPosition parkPosition = null;
    private AutonomousEnums.SpikeMarkSide spikeMarkSide = null;

    @Override
    public void runOpMode() {

        // initialize hardware :>
        initializeHardware();

        // TextMenu implementation yoinked from valsei's GitHub
        TextMenu startingConditionMenu = new TextMenu();
        MenuInput input = new MenuInput(MenuInput.InputType.CONTROLLER);

        startingConditionMenu.add("Select Starting Conditions")
                .add("Start Position: ")
                .add("start_position", AutonomousEnums.AutoStartPosition.class)
                .add("Auto Type: ")
                .add("auto_type", AutonomousEnums.AutoType.class)
                .add("confirm_selection", new MenuFinishedButton());

        // update the starting condition menu until it's done
        textMenuUpdateUntilComplete(startingConditionMenu, input);

        // get values from textmenu
        autoStartPosition = startingConditionMenu.getResult(AutonomousEnums.AutoStartPosition.class, "start_position");
        autoType = startingConditionMenu.getResult(AutonomousEnums.AutoType.class, "auto_type");

        // initialize mecanumdrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, autoStartPosition.startingPose);

        TextMenu scoringSelectionMenu = new TextMenu();

        // declare new textmenu
        scoringSelectionMenu.add("Scoring Settings: ")
                .addTextConditional("SpikeMark Side: ", autoType.equals(AutonomousEnums.AutoType.SCORING))
                .addEnumConditional("spikemark_side", AutonomousEnums.SpikeMarkSide.class, autoType.equals(AutonomousEnums.AutoType.SCORING))
                .add("Park Position: ")
                .add("park_position", AutonomousEnums.ParkPosition.class)
                .add("confirm_selection", new MenuFinishedButton());

        // update the scoring selection menu until it's done
        textMenuUpdateUntilComplete(scoringSelectionMenu, input);

        // only assign values if they are enabled for selected autotype
        if (autoType.equals(AutonomousEnums.AutoType.SCORING)) {
            spikeMarkSide = scoringSelectionMenu.getResult(AutonomousEnums.SpikeMarkSide.class, "spikemark_side");
        }
        parkPosition = scoringSelectionMenu.getResult(AutonomousEnums.ParkPosition.class, "park_position");

        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!
        // Scoring trajectories
        Action trajectoryAction = computeAutoPath(drive);

        // Get a preview of the trajectory's path:
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        // commented out so nothing's borked on accident :)
        // drive.runParallel(elbowAction.setTargetPosition(DRIFTConstants.ARM_ELBOW_SERVO_PRESET_POSITION_OVER_BARRIER));

        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();

            // Update and draw the arm simulator:
            slideAndDumperSim.update(packet.fieldOverlay(), drive.pose);
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

    private Action computeAutoPath(MecanumDrive drive) {
        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(autoStartPosition.startingPose);

        // apply scoring portion of auto
        if (Objects.requireNonNull(autoType) == AutonomousEnums.AutoType.SCORING) {
            // so much cleaner omg such wow much yes
            // yoink the auto scoring route from the enum
            actionBuilder = spikeMarkSide.appendScoringRoute(actionBuilder);

            // still putting the wait here because yeah
            actionBuilder = actionBuilder.waitSeconds(3);
        }

        // apply parking portion of auto
        actionBuilder = parkPosition.appendAutonomousSegment(actionBuilder, autoType, hardwareMap);

        return actionBuilder.endTrajectory().build();
    }
}
