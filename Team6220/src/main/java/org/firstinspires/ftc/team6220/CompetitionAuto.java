package org.firstinspires.ftc.team6220;

//import android.transition.Slide;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team6220.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team6220.javatextmenu.MenuInput;
import org.firstinspires.ftc.team6220.javatextmenu.TextMenu;

import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team6220.roadrunner.RobotAction;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous(name="CompetitionAuto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {

    // defaults so it doesnt explode if you skip the text menu
    private AutonomousEnums.AutoStartPosition autoStartPosition = AutonomousEnums.AutoStartPosition.LEFT;
    private AutonomousEnums.AutoType autoType = AutonomousEnums.AutoType.SCORING;
    private AutonomousEnums.ParkPosition parkPosition = AutonomousEnums.ParkPosition.OBSERVATION;
    private AutonomousEnums.SpikeMarkSide spikeMarkSide = AutonomousEnums.SpikeMarkSide.LEFT;

    @Override
    public void runOpMode() {

        // initialize hardware :>
        initializeHardware();

        // lock dumper servo to init position
        dumperServo.setPosition(DRIFTConstants.DUMPER_SERVO_POSITION_INIT);

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

        dumperServo.setPosition(DRIFTConstants.DUMPER_SERVO_POSITION_TRANSFER / 2);
        dumperServo.setPosition(DRIFTConstants.DUMPER_SERVO_POSITION_TRANSFER);

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
        while (!textMenu.isCompleted() && !isStopRequested() && !this.isStarted()) {
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
        slidesMotor.setVelocity(1500);
        actionBuilder = actionBuilder
                .stopAndAdd(new SlideMoveAction(SlideActionState.OVER_HIGH_CHAMBER))
                .setTangent((3*Math.PI)/2)
                .splineToLinearHeading(new Pose2d(0, 33.5, Math.toRadians(90)), Math.toRadians(270))
                .stopAndAdd(new SlideMoveAction(SlideActionState.HIGH_CHAMBER))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(0, 36, Math.toRadians(90)), Math.toRadians(270))
                .stopAndAdd(new SlideMoveAction(SlideActionState.OVER_HIGH_CHAMBER))
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(90)), Math.toRadians(270))

                .stopAndAdd(new DumperMoveAction(DumperActionState.TRANSFER))
                .afterDisp(0, new SlideMoveAction(SlideActionState.GROUND))

                .splineToLinearHeading(new Pose2d(0, 62, Math.toRadians(90)), Math.toRadians(270))

                // spline to prepare to collect first sample
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(40, 24, Math.toRadians(270)),  3 * Math.PI / 2)
                .splineToLinearHeading(new Pose2d(50, 9, Math.toRadians(270)),  Math.toRadians(340))
                .endTrajectory()
                .setTangent(Math.toRadians(90))

                // spline to deposit first sample in scoring area
                .splineToLinearHeading(new Pose2d(55, 60, Math.toRadians(220)),  Math.toRadians(40))
                .endTrajectory()
                .setTangent(Math.toRadians(180))

                // spline to prepare to collect second sample
                .splineToLinearHeading(new Pose2d(60, 10, Math.toRadians(270)),  Math.toRadians(330))
                .endTrajectory()
                .setTangent(Math.toRadians(90))

                // spline to deposit second sample in scoring area
                .splineToLinearHeading(new Pose2d(61, 55, Math.toRadians(220)),  Math.toRadians(40))
                .endTrajectory()
                .setTangent(Math.toRadians(-80))

                // spline to prepare to collect third sample
                .splineToLinearHeading(new Pose2d(70, 12, Math.toRadians(270)),  Math.toRadians(40))
                .endTrajectory()

                // strafe to deposit third sample
                .strafeTo(new Vector2d(70, 52))

                // prepare to move to park position and move dumper and arm to init position for teleop
                .splineToLinearHeading(new Pose2d(62, 50, Math.PI), 0) //change this in backup code too if it works!
                .endTrajectory()
                .setTangent(Math.toRadians(200))

                .endTrajectory();

        /*// apply scoring portion of auto
        if (Objects.requireNonNull(autoType) == AutonomousEnums.AutoType.SCORING) {
            switch (spikeMarkSide) {
                case LEFT: {

                    // declarations for the compound arm actions
                    ArrayList<RobotAction> GRAB = new ArrayList<>();
                    // GRAB.add();
                    ArrayList<RobotAction> TRANSFER = new ArrayList<>();
                    ArrayList<RobotAction> IDLE = new ArrayList<>();

                    actionBuilder = actionBuilder
                            // Go to high basket and raise slides
                            .afterDisp(0, new SlideMoveAction(SlideActionState.HIGH_BASKET))
                            .splineToLinearHeading(new Pose2d(59, 57, Math.toRadians(225)), 0)

                            // Lift slides and dump
                            .stopAndAdd(new DumperMoveAction(DumperActionState.DUMP))

                            // Reset dumper and lower slides
                            .waitSeconds(2)

                            .stopAndAdd(new DumperMoveAction(DumperActionState.TRANSFER))
                            .afterDisp(0, new SlideMoveAction(SlideActionState.GROUND))

                            // spline to prepare to collect first sample
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(40, 24, Math.toRadians(270)),  3 * Math.PI / 2)
                            .splineToLinearHeading(new Pose2d(50, 9, Math.toRadians(270)),  Math.toRadians(340))
                            .endTrajectory()
                            .setTangent(Math.toRadians(90))

                            // spline to deposit first sample in scoring area
                            .splineToLinearHeading(new Pose2d(55, 60, Math.toRadians(220)),  Math.toRadians(40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(180))

                            // spline to prepare to collect second sample
                            .splineToLinearHeading(new Pose2d(60, 10, Math.toRadians(270)),  Math.toRadians(330))
                            .endTrajectory()
                            .setTangent(Math.toRadians(90))

                            // spline to deposit second sample in scoring area
                            .splineToLinearHeading(new Pose2d(61, 55, Math.toRadians(220)),  Math.toRadians(40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(-80))

                            // spline to prepare to collect third sample
                            .splineToLinearHeading(new Pose2d(70, 12, Math.toRadians(270)),  Math.toRadians(40))
                            .endTrajectory()

                            // strafe to deposit third sample
                            .strafeTo(new Vector2d(70, 52))

                            // prepare to move to park position and move dumper and arm to init position for teleop
                            .splineToLinearHeading(new Pose2d(62, 50, Math.PI), 0) //change this in backup code too if it works!
                            .endTrajectory()
                            .setTangent(Math.toRadians(200));


                }
                case RIGHT: {
                    // to be implemented
                }
            }
        }



        // apply parking portion of auto
        switch (parkPosition) {
            case OBSERVATION: {
                switch(autoType) {
                    case PARK: {
                        actionBuilder = actionBuilder.strafeTo(parkPosition.parkingPosition)
                                .endTrajectory();
                    }
                    case SCORING: {
                        actionBuilder = actionBuilder.splineTo(parkPosition.parkingPosition, Math.toRadians(170))
                                .endTrajectory();
                    }
                }
            }

            case SUBMERSIBLE: {

                switch (autoType) {
                    case PARK: {
                        // code for submersible park auto goes here
                    }
                    case SCORING: {
                        // code for submersible scoring-park auto goes here
                    }
                }
            }
        }*/


        return actionBuilder.endTrajectory().build();
    }
}
