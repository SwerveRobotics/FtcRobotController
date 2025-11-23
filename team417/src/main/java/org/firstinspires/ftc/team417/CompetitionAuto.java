package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417.apriltags.AprilTagDetector;
import org.firstinspires.ftc.team417.apriltags.Pattern;
import org.firstinspires.ftc.team417.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team417.javatextmenu.MenuHeader;
import org.firstinspires.ftc.team417.javatextmenu.MenuInput;
import org.firstinspires.ftc.team417.javatextmenu.MenuSlider;
import org.firstinspires.ftc.team417.javatextmenu.TextMenu;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="Auto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {
    public enum Alliance {
        RED,
        BLUE,
    }

    enum Movement {
        NEAR,
        FAR,
        FAR_MINIMAL,
    }

    double minWaitTime = 0.0;
    double maxWaitTime = 30.0;

    @Override
    public void runOpMode() {
        initHardware();
        Pose2d startPose = new Pose2d(0, 0, 0);

        Pose2d redNearStartPose = new Pose2d(-60, 48, Math.toRadians(41));
        Pose2d redFarStartPose = new Pose2d(64, 16, Math.toRadians(0));

        Pose2d blueNearStartPose = new Pose2d(-50, -50.5, Math.toRadians(139));
        Pose2d blueFarStartPose = new Pose2d(64, -16, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, startPose);

        // Test to make sure the camera is there, and then immediately close the detector object
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.initAprilTag(hardwareMap);
        }

        TextMenu menu = new TextMenu();
        MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);

        menu.add(new MenuHeader("AUTO SETUP"))
                .add() // empty line for spacing
                .add("Pick an alliance:")
                .add("alliance-picker-1", Alliance.class) // enum selector shortcut
                .add()
                .add("Pick a movement:")
                .add("movement-picker-1", Movement.class) // enum selector shortcut
                .add()
                .add("Wait time:")
                .add("wait-slider-1", new MenuSlider(minWaitTime, maxWaitTime))
                .add()
                .add("finish-button-1", new MenuFinishedButton());

        while (!menu.isCompleted()) {
            // get x, y (stick) and select (A) input from controller
            // on wilyworks, this is x, y (wasd) and select (enter) on the keyboard
            menuInput.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.a);
            menu.updateWithInput(menuInput);
            // display the updated menu
            for (String line : menu.toListOfStrings()) {
                telemetry.addLine(line); // but with appropriate printing method
            }
            telemetry.update();
        }

        // the first parameter is the type to return as
        Alliance chosenAlliance = menu.getResult(Alliance.class, "alliance-picker-1");
        Movement chosenMovement = menu.getResult(Movement.class, "movement-picker-1");
        double waitTime = menu.getResult(Double.class, "wait-slider-1");

        // Red alliance auto paths
        Action redNear = drive.actionBuilder(redNearStartPose)
                .stopAndAdd(new SpinUpAction(1220))
                .setTangent(Math.toRadians(-49))
                .splineToLinearHeading(new Pose2d(-36,36, Math.toRadians(36)),Math.toRadians(-49))

                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-28,54,Math.toRadians(0)), Math.toRadians(90))
                .build();

        Action redFar = drive.actionBuilder(redFarStartPose)
                /*.setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, 48, Math.toRadians(41)), Math.toRadians(90))
                .stopAndAdd(new SpinUpAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(0)), Math.toRadians(-90))
                .build();*/
                .stopAndAdd(new SpinUpAction(1220))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(36)), Math.toRadians(139))

                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, 12, Math.toRadians(0)), Math.toRadians(180))
                .build();

        Action redFarMinimal = drive.actionBuilder(redFarStartPose)
                .setTangent(Math.PI/2)
                .splineTo(new Vector2d(56, 35), Math.PI/2)
                .build();

        // Blue alliance auto paths
        Action blueNear = drive.actionBuilder(blueNearStartPose)
                .stopAndAdd(new SpinUpAction(1220))
                .setTangent(Math.toRadians(49))
                .splineToLinearHeading(new Pose2d(-36,-36, Math.toRadians(134)),Math.toRadians(49))

                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
//                .splineTo(new Vector2d(-44, -44), Math.toRadians(49))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-28,-54 ,Math.toRadians(180)), Math.toRadians(-90))
                .build();

        Action blueFar = drive.actionBuilder(blueFarStartPose)
                .setTangent(Math.toRadians(-135))
                .stopAndAdd(new SpinUpAction(1220))
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(134)), Math.toRadians(-139))

                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .stopAndAdd(new LaunchAction())
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(-180)), Math.toRadians(90))
                .build();

        Action blueFarMinimal = drive.actionBuilder(blueFarStartPose)
                .setTangent(3*Math.PI/2)
                .splineTo(new Vector2d(56, -35), 3*Math.PI/2)
                .build();


        Action trajectoryAction = null;
        switch (chosenAlliance) {
            case RED:
                switch (chosenMovement) {
                    case NEAR:
                        drive.setPose(redNearStartPose);
                        trajectoryAction = redNear;
                        break;
                    case FAR:
                        drive.setPose(redFarStartPose);
                        trajectoryAction = redFar;
                        break;
                    default:
                        drive.setPose(redFarStartPose);
                        trajectoryAction = redFarMinimal;
                        break;
                }
                break;

            default:
                switch (chosenMovement) {
                    case NEAR:
                        drive.setPose(blueNearStartPose);
                        trajectoryAction = blueNear;
                        break;
                    case FAR:
                        drive.setPose(blueFarStartPose);
                        trajectoryAction = blueFar;
                        break;
                    default:
                        drive.setPose(blueFarStartPose);
                        trajectoryAction = blueFarMinimal;
                        break;
                }
                break;
        }

        // Get a preview of the trajectory's path:
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);


        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!

        // Assume unknown pattern unless detected otherwise.
        Pattern pattern = Pattern.UNKNOWN;

        // Detect the pattern with the AprilTags from the camera!
        // Wait for Start to be pressed on the Driver Hub!
        try (AprilTagDetector detector = new AprilTagDetector()) {
            detector.initAprilTag(hardwareMap);

            while (!isStarted() && !isStopRequested()) {
                Pattern last = detector.detectPattern(chosenAlliance);
                if (last != Pattern.UNKNOWN) {
                    pattern = last;
                }

                telemetry.addData("Chosen alliance: ", chosenAlliance);
                telemetry.addData("Chosen movement: ", chosenMovement);
                telemetry.addData("Chosen wait time: ", waitTime);
                telemetry.addData("Last valid pattern: ", pattern);

                telemetry.update();
            }
        }

        sleep((long)waitTime*1000);
        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");

            telemetry.addData("State", launchState);
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("motorSpeed", launcher.getVelocity());
            telemetry.addData("FEED_TIME_SECONDS", FEED_TIME_SECONDS);
            telemetry.addData("leftFeeder", leftFeeder.getPower());
            telemetry.addData("rightFeeder", rightFeeder.getPower());


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
