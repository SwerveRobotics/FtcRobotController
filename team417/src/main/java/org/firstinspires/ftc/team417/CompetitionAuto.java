package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;
import org.firstinspires.ftc.team417.apriltags.Pattern;
import org.firstinspires.ftc.team417.javatextmenu.MenuFinishedButton;
import org.firstinspires.ftc.team417.javatextmenu.MenuHeader;
import org.firstinspires.ftc.team417.javatextmenu.MenuInput;
import org.firstinspires.ftc.team417.javatextmenu.MenuSlider;
import org.firstinspires.ftc.team417.javatextmenu.MenuSwitch;
import org.firstinspires.ftc.team417.javatextmenu.TextMenu;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */

class BaseCompetitonMode extends BaseOpMode {
    static public double FEEDER_TIME = 0.5;
    static public double ROBOT_SPEED = 25;
    static public double INTAKE_SPEED = 1;

    public enum Alliance {
        RED,
        BLUE,
    }

    enum SlowBotMovement {
        NEAR,
        FAR,
        FAR_OUT_OF_WAY,
        FAR_MINIMAL,
    }

    double minWaitTime = 0.0;
    double maxWaitTime = 30.0;

    double minIntakes = 0.0;
    double maxIntakes = 6.0;
    TextMenu menu = new TextMenu();
    MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);
    Pattern pattern;
    class SpinUpAction extends RobotAction {

        double velocity;
        public SpinUpAction(double velocity) {
            this.velocity = velocity;
        }
        @Override
        public boolean run(double elapsedTime) {
            upperFlywheelMot.setVelocity(velocity-FLYWHEEL_BACKSPIN);
            lowerFlywheelMot.setVelocity(velocity+FLYWHEEL_BACKSPIN);
            return false;
        }
    }
    class LaunchAction extends RobotAction {
        @Override
        public boolean run(double elapsedTime) {
            if (elapsedTime < 1.5) {
                transferWheelMot.setPower(1);
                intakeMot.setPower(1);
                return true;
            }
            else {
                transferWheelMot.setPower(0);
                intakeMot.setPower(0);
                return false;
            }
        }

    }
    class WaitAction extends RobotAction {
        double time;
        public WaitAction(double time) {
            this.time = time;
        }
        @Override
        public boolean run(double elapsedTime) {
            return elapsedTime < time;
        }
    }



    class IntakeAction extends RobotAction {
        double time;
        public IntakeAction(double time) {
            this.time = time;
        }

        @Override
        public boolean run(double elapsedTime) {
            if (elapsedTime < time) {
                intakeMot.setPower(1);
                return true;
            }
            else {
                intakeMot.setPower(0);
                return false;
            }

        }
    }
    class WaitOnAction extends RobotAction {
        RobotAction actionToWaitOn;
        WaitOnAction(RobotAction actionToWaitOn) {
            this.actionToWaitOn = actionToWaitOn;
        }

        @Override
        public boolean run(double elapsedTime) {
            return actionToWaitOn.isRunning();
        }
    }

    public Action getPath(SlowBotMovement chosenMovement, Alliance chosenAlliance, double intakeCycles, MecanumDrive drive) {
        Pose2d beginPose = drive.pose;
        PoseMap poseMap = pose -> new Pose2dDual<>(

                pose.position.x,
                pose.position.y,
                pose.heading);
        if (chosenAlliance == Alliance.BLUE) {
            beginPose = new Pose2d(drive.pose.position.x, -drive.pose.position.y, -drive.pose.heading.log());
            poseMap = pose -> new Pose2dDual<>(
                    pose.position.x,
                    pose.position.y.unaryMinus(),
                    pose.heading.inverse());
        }
        TrajectoryActionBuilder trajectoryAction = null;
        switch (chosenMovement) {
            case NEAR:
                trajectoryAction = drive.actionBuilder(beginPose, poseMap);
                trajectoryAction = trajectoryAction.setTangent(Math.toRadians(-51))
                        .stopAndAdd(new SpinUpAction(FLYWHEEL_NEAR_SPEED))
                        .splineToSplineHeading(new Pose2d(-12, 12,Math.toRadians(135)), Math.toRadians(-51))
                        .stopAndAdd(new LaunchAction())
                        .stopAndAdd(new WaitAction(FEEDER_TIME))
                        .setTangent(Math.toRadians(90))
                        .afterDisp(0,new IntakeAction(6))
                        .splineToSplineHeading(new Pose2d(12, 32, Math.toRadians(90)), Math.toRadians(90)) //go to intake closest from goal
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(135)), Math.toRadians(-90)) //go to launch position
                        .stopAndAdd(new LaunchAction())
                        .stopAndAdd(new WaitAction(FEEDER_TIME));
                for (int i = 0; i < intakeCycles; i++) {
                    trajectoryAction = trajectoryAction.setTangent(Math.toRadians(45))
                            .afterDisp(0, new IntakeAction(8))
                            .splineToSplineHeading(new Pose2d(0, 57, Math.toRadians(90)), Math.toRadians(90)) //go to intake middle from goal
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(20,59,Math.toRadians(120)),Math.toRadians(120))
                            .setTangent(Math.toRadians(-123))
                            .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(135)), Math.toRadians(180)); //go to launch position

                }
                trajectoryAction = trajectoryAction.stopAndAdd(new WaitAction(FEEDER_TIME))
                        .setTangent(Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(0,24), Math.toRadians(45));

                break;

            case FAR:
                trajectoryAction = drive.actionBuilder(beginPose, poseMap);

                    trajectoryAction = trajectoryAction.setTangent(Math.toRadians(157.5))
                            .stopAndAdd(new SpinUpAction(FLYWHEEL_FAR_SPEED))
                            .stopAndAdd(new WaitOnAction(new SpinUpAction(FLYWHEEL_FAR_SPEED)))
                            .splineToSplineHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-90))  //go to launch position
                            .stopAndAdd(new LaunchAction())
                            .stopAndAdd(new WaitAction(FEEDER_TIME));

                trajectoryAction = trajectoryAction.splineToSplineHeading(new Pose2d(36, 32, Math.toRadians(90)), Math.toRadians(90)) //go to intake farthest from goal
                        .afterDisp(0,new IntakeAction(6))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(36, 60), Math.toRadians(90), new TranslationalVelConstraint(ROBOT_SPEED))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-90))  //go to launch position
                        .stopAndAdd(new LaunchAction())
                        .stopAndAdd(new WaitAction(FEEDER_TIME));
                for(int i = 0; i < intakeCycles; i++) {
                    trajectoryAction = trajectoryAction.setTangent(Math.toRadians(90))
                            .afterDisp(0, new IntakeAction(9))
                            .splineToSplineHeading(new Pose2d(62, 62, Math.toRadians(90)), Math.toRadians(90)) //go to intake middle from goal
                            .setTangent(Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(57, 62), Math.toRadians(-90))
                            .setTangent(Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(62,62),Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-90)) //go to launch position
                            .stopAndAdd(new WaitAction(FEEDER_TIME));
                }
                trajectoryAction = trajectoryAction.setTangent(Math.toRadians(157.5))
                        .splineToConstantHeading(new Vector2d(48,24), Math.toRadians(157.5));
                break;

            case FAR_OUT_OF_WAY:
                // 3 launch actions
                // after disp intake action
                trajectoryAction = drive.actionBuilder(beginPose, poseMap);
                        trajectoryAction.splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-157.5))
                        .stopAndAdd(new WaitAction(FEEDER_TIME))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(60, 61, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(60,46,Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(71-(ROBOT_WIDTH/2), 61, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-90))
                        .stopAndAdd(new WaitAction(FEEDER_TIME));

                if (intakeCycles > 1) {
                    trajectoryAction = trajectoryAction.setTangent(Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(60, 61, Math.toRadians(90)), Math.toRadians(90))
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(60,46,Math.toRadians(90)), Math.toRadians(-90))
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(71-(ROBOT_WIDTH/2), 61, Math.toRadians(90)), Math.toRadians(90))
                            .setTangent(Math.toRadians(-90))
                            .splineToLinearHeading(new Pose2d(54, 12, Math.toRadians(157.5)), Math.toRadians(-90))
                            .stopAndAdd(new WaitAction(FEEDER_TIME));
                }
                trajectoryAction = trajectoryAction.setTangent(Math.toRadians(157.5))
                        .splineToConstantHeading(new Vector2d(48,24), Math.toRadians(157.5));
                break;
            case FAR_MINIMAL:
                trajectoryAction = drive.actionBuilder(beginPose, poseMap);
                trajectoryAction = trajectoryAction.setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(48, 32, Math.toRadians(180)), Math.toRadians(180));

                break;
        }
        return trajectoryAction.build();


    }

    @Override
    public void runOpMode() {
    initializeHardware();


        // Text menu for FastBot


        // Text menu for SlowBot
        menu.add(new MenuHeader("AUTO SETUP"))
                .add() // empty line for spacing
                .add("Pick an alliance:")
                .add("alliance-picker-1", Alliance.class) // enum selector shortcut
                .add()
                .add("Pick a movement:")
                .add("movement-picker-1", SlowBotMovement.class) // enum selector shortcut
                .add()
                .add("Intake Cycles:")
                .add("intake-slider", new MenuSlider(minIntakes, maxIntakes))
                .add()
                .add("Wait time:")
                .add("wait-slider-1", new MenuSlider(minWaitTime, maxWaitTime))
                .add()
                .add("Use pose correction:")
                .add("correction-switch-1", new MenuSwitch(false))
                .add()
                .add("finish-button-1", new MenuFinishedButton());

        while (!menu.isCompleted() && !isStopRequested()) {
            // get x, y (stick) and select (A) input from controller
            // on Wily Works, this is x, y (wasd) and select (enter) on the keyboard
            menuInput.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.a);
            menu.updateWithInput(menuInput);
            // display the updated menu
            for (String line : menu.toListOfStrings()) {
                telemetry.addLine(line); // but with appropriate printing method
            }
            telemetry.update();
        }

        Alliance chosenAlliance = menu.getResult(Alliance.class, "alliance-picker-1");
        SlowBotMovement chosenMovement = menu.getResult(SlowBotMovement.class, "movement-picker-1");
        double waitTime = menu.getResult(Double.class, "wait-slider-1");
        double intakeCycles = menu.getResult(Double.class, "intake-slider");
        TransferState.usePoseCorrection = menu.getResult(Boolean.class, "correction-switch-1");
        processAuto(chosenAlliance, chosenMovement, waitTime, intakeCycles);
    }
    void processAuto(Alliance chosenAlliance, SlowBotMovement chosenMovement, double waitTime, double intakeCycles) {
        // the first parameter is the type to return as
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        Pose2d startPose = new Pose2d(0, 0, 0);

        Pose2d SBRedNearStartPose = new Pose2d(-72 + (ROBOT_WIDTH / 2), 24 + (ROBOT_LENGTH / 2), Math.toRadians(-90));
        Pose2d SBBlueNearStartPose = new Pose2d(-72 + (ROBOT_WIDTH / 2), -(24 + (ROBOT_LENGTH / 2)), Math.toRadians(90));
        Pose2d SBRedFarStartPose = new Pose2d(72 - ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, Math.toRadians(180));
        Pose2d SBBlueFarStartPose = new Pose2d(72 - ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, startPose);


        detector = new LimelightAprilTagDetector(hardwareMap, drive);

        Action trajectoryAction;
        if (chosenAlliance == Alliance.RED) {
            switch (chosenMovement) {
                case NEAR:

                    drive.setPose(SBRedNearStartPose);

                    break;
                case FAR:
                    drive.setPose(SBRedFarStartPose);
                    break;
                case FAR_OUT_OF_WAY:
                    drive.setPose(SBRedFarStartPose);
                    break;
                case FAR_MINIMAL:
                    drive.setPose(SBRedFarStartPose);
                    break;
            }
        }
        else {
            switch (chosenMovement) {
                case NEAR:

                    drive.setPose(SBBlueNearStartPose);

                    break;
                case FAR:
                    drive.setPose(SBBlueFarStartPose);
                    break;
                case FAR_OUT_OF_WAY:
                    drive.setPose(SBBlueFarStartPose);
                    break;
                case FAR_MINIMAL:
                    drive.setPose(SBBlueFarStartPose);
                    break;
            }
        }

        TransferState.trustPose = true;

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // this lets us move the robot to see the obelisk before start and after init

        sleep(100);
        while (opModeInInit()) {
            telemetry.addLine("Ok to move \n Y to start");
            telemetry.update();
            if (gamepad1.y) {
                break;
            }
            drive.updatePoseEstimate();
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);
        }
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trajectoryAction = getPath(chosenMovement, chosenAlliance, intakeCycles, drive);
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);


        // Get a preview of the trajectory's path:




        // Wait for Start to be pressed on the Driver Hub!
        while (opModeInInit()) {

                telemetry.addData("Chosen alliance: ", chosenAlliance);
                telemetry.addData("Chosen movement: ", chosenMovement);
                telemetry.addData("Chosen wait time: ", waitTime);

                    telemetry.update();

                    if (isStopRequested()) {
                        break;
                    }
                }
        sleep((long) waitTime * 1000);
        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();

            // Draw the preview and then run the next step of the trajectory on top:
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            Pose2d cameraPose = detector.detectRobotPose();
            if (cameraPose != null) {
                packet.fieldOverlay().setStroke("#3FB578");
                Drawing.drawRobot(packet.fieldOverlay(), cameraPose);
            }

            more = trajectoryAction.run(packet);
            WilyWorks.updateSimulation(0); // Advance the simulation when not driving

            detector.updateRobotYaw(drive.pose.heading.log());

            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                MecanumDrive.sendTelemetryPacket(packet);

            telemetry.update();
        }

        // Stores these so they can be transferred to teleop
        TransferState.chosenAlliance = chosenAlliance;

        detector.close();
    }
}



@Autonomous(name = "Auto", group = "Competition")
public class CompetitionAuto extends BaseCompetitonMode {

}


