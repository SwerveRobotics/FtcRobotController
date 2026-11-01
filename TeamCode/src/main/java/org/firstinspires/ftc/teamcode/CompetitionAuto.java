package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.menu.MenuFinishedButton;
import org.firstinspires.ftc.teamcode.menu.MenuHeader;
import org.firstinspires.ftc.teamcode.menu.MenuInput;
import org.firstinspires.ftc.teamcode.menu.MenuSlider;
import org.firstinspires.ftc.teamcode.menu.TextMenu;

import static com.pedropathing.ivy.groups.Groups.parallel;
@Autonomous(name = "417 Auto")
public class CompetitionAuto extends BaseOpMode{
    //when you select the class
    private Follower follower;
    //the values of the paths
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose[] poses1 = new Pose[9]; //this holds the poses for the garden path
    private Pose[] poses2 = new Pose[7]; //this holds the poses for the flower path
    private Pose autoParkingStartPose;
    private Pose shootPose; //small move forward before the first garden shot
    Alliance chosenAlliance;


    private final ElapsedTime timer = new ElapsedTime();

    public enum AutoPath { //different options for auto routine
        GARDEN_PATH,
        FLOWER_PATH,
        PARKING_PATH
    }
    public double waitTimeMs; //how long we wait at the start
    //what happens when you press the start button on the driver hub

    //create paths for GARDEN_PATH, telling the paths to go in order, intaking and shooting

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware
        initializeHardware();
        //Making follower to tell the robot
        TextMenu menu = new TextMenu();
        MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);
        menu.add(new MenuHeader("Auto Setup"))
                .add() //empty line
                .add("Pick alliance:")
                .add("alliancePicker", Alliance.class)
                .add()
                .add("Pick a path:")
                .add("pathPicker", AutoPath.class) // enum selector shortcut
                .add()
                .add("Wait time:")
                .add("waitSlider", new MenuSlider(0.0, 30.0))
                .add()
                .add("finishButton", new MenuFinishedButton());
        while (!menu.isCompleted() && !isStopRequested()) {
            // get x, y (stick) and select (A) input from controller
            // this is x, y (wasd) and select (enter) on the keyboard
            menuInput.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.a);
            menu.updateWithInput(menuInput);

            // display the updated menu
            for (String line : menu.toListOfStrings()) {
                telemetry.addLine(line); // but with appropriate printing method
            }
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }


        // get data from menu
        chosenAlliance = menu.getResult(Alliance.class, "alliancePicker");
        PoseStorage.chosenAlliance = menu.getResult(Alliance.class, "alliancePicker");
        AutoPath chosenPath = menu.getResult(AutoPath.class, "pathPicker");
        waitTimeMs = menu.getResult(Double.class, "waitSlider") * 1000.0;
        // set up the poseFactory and poses
        if (chosenAlliance.equals(Alliance.RED)) {
            poseFactory = poseFactory.mirrorY(70.75);
            poseFactory = poseFactory.mirrorX(70.75);
        }
        autoParkingStartPose = poseFactory.of(126.2764, 9.6619, 90);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        //this changes based on path

        switch(chosenPath) {
            case GARDEN_PATH:
                initializeGardenPath(poses1, poseFactory);
                follower.setPose(poses1[0]);
                //follower.setPose(autoRoutineGarden());
                schedule(autoRoutineGarden());
                break;
            case FLOWER_PATH:
                initializeFlowerPath(poses2, poseFactory);
                follower.setPose(poses2[0]);
                schedule(autoRoutineFlower());
                break;
            case PARKING_PATH:
                follower.setPose(autoParkingStartPose);
                schedule(autoParkingPath());
                break;

            default:
                schedule(autoParkingPath());
                telemetry.addLine("How did you do this? You have to choose a path!");
        }
        follower.update();
        waitForStart();


        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            //telling the values of the robot on the driver hub
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Launcher Speed: ", launchMotor.getVelocity());
            telemetry.addData("Ramp Position",flowerRamp.getPosition() == RAMP_UP ? "Up":"Down");

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        PoseStorage.endPose = follower.pose();
    }
    public Command intakeBallForMillis(long millis) {
        return Command.build()
                .setStart(() -> {
                    //something goes here
                    timer.reset();
                    intakeMotor.setPower(-INTAKE_SPEED);
                })
                .setDone(() ->
                        timer.milliseconds() > millis)
                .setEnd(endCondition -> {
                    intakeMotor.setPower(0.0);
                    // executed on end
                })
                .requiring(intakeMotor);
    }
    public Command rampUp() {
        return instant(() -> flowerRamp.setPosition(RAMP_UP));
    }
    public Command rampDown() {
        return instant(() -> flowerRamp.setPosition(RAMP_DOWN));
    }
    /*    public Command shootBalls(int numBalls) {
            return sequential(
                    instant(() -> gate.setPosition(GATE_OPEN)),
                    instant(()->intakeMotor.setVelocity(-INTAKE_SPEED)),
                    instant(() -> launchMotor.setVelocity(LAUNCHER_SPEED)),
                    //instant(()->intakeMotor.setVelocity(-INTAKE_SPEED)),
                    waitMs(numBalls*LAUNCH_TIME),
                    instant(()->intakeMotor.setVelocity(0)),
                    instant(() -> launchMotor.setVelocity(0)),
                    instant(() -> gate.setPosition(GATE_CLOSE))
            );
        }*/
    public Command shootBalls(int numBalls) {
        return sequential(
                instant(() -> launchMotor.setVelocity(LAUNCHER_SPEED)),
                waitMs(8000), // tune this
                instant(() -> gate.setPosition(GATE_OPEN)),
                instant(() -> intakeMotor.setPower(0.5*INTAKE_SPEED)),
                waitMs(numBalls * LAUNCH_TIME),
                instant(() -> intakeMotor.setPower(0)),
                instant(() -> launchMotor.setVelocity(0)),//longer wait time for launch
                ///
                instant(() -> gate.setPosition(GATE_CLOSE))
        );
    }
    public Command intake(long millis) {
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    intakeMotor.setPower(-INTAKE_SPEED);
                })
                .setDone(() -> timer.milliseconds() >= millis)
                .setEnd(endCondition -> {
                    intakeMotor.setPower(0);
                })
                .requiring(intakeMotor);
    }



    public Command intakeOn() {
        telemetry.addLine("INTAKE SHUD BE ON");
        telemetry.update();
        return instant(() -> intakeMotor.setPower(INTAKE_SPEED));
    }

    public Command intakeOff() {
        return instant(() -> intakeMotor.setPower(0));
    }
    public Command autoRoutineGarden() {
        return sequential(
                waitMs(waitTimeMs),
                follow(follower, Paths.line(poses1[0], shootPose).constant(shootPose)),

                shootBalls(4),

                parallel(follow(follower, Paths.line(shootPose, poses1[1]).constant(poses1[1])),
                        rampDown()),

                // drive into the balls with intake running
                intakeOn(),
                follow(follower, Paths.line(poses1[1], poses1[2]).constant(poses1[2])),
                waitMs(700), // time to pick up balls, tune this
                follow(follower, Paths.line(poses1[2], poses1[1]).constant(poses1[2])),

                follow(follower, Paths.line(poses1[1], poses1[2]).constant(poses1[2])),
                waitMs(700),
                intakeOff(),

                follow(follower, Paths.line(poses1[2], poses1[3]).constant(poses1[3])),
                rampUp(),

                follow(follower, Paths.line(poses1[3], poses1[8]).constant(poses1[3]))
                /*
                follow(follower, Paths.line(poses1[3], poses1[4]).constant(poses1[4])),
                follow(follower, Paths.line(poses1[4], poses1[5]).constant(poses1[5])),
                follow(follower, Paths.line(poses1[5], poses1[6]).constant(poses1[6])),

                //shootBalls(4),

                follow(follower, Paths.line(poses1[6], poses1[7]).constant(poses1[7])),
                follow(follower, Paths.line(poses1[7], poses1[8]).constant(poses1[8]))

                 */
        );
    }
    /*
        public Command autoRoutineGarden() {
            return sequential(
                    waitMs(waitTimeMs),

                    shootBalls(4),
                    parallel(
                            follow(follower, Paths.line(poses1[0], poses1[1]).constant(poses1[0])),
                            rampDown()
                    ),
                    intake(5000);
                    follow(follower, Paths.line(poses1[1], poses1[2]).constant(poses1[1])),
                    intakeBallForMillis(5000),
                    follow(follower, Paths.line(poses1[2], poses1[3]).constant(poses1[2])),
                            intake(5000),

            rampUp(),
                    follow(follower, Paths.line(poses1[3], poses1[4]).constant(poses1[3])),
                    follow(follower, Paths.line(poses1[4], poses1[5]).constant(poses1[4])),

                    follow(follower, Paths.line(poses1[5], poses1[6]).constant(poses1[5])),
                    shootBalls(4).raceWith(hold(follower)),
                    follow(follower, Paths.line(poses1[6], poses1[7]).constant(poses1[6])),
                    follow(follower, Paths.line(poses1[7], poses1[8]).constant(poses1[7]))
            );
        }

     */
    public void initializeGardenPath(Pose[] gardenPoses, PoseFactory poseFactory) { //BRUTE FORCE MIRROR
        gardenPoses[0] = poseFactory.of(84.1293, 132.8942, 90);
        if (chosenAlliance.equals(Alliance.RED)) {
            gardenPoses[1] = poseFactory.of(119.598, 92.8211, 180); //this one
            gardenPoses[2] = poseFactory.of(128, 93.4615, 180); // this one
            gardenPoses[3] = poseFactory.of(119.9248, 91.6962, 180);
        } else {
            gardenPoses[1] = poseFactory.of(119.598, 92.8211, 0); //this one
            gardenPoses[2] = poseFactory.of(128, 93.4615, 0); // this one
            gardenPoses[3] = poseFactory.of(119.9248, 91.6962, 0);
        }
        gardenPoses[4] = poseFactory.of(105.4226, 30.1768, -90);
        gardenPoses[5] = poseFactory.of(76.4427, 26.1793, -90);
        gardenPoses[6] = poseFactory.of(83.7372, 8.6485, -90);
        gardenPoses[7] = poseFactory.of(83.3717, 29.9152, -90);
        gardenPoses[8] = poseFactory.of(129.5481, 46.3087, -90);

        // slightly forward from the start pose, flip the sign on y if it moves the wrong way
        shootPose = poseFactory.of(84.1293, 130.8942, 90);
    }
    public Command autoParkingPath() {
        Pose end = poseFactory.of(126.76431357656855, 24.369302898095505, 90);
        return sequential(
                waitMs(waitTimeMs),
                follow(follower,
                        Paths.line(autoParkingStartPose, end).constant(end))
        );

    }
    public Command autoRoutineFlower() {
        return sequential(
                waitMs(waitTimeMs),
                follow(follower,Paths.line(poses2[0], poses2[1]).constant(poses2[1])),
                rampDown(),
                shootBalls(4).raceWith(hold(follower)),
                follow(follower,Paths.line(poses2[1], poses2[2]).constant(poses2[2])),
                follow(follower, Paths.line(poses2[2], poses2[3]).constant(poses2[3])),
                intakeBallForMillis(5000).raceWith(hold(follower)),
                follow(follower, Paths.line(poses2[3], poses2[4]).constant(poses2[4])),
                follow(follower, Paths.line(poses2[4], poses2[5]).constant(poses2[5])).with(rampUp()),
                shootBalls(4).raceWith(hold(follower)),
                follow(follower, Paths.line(poses2[5], poses2[6]).constant(poses2[6]))
        );
    }
    public void initializeFlowerPath(Pose[] flowerPoses, PoseFactory poseFactory) {
        flowerPoses[0] = poseFactory.of(79.1085, 8.6331, -90);
        flowerPoses[1] = poseFactory.of(84.7, 23.3459, -90);
        flowerPoses[2] = poseFactory.of(94.4916, 17.3314, -90);
        flowerPoses[3] = poseFactory.of(94.0503, 9.5, -90);
        flowerPoses[4] = poseFactory.of(94.4916, 17.3314, -90);
        flowerPoses[5] = poseFactory.of(84.7, 23.3459, -90);
        flowerPoses[6] = poseFactory.of(129.3126, 24.6121, -90); //always face the back
    }
}