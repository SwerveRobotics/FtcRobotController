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

import org.firstinspires.ftc.teamcode.menu.MenuFinishedButton;
import org.firstinspires.ftc.teamcode.menu.MenuHeader;
import org.firstinspires.ftc.teamcode.menu.MenuInput;
import org.firstinspires.ftc.teamcode.menu.MenuSlider;
import org.firstinspires.ftc.teamcode.menu.TextMenu;


@Autonomous(name = "417 Auto")
public class CompetitionAuto extends BaseOpMode{
    //when you select the class
    private Follower follower;
    //the values of the paths
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose[] poses1 = new Pose[9]; //this holds the poses for the garden path
    private Pose[] poses2 = new Pose[7]; //this holds the poses for the flower path
    private Pose autoParkingStartPose;


    private final ElapsedTime timer = new ElapsedTime();
    public enum Alliance {
            RED,
            BLUE,
    }
    public enum AutoPath { //different options for auto routine
        GARDEN_PATH,
        FLOWER_PATH,
        PARKING_PATH
    }
    public double waitTimeMs; //how long we wait at the start
    //what happens when you press the start button on the driver hub

    //create paths for GARDEN_PATH, telling the paths to go in order, intaking and shooting
    public Command autoRoutineGarden() {
        return sequential(
                waitMs(waitTimeMs),
                follow(follower, Paths.line(poses1[0], poses1[1]).constant(poses1[1])),
                shootBalls(4).raceWith(hold(follower)),
                follow(follower, Paths.line(poses1[2], poses1[3]).linear(poses1[2], poses1[3])),
                follow(follower, Paths.line(poses1[3], poses1[4]).constant(poses1[4])).raceWith(intakeBallForMillis(3000)),
                follow(follower, Paths.line(poses1[4], poses1[5]).tangent()),
                follow(follower, Paths.line(poses1[6], poses1[7]).linear(poses1[6], poses1[7])),
                shootBalls(4).raceWith(hold(follower)),
                follow(follower, Paths.line(poses1[7], poses1[8]).tangent())
        );
    }
    public Command autoParkingPath() {
        Pose end = poseFactory.of(127.07736667391369, 30.31731174765303, 90);
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

    @Override
    public void runOpMode() throws InterruptedException {
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
        while (!menu.isCompleted()) {
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


        // get data from menu
        Alliance chosenAlliance = menu.getResult(Alliance.class, "alliancePicker");
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

            default:
                schedule(autoRoutineGarden());
                telemetry.addData("How did you do this? You have to choose a path!", 67);
        }
        follower.update();
        initializeHardware();
        waitForStart();


        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            //telling the values of the robot on the driver hub
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
    public Command intakeBallForMillis(long millis) {
        return Command.build()
                .setStart(() -> {
                    //something goes here
                    timer.reset();
                    intakeMotor.setPower(1.0);
                    intakeMotor.setVelocity(INTAKE_SPEED);
                })
                .setDone(() ->
                        timer.milliseconds() > millis)
                .setEnd(endCondition -> {
                    intakeMotor.setVelocity(0.0);
                    intakeMotor.setPower(0.0);
                    // executed on end
                })
                .requiring(intakeMotor);
    }
    public Command rampUp() {
        return Command.build().setStart(() -> flowerRamp.setPosition(RAMP_UP));
    }
    public Command rampDown() {
        return Command.build().setStart(() -> flowerRamp.setPosition(RAMP_DOWN));
    }
    public Command shootBalls(int numBalls) {
        Command revUp = Command.build()
                .setStart(() -> {launchMotor.setVelocity(LAUNCHER_SPEED);
                gate.setPosition(GATE_OPEN);
                })
                .setDone(() -> launchMotor.getVelocity()  > LAUNCHER_SPEED * 0.95)//let it rev up to nearly the full launch speed
                .requiring(launchMotor, gate);
        Command actuallyShoot = repeat(
                Command.build()
                .setStart(() -> {
                    telemetry.addData("Waiting for launcher to get up to speed", launchMotor.getVelocity());
                    waitUntil(()->launchMotor.getVelocity() > LAUNCHER_SPEED * 0.95);
                })
                .setExecute(()-> {
                    intakeMotor.setVelocity(INTAKE_SPEED);
                    telemetry.addData("Launching with speed ", launchMotor.getVelocity());
                })
                .setDone(() -> launchMotor.getVelocity() < LAUNCHER_SPEED * 0.95)
                        .requiring(launchMotor, intakeMotor)
                ,numBalls);
        Command end = Command.build()
                .setStart(() -> {
                    launchMotor.setVelocity(0);
                    gate.setPosition(GATE_CLOSE);
                })
                .requiring(launchMotor, gate);
        return sequential(revUp, actuallyShoot, end);
    }
    public void initializeFlowerPath(Pose[] flowerPoses, PoseFactory poseFactory) {
        flowerPoses[0] = poseFactory.of(79.1085, 8.6331, 90);
        flowerPoses[1] = poseFactory.of(84.7, 23.3459, 90);
        flowerPoses[2] = poseFactory.of(94.4916, 17.3314, -90);
        flowerPoses[3] = poseFactory.of(94.0503, 9.5, -90);
        flowerPoses[4] = poseFactory.of(94.4916, 17.3314, -90);
        flowerPoses[5] = poseFactory.of(84.7, 23.3459, 90);
        flowerPoses[6] = poseFactory.of(129.3126, 24.6121, 90);
    }
    public void initializeGardenPath(Pose[] gardenPoses, PoseFactory poseFactory) {
        gardenPoses[0] = poseFactory.of(85.1, 131.6, 90);
        gardenPoses[1] = poseFactory.of(86.2461, 125.7, -90);
        gardenPoses[2] = poseFactory.of(86.2461, 125.7, 0);
        gardenPoses[3] = poseFactory.of(106.9552, 131, 0);
        gardenPoses[4] = poseFactory.of(129.6295, 131, 0);
        gardenPoses[5] = poseFactory.of(105.3705, 33.0811, -103.9147);
        gardenPoses[6] = poseFactory.of(105.3705, 33.0811, -136);
        gardenPoses[7] = poseFactory.of(85.1562, 16.4153, 90);
        gardenPoses[8] = poseFactory.of(125.0884, 34.0981, 23.8848);
    }
}
