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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.menu.MenuHeader;
import org.firstinspires.ftc.teamcode.menu.MenuInput;
import org.firstinspires.ftc.teamcode.menu.TextMenu;


@Autonomous(name = "417 Auto")
public class CompetitionAuto extends BaseOpMode{
    //when you select the class
    private Follower follower;
    //the values of the paths
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose[] poses1 = new Pose[9]; //this holds the poses for the garden path
    //create paths

    private final ElapsedTime timer = new ElapsedTime();
    public enum Alliance {
            RED,
            BLUE,
    }
    //what happens when you press the start button on the driver hub

    //create paths, telling the paths to go in order
    public Command autoRoutineGarden() {
        return sequential(
                follow(follower, Paths.line(poses1[0], poses1[1]).constant(poses1[1])),
                shootBalls(4),
                follow(follower, Paths.line(poses1[2], poses1[3]).linear(poses1[2], poses1[3])),
                follow(follower, Paths.line(poses1[3], poses1[4]).constant(poses1[4])).raceWith(intakeBallForMillis(3000)),
                follow(follower, Paths.line(poses1[4], poses1[5]).tangent()),
                follow(follower, Paths.line(poses1[6], poses1[7]).linear(poses1[6], poses1[7])),
                shootBalls(4),
                follow(follower, Paths.line(poses1[7], poses1[8]).tangent())
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
                .add();
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


        // the first parameter is the type to return as
        Alliance chosenAlliance = menu.getResult(Alliance.class, "alliancePicker");
        // set up the poseFactory and poses
        if (chosenAlliance.equals(Alliance.RED)) {
            poseFactory = poseFactory.mirrorY(70.75);
            poseFactory = poseFactory.mirrorX(70.75);
        }
        initializeGardenPath(poses1, poseFactory);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(poses1[0]); //this changes based on path
        follower.update();

        initializeHardware();
        waitForStart();
        schedule(autoRoutineGarden());

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
        Command intakeBall = Command.build()
                .setStart(() -> {
                    //something goes here
                    timer.reset();
                    intakeMotor.setPower(1);
                })
                .setDone(() ->
                        timer.milliseconds() > millis)
                //check if ball went in
                .setEnd(endCondition -> {
                    intakeMotor.setPower(0);
                    // executed on end
                })
                .requiring(intakeMotor);
        return  intakeBall;
    }
    public Command shootBalls(int numBalls) { // WORK IN PROGRESS
        Command shoot = Command.build()
                .setStart(() -> {launchMotor.setPower(LAUNCHER_SPEED);
                gate.setPosition(GATE_OPEN);
                    })
                .setDone(() -> timer.milliseconds() > LAUNCH_TIME * numBalls)
                .setEnd(endCondition ->
                    { launchMotor.setPower(0.0);
                        gate.setPosition(GATE_CLOSE);
                })
                .requiring(launchMotor);

        return shoot;
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
