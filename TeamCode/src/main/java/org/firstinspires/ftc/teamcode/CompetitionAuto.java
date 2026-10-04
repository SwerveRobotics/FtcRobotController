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


@Autonomous(name = "417 Auto")
public class CompetitionAuto extends BaseOpMode{
    //when you select the class
    private Follower follower;
    //the values of the paths
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(85.1, 131.6, 90);
    private final Pose path1 = poseFactory.of(86.2461, 125.7, -90);
    private final Pose point2Start = poseFactory.of(86.2461, 125.7, 0);
    private final Pose point2 = poseFactory.of(106.9552, 131, 0);
    private final Pose point3 = poseFactory.of(129.6295, 131, 0);
    private final Pose point4 = poseFactory.of(105.3705, 33.0811, -103.9147);
    private final Pose point5Start = poseFactory.of(105.3705, 33.0811, -136);
    private final Pose point5 = poseFactory.of(85.1562, 16.4153, 90);
    private final Pose point6 = poseFactory.of(125.0884, 34.0981, 23.8848);
    //create paths
    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }
    private final ElapsedTime timer = new ElapsedTime();
    public enum Alliance {
            RED,
            BLUE,
    }
    //what happens when you press the start button on the driver hub

    //telling the paths to go in order
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                shootBalls(4),
                follow(follower, path2()),
                follow(follower, path3()).raceWith(intakeBallForMillis(1000)),
                follow(follower, path4()),
                follow(follower, path5()),
                shootBalls(4),
                follow(follower, path6())
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
        //pick up here
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

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
}
