package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;


import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static com.pedropathing.api.Paths.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoPath", group = "Autonomous")

public class CompetitionAuto extends BaseOpMode{
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose path1Move1Start = poseFactory.of(84, 134, 270);
    private final Pose path1Move2Start = poseFactory.of(134, 134, 0);
    private final Pose path1Move2End = poseFactory.of(83, 20, 90);
    private final Pose path1Move2Control = poseFactory.of(102, 5, 0);


    public Command autoRoutine() {
        return sequential(
                follow(follower, path1Move1()),
                follow(follower, path1Move2())
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(path1Move1Start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

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

    public Path path1Move1() {
        return line(path1Move1Start, path1Move2Start).linear(path1Move1Start, path1Move2Start);
    }

    public Path path1Move2() {
        return curve(path1Move2Start, path1Move2Control, path1Move2End).linear(path1Move2Start, path1Move2End);
    }

}
/*
package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(134, 134, 90);
    private final Pose move1Start = poseFactory.of(134, 134, 270);
    private final Pose move1 = poseFactory.of(134, 134, 0);
    private final Pose point2 = poseFactory.of(83, 20, 90);
    private final Pose point2Control1 = poseFactory.of(102, 5, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, move1()),
            follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

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

    public Path move1() {
        return line(move1Start, move1).linear(move1Start, move1);
    }

    public Path path2() {
        return curve(move1, point2Control1, point2).linear(move1, point2);
    }
}

 */