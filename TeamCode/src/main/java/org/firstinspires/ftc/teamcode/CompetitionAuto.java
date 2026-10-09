package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;


import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


import static com.pedropathing.api.Paths.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoPath", group = "Autonomous")

public class CompetitionAuto extends BaseOpMode{
    private Follower follower;
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose leaveAndParkStart = poseFactory.of(132.832, 59.8271, 90);
    private Pose leaveAndParkStart_2 = poseFactory.of(132.832, 59.8271, 180);
    private Pose leaveAndPark = poseFactory.of(127, 32, 180);
    private Pose leaveAndParkControl1 = poseFactory.of(96, 50, 180);

    //For launch and park path
    //private final Pose start = poseFactory.of(84, 134, 90);
    private Pose launchAndParkStart_2 = poseFactory.of(84, 134, 270);
    private Pose launchAndPark = poseFactory.of(127, 32, 180);
    private Pose launchAndParkControl1 = poseFactory.of(125, 132, 0);

    private Pose hDLaunchandParkStart = poseFactory.of(80, 9, 90);
    private Pose hDLaunchandPark = poseFactory.of(127, 32, 180);
    private Pose hDLaunchandParkControl1 = poseFactory.of(75, 20, 0);

    public Command startIntake = instant(() -> intakeMot.setPower(1.0) );
    public Command stopIntake = instant(() -> intakeMot.setPower(0.0) );
    public Command aRleaveAndPark() {
        return sequential(
            follow(follower, leaveAndPark())

        );
    }
    public Command aRlaunchAndPark() {
        return sequential(
                launchFourBalls(),
                follow(follower, launchAndPark())

        );
    }
    public Command aRHDlaunchAndPark() {
        return sequential(
                waitMs(PARTNER_LAUNCH_TIME),
                launchFourBalls(),
                follow(follower, hDLaunchandPark())

        );
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // this needs to be passed in via text menu so we can alternate between blue and red
        boolean redAlliance = true;
        if(redAlliance) {
            poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
            leaveAndParkStart = poseFactory.of(132.832, 59.8271, 90);
            leaveAndParkStart_2 = poseFactory.of(132.832, 59.8271, 180);
            leaveAndPark = poseFactory.of(127, 32, 180);
            leaveAndParkControl1 = poseFactory.of(96, 50, 180);

            //For launch and park path
            //private final Pose start = poseFactory.of(84, 134, 90);
            launchAndParkStart_2 = poseFactory.of(84, 134, 270);
            launchAndPark = poseFactory.of(127, 32, 180);
            launchAndParkControl1 = poseFactory.of(125, 132, 0);

            hDLaunchandParkStart = poseFactory.of(80, 9, 90);
            hDLaunchandPark = poseFactory.of(127, 32, 180);
            hDLaunchandParkControl1 = poseFactory.of(75, 20, 0);
        } else {
            poseFactory = PoseFactory.degrees();
        }
       ///Scheduler.reset();
//        follower = Constants.create(hardwareMap);
//        follower.setPose(leaveAndParkStart_2);
//        follower.update();

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(launchAndParkStart_2);
        follower.update();


        waitForStart();
        schedule(aRlaunchAndPark());

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

    public Path leaveAndPark() {
        return curve(leaveAndParkStart_2, leaveAndParkControl1, leaveAndPark).linear(leaveAndParkStart_2, leaveAndPark);
    }

    public Path launchAndPark() {
        return curve(launchAndParkStart_2, launchAndParkControl1, launchAndPark).linear(launchAndParkStart_2, launchAndPark);
    }
    public Path hDLaunchandPark() {
        return curve(hDLaunchandParkStart, hDLaunchandParkControl1, hDLaunchandPark).linear(hDLaunchandParkStart, hDLaunchandPark);
    }

    // Mechanism movement commands (intake and launching)
    public Command runIntakeForMS(long milliseconds) {
        return sequential(
                // Turn on intake
                instant(() -> intakeMot.setPower(1.0)),

                // Run intake for the time in seconds
                waitMs(milliseconds),

                // Turn intake off
                instant(() -> intakeMot.setPower(0.0))
        );
    }

    public Command launchFourBalls() {
        return sequential(
                // Turn on transfer wheel
                instant(() -> transferWheelMot.setVelocity(TRANSFER_SPEED)),

                // Wait for balls to get to launcher
                waitMs(BALLS_TO_LAUNCH_TIME), //TODO: need to decide a constant value

                // Turn on launcher flywheels
                instant(() -> lowerFlywheelMot.setVelocity(LAUNCHER_SPEED)),
                instant(() -> upperFlywheelMot.setVelocity(LAUNCHER_SPEED - LAUNCHER_BACKSPIN)),

                // Wait for four balls to launch7
                waitMs(FOUR_BALLS_TIME), //TODO: need a constant for this too to tune later

                // Turn launcher off
                instant(() -> transferWheelMot.setPower(0.0)),
                instant(() -> lowerFlywheelMot.setPower(0.0)),
                instant(() -> upperFlywheelMot.setPower(0.0))
        );
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