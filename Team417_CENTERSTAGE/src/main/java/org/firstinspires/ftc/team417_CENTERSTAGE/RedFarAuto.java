package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "Red Far")
public class RedFarAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {

        initializeAuto();

        waitForStart();

        detectingBlue = false;

        switch (detectTeamProp()) {
            case LEFT:
                telemetry.addData("Side", "Left");
                break;
            case CENTER:
                telemetry.addData("Side", "Center");
                break;
            case RIGHT:
                telemetry.addData("Side", "Right");
                break;
            default:
                telemetry.addData("Side", "Unsure");
        }
        telemetry.update();

        Pose2d startPose = new Pose2d(-36.00, -60.00, Math.toRadians(90.00));
        TrajectorySequence goToParkingRedFar = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-36.00, -37.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(-36.00, -60.00), Math.toRadians(105.00))
                .splineTo(new Vector2d(-58.56, -35.09), Math.toRadians(90.00))
                .splineTo(new Vector2d(-48.00, -12.00), Math.toRadians(360.00))
                .splineTo(new Vector2d(60.00, -12.00), Math.toRadians(360.00))
                .build();

        drive.followTrajectorySequence(goToParkingRedFar);
    }
}