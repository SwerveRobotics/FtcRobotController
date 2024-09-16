package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.team417.roadrunner.HolonomicDrive;
import org.firstinspires.ftc.team417.roadrunner.KinematicType;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name="Auto", group="Competition", preselectTeleOp="CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {
    public static final KinematicType kinematicType = KinematicType.MECANUM;

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        HolonomicDrive drive = new HolonomicDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);

        telemetry.addLine("Running 417's TeleOp!");
        telemetry.addLine("Kinematic type is " + drive.kinematicType);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
    }
}
