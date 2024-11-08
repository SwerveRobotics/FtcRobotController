package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous (name = "AutoSpecimenXDrive", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompeitionSpeicmenAutoXDrive extends BaseOpMode{
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), -90);  // sets the beginning pose relative to the robot and  cxc
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar

                .build();
    }
}
