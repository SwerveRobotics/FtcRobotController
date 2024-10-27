package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous(name = "AutoSpecimen", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionSpecimenAuto extends BaseOpMode {
    @Override

    public void runOpMode() {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2), Y_SCORE_POSE, Math.toRadians(45)), Math.toRadians(-45))
                .build();

    }


}
