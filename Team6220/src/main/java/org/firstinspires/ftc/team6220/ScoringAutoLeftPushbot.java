package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ScoringAutoLeftPushbot", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class ScoringAutoLeftPushbot extends ScoringAutoMiddlePushbot{

    @Override
    protected Pose2d getInitializationPose() {
        return Constants.LEFT_STARTING_POSE;
    }
}
