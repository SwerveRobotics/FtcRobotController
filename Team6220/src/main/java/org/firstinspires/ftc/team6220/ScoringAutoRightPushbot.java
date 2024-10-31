package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ScoringAutoRightPushbot", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class ScoringAutoRightPushbot extends ScoringAutoMiddlePushbot{

    @Override
    protected Pose2d getInitializationPose() {
        return Constants.RIGHT_STARTING_POSE;
    }
}
