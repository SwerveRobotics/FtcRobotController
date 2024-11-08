package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "ScoringAutoLeftPushbot", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class ScoringAutoLeftPushbot extends ScoringAutoMiddlePushbot{

    @Override
    protected Pose2d getInitializationPose() {
        return DRIFTConstants.LEFT_STARTING_POSE;
    }
}
