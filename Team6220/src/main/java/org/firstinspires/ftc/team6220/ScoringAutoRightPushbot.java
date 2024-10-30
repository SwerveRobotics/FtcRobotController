package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;

@Autonomous(name = "ScoringAutoRightPushbot", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class ScoringAutoRightPushbot extends ScoringAutoMiddlePushbot{

    @Override
    protected Pose2d getInitializationPose() {
        return Constants.RIGHT_STARTING_POSE;
    }
}
