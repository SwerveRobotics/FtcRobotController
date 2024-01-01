package org.firstinspires.ftc.team417_CENTERSTAGE.concepts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

public class TrussDAConcept extends BaseOpMode {
    public void runOpMode() {
        Action driveAssistTo = drive.actionBuilder(new Pose2d(-39.05, 37.47, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-39.50, 37.92), Math.toRadians(270.00))
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.00, -60.00), Math.toRadians(1.41))
                .build();

        drive.runParallel(driveAssistTo);
        
        drive.doActionsWork();
    }
}
