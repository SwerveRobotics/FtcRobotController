package org.firstinspires.ftc.team417_CENTERSTAGE.concepts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseTeleOp;

public class TrussDAConcept extends BaseTeleOp {
    public void runOpMode() {
        initializeHardware();

        while (opModeIsActive()) {
            driveUsingControllers();
            drive.doActionsWork();
        }
    }

    public Action buildOuterTrussAction(double x, double y, double theta) {
        Action action = drive.actionBuilder(new Pose2d(x, y, theta))
                .splineTo(new Vector2d(x, y), Math.toRadians(270.00))
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.00, -60.00), Math.toRadians(1.41))
                .splineTo(new Vector2d(48.00, -36.00), Math.toRadians(0.00))
                .build();
        return action;
    }
}
