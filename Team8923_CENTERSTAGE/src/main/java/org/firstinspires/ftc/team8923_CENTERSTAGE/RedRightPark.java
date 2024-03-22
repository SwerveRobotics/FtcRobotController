package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedRightPark")

public class RedRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initRedAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(0, 4);
        pivot(90);
        driveInches(0, 48);
    }
}
