package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedRightPark")

public class RedRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initRedAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(0, 2);
        driveInches(61, 0);
    }
}
