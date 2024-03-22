package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueRightPark")

public class BlueRightPark extends BaseAutonomous {
    @Override
    public void runOpMode() {
        initBlueAuto();
        waitForStart();
        // go forward then strafe to backstage
        driveInches(0, 4);
        pivot(-90);
        driveInches(0, 96);
    }
}
