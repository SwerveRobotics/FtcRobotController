package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "TestIntakeAuto")

public class TestIntakeAuto extends BaseAutonomous {
    public void runOpMode() {
        initHardware();
        waitForStart();
        runIntake(0.5, 2000);
    }
}
