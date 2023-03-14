package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;


@TeleOp(name = "SlideReportLimitSwitchTest")
public class SlidesWithLimitSwitchTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveSlidesWithControllerLimitSwitch();
            telemetry.addData("Limit switch hit", slideLimitSwitch.isPressed());
            telemetry.update();
        }
    }
}