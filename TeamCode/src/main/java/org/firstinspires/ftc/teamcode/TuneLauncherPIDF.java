package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

abstract public class TuneLauncherPIDF extends LinearOpMode {
    protected Follower follower;
    protected void initializeDrive() {
        follower = Constants.create(hardwareMap);

    }

    protected double slowModeScale() {
        if (gamepad1.right_trigger != 0) {
            return ((-gamepad1.right_trigger + 1) / 2) + 0.5;
        }
        return 1;
    }
}