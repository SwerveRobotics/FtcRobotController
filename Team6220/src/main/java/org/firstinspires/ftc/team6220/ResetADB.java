package org.firstinspires.ftc.team6220;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name = "Reset ADB", group = "Utility")
public class ResetADB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Press ▶ to reset the ADB connection.");
        telemetry.update();
        waitForStart();

        try {
            // Restart the ADB daemon:
            Runtime.getRuntime().exec("setprop ctl.restart adbd");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

