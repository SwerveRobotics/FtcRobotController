/**
 * This is a robot application to reset the Android Debug Bridge connection so that ADB can
 * reconnect after a Wi-Fi connection is dropped.
 */
package org.firstinspires.ftc.teamMentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name = "Reset ADB", group = "Utility")
public class ResetAdb extends LinearOpMode {
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
