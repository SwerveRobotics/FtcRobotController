package org.firstinspires.ftc.team417;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestColorDetect extends LinearOpMode {
    CoolColorDetector detector;
    @Override
    public void runOpMode() {
        detector = new CoolColorDetector(hardwareMap, telemetry);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        waitForStart();
        while (opModeIsActive()) {
            detector.testTelemetry();
        }
    }
}

