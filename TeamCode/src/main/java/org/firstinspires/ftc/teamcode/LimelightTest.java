package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Limelight")
public class LimelightTest extends BaseOpMode {
    @Override
    public void initializeHardware() {
        super.initializeHardware();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeLimelight();
        waitForStart();
        chosenAlliance = Alliance.BLUE;
        while (opModeIsActive()) {
            telemetry.addData("Is Hive Scorable",isHiveScorable());
            telemetry.update();
        }
    }
}