package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseTeleOp;

@TeleOp(name = "Competition TeleOp")
public class CompetitionTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        runTeleOp(Config.useDriveTo, Config.useAprilTags, Config.pose, Config.armPosition);
    }
}