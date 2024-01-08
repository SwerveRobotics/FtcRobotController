package org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseAutonomous;

@Autonomous(name = "Competition Autonomous")

public class CompetitionAuto extends BaseAutonomous {
    @Override
    public void runOpMode() {
        Config.menu(telemetry, gamepad1);
        runAuto(Config.isRed, Config.isClose, Config.useOpenCV, Config.useAprilTags);
    }
}
