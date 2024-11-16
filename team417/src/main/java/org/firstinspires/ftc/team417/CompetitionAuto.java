package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.programs.BaseOpMode;
import org.firstinspires.ftc.team417.programs.BaseOpModeFastBot;
import org.firstinspires.ftc.team417.programs.CompetitionBasketAutoFastBot;
import org.firstinspires.ftc.team417.programs.CompetitionSpecimenAutoFastBot;
import org.firstinspires.ftc.team417.programs.CompetitionSpecimenAutoSlowBot;
import org.firstinspires.ftc.team417.programs.ReliableAuto;

@Autonomous(name = "Auto")
public class CompetitionAuto extends BaseOpModeFastBot {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        runMenu();

        getResultsFromMenu();

        BaseOpMode program = chooseProgram();

        waitForStart();

        Thread.sleep((long) (Config.waitTime * 1000));

        program.runOpMode();
    }

    public BaseOpMode chooseProgram() {
        if (Config.useReliableAuto) {
            return new ReliableAuto();
        } else {
            switch (Config.robot) {
                case FAST_BOT:
                    switch (Config.location) {
                        case NET:
                            return new CompetitionSpecimenAutoFastBot();
                        case OBSERVATION:
                            return new CompetitionBasketAutoFastBot();
                    }
                case SLOW_BOT:
                    switch (Config.location) {
                        case NET:
                            throw new IllegalArgumentException("The Slow Bot doesn't have a net program yet.");
                        case OBSERVATION:
                            return new CompetitionSpecimenAutoSlowBot();
                    }
            }
        }
        throw new IllegalArgumentException("Your values aren't supported as a program.");
    }
}
