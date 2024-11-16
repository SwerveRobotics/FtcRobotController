package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.programs.BaseOpMode;
import org.firstinspires.ftc.team417.programs.FastBotTeleOp;
import org.firstinspires.ftc.team417.programs.SlowBotTeleOp;

@TeleOp(name = "TeleOp")
public class CompetitionTeleOp extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        if (!Config.used) {
            runMenu();

            getResultsFromMenu();
        }

        BaseOpMode program = chooseProgram();

        waitForStart();

        program.runOpMode();
    }

    public BaseOpMode chooseProgram() {
        switch (Config.robot) {
            case FAST_BOT:
                return new FastBotTeleOp();
            case SLOW_BOT:
                return new SlowBotTeleOp();
        }
        throw new IllegalArgumentException("Your values aren't supported as a program.");
    }
}

