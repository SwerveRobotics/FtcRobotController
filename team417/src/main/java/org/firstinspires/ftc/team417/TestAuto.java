package org.firstinspires.ftc.team417;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Test", group = "Auto")
@Config
public class TestAuto extends CompetitionAuto{
    public static double LAUNCH_SPEED = 835;
    public static double DRIVE_SPEED = 5;
    public static double INTAKE_SPEED = 1;
    public static double INTAKE_BACK_TIME = 0.1;
    public static double DRUM_VOLTAGE_EPS = 0.03;

    @Override
    public void runOpMode() {
        ComplexMechGlob.NEAR_FLYWHEEL_VELOCITY = LAUNCH_SPEED;
        CompetitionAuto.ROBOT_SPEED = DRIVE_SPEED;
        CompetitionAuto.INTAKE_SPEED = INTAKE_SPEED;
        ComplexMechGlob.VOLTAGE_TOLERANCE = DRUM_VOLTAGE_EPS;
        ComplexMechGlob.INTAKE_BACK_TIME = INTAKE_BACK_TIME;

        //super.runOpMode();
        processAuto(Alliance.RED, SlowBotMovement.NEAR, 0,2);
        // stops auto from stopping so we can see the spew
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}
