package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends BaseOpMode{
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        initializeHardware();
        follower = Constants.create(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intakeMot.setVelocity(gamepad2.left_stick_y * INTAKE_SPEED_MULTIPLIER);

            if (gamepad2.aWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);

            }
            if (gamepad2.yWasPressed()) {
                lowerFlywheelMot.setVelocity(LAUNCHER_SPEED);
                upperFlywheelMot.setVelocity(LAUNCHER_SPEED - LAUNCHER_BACKSPIN);

            }

            follower.manual(forward, lateral, turn);
            follower.update();
        }
    }
    public double doSLOWMODE() {
        if (gamepad1.right_trigger != 0) {
            return ((-gamepad1.right_trigger + 1)/2) + 0.5;
        } else {
            return 1;
        }
    }
}


