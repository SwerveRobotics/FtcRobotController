package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends BaseOpMode{
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        follower = Constants.create(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            intakeMot.setVelocity(gamepad2.left_stick_y * INTAKE_SPEED_MULTIPLIER);
            leftIntakeServo.setPower(gamepad2.left_stick_y);
            rightIntakeServo.setPower(gamepad2.left_stick_y);

            if (gamepad2.aWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);

            }
            if (gamepad2.yWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);
                lowerFlywheelMot.setVelocity(LAUNCHER_SPEED - LAUNCHER_TOPSPIN);
                upperFlywheelMot.setVelocity(LAUNCHER_SPEED);

            }
            if(gamepad2.bWasPressed()) {
                transferWheelMot.setVelocity(0.0);
                lowerFlywheelMot.setVelocity(0.0);
                upperFlywheelMot.setVelocity(0.0);
            }

            double slowModeMultiplier = doSLOWMODE();

            follower.manual(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    gamepad1.right_stick_x * slowModeMultiplier
            );

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


