package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Teleop without the intake and drum logic so we can just test for launcher speeds
@TeleOp (name = "LauncherTest", group = "Competition")
public class LauncherTest extends CompetitionTeleOp {
    Servo servoTransfer;
    DcMotorEx motLLauncher;
    DcMotorEx motULauncher;
    CRServo servoBLaunchFeeder;
    CRServo servoFLaunchFeeder;
    ElapsedTime transferTimer;
    TransferState transferState;


    enum TransferState {
        WAIT,
        DONE
    }
    public void initHardware() {
        // Initializing only the transfer, feeder, and launcher motors/servos
        servoTransfer = hardwareMap.get(Servo.class, "servoTransfer");
        motLLauncher = hardwareMap.get(DcMotorEx.class, "motLLauncher");
        motULauncher = hardwareMap.get(DcMotorEx.class, "motULauncher");
        servoBLaunchFeeder = hardwareMap.get(CRServo.class, "servoBLaunchFeeder");
        servoFLaunchFeeder = hardwareMap.get(CRServo.class, "servoFLaunchFeeder");


        servoBLaunchFeeder.setDirection(CRServo.Direction.REVERSE);

    }

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {

            // Spin up launcher flywheels to set flywheel velocities and start feeder wheels
            if (gamepad2.dpadUpWasPressed()) {
                motULauncher.setVelocity(ComplexMechGlob.FAR_FLYWHEEL_VELOCITY - (0.5 * ComplexMechGlob.FLYWHEEL_BACK_SPIN));
                motLLauncher.setVelocity(ComplexMechGlob.FAR_FLYWHEEL_VELOCITY + (0.5 * ComplexMechGlob.FLYWHEEL_BACK_SPIN));
                servoBLaunchFeeder.setPower(ComplexMechGlob.FEEDER_POWER);
                servoFLaunchFeeder.setPower(ComplexMechGlob.FEEDER_POWER);

            } else if (gamepad2.dpadDownWasPressed()) {
                motULauncher.setVelocity(ComplexMechGlob.NEAR_FLYWHEEL_VELOCITY - (0.5 * ComplexMechGlob.FLYWHEEL_BACK_SPIN));
                motLLauncher.setVelocity(ComplexMechGlob.NEAR_FLYWHEEL_VELOCITY + (0.5 * ComplexMechGlob.FLYWHEEL_BACK_SPIN));
                servoBLaunchFeeder.setPower(ComplexMechGlob.FEEDER_POWER);
                servoFLaunchFeeder.setPower(ComplexMechGlob.FEEDER_POWER);
            }


            // When y is pressed, start the transfer, run for TRANSFER_TIME_UP, then stop it
            if (gamepad2.yWasPressed()) {
                if (transferTimer == null) {
                    transferTimer = new ElapsedTime();
                }
                if (transferTimer.seconds() <= ComplexMechGlob.TRANSFER_TIME_UP) {
                    servoTransfer.setPosition(ComplexMechGlob.TRANSFER_ACTIVE_POSITION);
                    transferState = TransferState.WAIT;
                }
                if (transferTimer.seconds() >= ComplexMechGlob.TRANSFER_TIME_DOWN + ComplexMechGlob.TRANSFER_TIME_UP) {
                    servoTransfer.setPosition(ComplexMechGlob.TRANSFER_INACTIVE_POSITION);
                    transferState = TransferState.DONE;
                    transferTimer = null;
                }
            }


            // Stop all motors and feeders
            if (gamepad2.dpadRightWasPressed()) {
                servoBLaunchFeeder.setPower(0);
                servoFLaunchFeeder.setPower(0);
                motULauncher.setVelocity(0);
                motLLauncher.setVelocity(0);
            }

        }
    }
}
