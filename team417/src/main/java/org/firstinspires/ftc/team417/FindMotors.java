package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417.programs.BaseOpModeFastBot;

/**
 * This class finds the motors. If you just built a robot, use this class to get the configuration.
 */
@TeleOp(name="Find Motors", group="Competition")
public class FindMotors extends BaseOpModeFastBot {
    DcMotor leftFront, leftBack, rightBack, rightFront, armMotor;
    CRServo intake;
    Servo wrist;

    private final boolean HAS_MECANISMS = true;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        if (HAS_MECANISMS) {
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            intake = hardwareMap.get(CRServo.class, "intake");
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            setPower(gamepad1.x, leftFront);
            setPower(gamepad1.y, rightFront);
            setPower(gamepad1.a, leftBack);
            setPower(gamepad1.b, rightBack);
            if (HAS_MECANISMS) {
                setPower(gamepad1.right_bumper, armMotor);
                setPower(gamepad1.left_bumper, wrist);
                setPower(gamepad1.dpad_down, intake);
            }
            telemetry.update();
        }
    }

    private void setPower(boolean on, DcMotor motor) {
        if (on) {
            motor.setPower(1);
            telemetry.addLine("Running " + motor);
        } else {
            motor.setPower(0);
        }
    }

    private void setPower(boolean on, Servo servo) {
        if (on) {
            servo.setPosition(1);
            telemetry.addLine("Running " + servo);
        } else {
            servo.setPosition(0);
        }
    }

    private void setPower(boolean on, CRServo crServo) {
        if (on) {
            crServo.setPower(1);
            telemetry.addLine("Running " + crServo);
        } else {
            crServo.setPower(0);
        }
    }
}
