package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * This class finds the motors. If you just built a robot, use this class to get the configuration.
 */
@TeleOp(name="Find Motors", group="Competition")
public class FindMotors extends BaseOpMode {
    DcMotor leftFront, leftBack, rightBack, rightFront;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            setPower(gamepad1.x, leftFront);
            setPower(gamepad1.y, rightFront);
            setPower(gamepad1.a, leftBack);
            setPower(gamepad1.b, rightBack);
            telemetry.update();
        }
    }

    private void setPower(boolean mode, DcMotor motor) {
        if (mode) {
            motor.setPower(1);
            telemetry.addLine("Running " + motor.getDeviceName());
        } else {
            motor.setPower(0);
        }
    }
}
