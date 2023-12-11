package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous (name = "AngleTest")

public class AngleTest extends BaseAutonomous {
    public void runOpMode() {
        initAuto();
        waitForStart();
        double orientation;
        double currentAngle;
        double currentAngle2;
        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentAngle2 = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("current angle 2", currentAngle2);
            telemetry.update();
            runIntake(-0.5, 1);
        }

    }
}