package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="TuneLauncherPIDF")
public class TuneLauncherPIDF extends BaseOpMode {
    ElapsedTime timer = new ElapsedTime();
    public double maxVelocity = 0;
    public double currVelocity = 0;
    public double currAccel = 0;
    public double targetVelocity = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        currAccel = launchMotor.getVelocity() - currVelocity;
        currVelocity = launchMotor.getVelocity();

        initializeHardware();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.right_bumper) {
                launchMotor.setPower(1);
            }
            if (gamepad2.left_bumper) {
                launchMotor.setPower(0);
            }

            if (currVelocity > maxVelocity) {
                maxVelocity = currVelocity;
            }


            telemetry.addData("Velocity", currVelocity);
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("Acceleration", currAccel);
            telemetry.update();

        }
    }
}