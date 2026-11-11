package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

abstract public class TuneLauncherPIDF extends LinearOpMode {
    protected Follower follower;
    protected DcMotorEx intakeMoto;
    protected DcMotorEx launchMoto;
    public static double LAUNCHERspeed = 1360;//1250
    public static double INTAKEspeed = 1.0;

    protected void initializeDrive() {
        follower = Constants.create(hardwareMap);

        launchMoto = hardwareMap.get(DcMotorEx.class, "motLaunch");

        launchMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMoto = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMoto.setMotorEnable();
        launchMoto.setMotorEnable();

    }

    protected double slowModeScale() {
        if (gamepad1.right_trigger != 0) {
            return ((-gamepad1.right_trigger + 1) / 2) + 0.5;
        }
        return 1;
    }
}