package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {
    protected DcMotorEx intakeMot;

    void initializeHardware(){
        intakeMot = hardwareMap.get(DcMotorEx.class, "motIntake");

    }
}
