package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417.roadrunner.KinematicType;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {
    public static final KinematicType kinematicType = KinematicType.X;
}
