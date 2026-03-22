package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {
    LimelightAprilTagDetector detector;

    public static double ROBOT_WIDTH = 16.15;
    public static double ROBOT_LENGTH = 16.5;

}


