package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public abstract class DRIFTConstants {
    public static double ARM_ELBOW_SERVO_POSITION_OVER_BAR = 0.55;
    public static double ARM_ELBOW_SERVO_POSITION_GROUND = 0.7;
    public static double ARM_ELBOW_SERVO_POSITION_TRANSFER = 0.45;
    public static int ARM_BASE_MOTOR_POSITION_GROUND = -840;
    public static int ARM_BASE_MOTOR_POSITION_INIT = 0;
    public static int ARM_BASE_MOTOR_POSITION_OUT = -400;
    public static int ARM_BASE_MOTOR_POSITION_TRANSFER = -270;
    public static int ARM_BASE_MOTOR_POSITION_OVER_BAR = -1450; //changed on 11/15, was -1500

    public static int SLIDES_MOTOR_GROUND_POSITION = 0;
    //public static int SLIDES_MOTOR_LOW_BASKET_POSITION = 3700;
    public static int SLIDES_MOTOR_LOW_BASKET_POSITION = 2000;
    public static int SLIDES_MOTOR_HIGH_BASKET_POSITION = 3850;
    public static int SLIDES_MOTOR_OVER_HIGH_CHAMBER_POSITION = 2500;
    public static int SLIDES_MOTOR_HIGH_CHAMBER_POSITION = 1500;

    public static double DUMPER_SERVO_POSITION_DUMP = 0.9;
    public static double DUMPER_SERVO_POSITION_TRANSFER = 0.40;
    public static double DUMPER_SERVO_POSITION_HORIZONTAL = 0.5;
    public static double DUMPER_SERVO_POSITION_INIT = 0.2;
    public static final float[] RED_COLOR_SENSOR_RANGES = {0.025f, 1f};
    public static final float[] BLUE_COLOR_SENSOR_RANGES = {0.025f, 1f};
    public static final float COLOR_SENSOR_GAIN = 3.8f;
    //public static final Pose2d BASKET_SCORING_POSE = new Pose2d()
    public static final double ROBOT_LENGTH_IN = 16.50;
    public static final Pose2d MIDDLE_STARTING_POSE = new Pose2d(0, 72-(ROBOT_LENGTH_IN/2), (3 * Math.PI)/2);
    public static final Pose2d LEFT_STARTING_POSE = new Pose2d(24, 72-(ROBOT_LENGTH_IN/2), (3 * Math.PI)/2);
    public static final Pose2d RIGHT_STARTING_POSE = new Pose2d(-20, 72-(ROBOT_LENGTH_IN/2), (3 * Math.PI)/2);
    public static final Pose2d MIDDLE_STARTING_POSE_SPECIMEN = new Pose2d(0, 72-(ROBOT_LENGTH_IN/2), Math.PI/2);
    public static final Pose2d LEFT_STARTING_POSE_SPECIMEN = new Pose2d(24, 72-(ROBOT_LENGTH_IN/2), Math.PI/2);
    public static final Pose2d RIGHT_STARTING_POSE_SPECIMEN = new Pose2d(-20, 72-(ROBOT_LENGTH_IN/2), Math.PI/2);
    public static final Vector2d SUBMERSIBLE_PARK_POSITION = new Vector2d(20, 12);
    public static final Vector2d OBSERVATION_PARK_POSITION = new Vector2d(-60, 60);

    // hardware identifiers
    public static final String ARM_ELBOW_SERVO_HARDWARE_IDENTIFIER = "armElbowServo";
    public static final String SLIDES_MOTOR_HARDWARE_IDENTIFIER = "slidesMotor";
    public static final String ARM_BASE_MOTOR_HARDWARE_IDENTIFIER = "armBaseMotor";
    public static final String INTAKE_SERVO_HARDWARE_IDENTIFIER = "intakeServo";
    public static final String DUMPER_SERVO_HARDWARE_IDENTIFIER = "dumperServo";
    public static final String OTOS_HARDWARE_IDENTIFIER = "otos";
    public static final String PINPOINT_ODOMETRY_HARDWARE_IDENTIFIER = "pinpoint";
    public static final String LEFT_FRONT_MOTOR_HARDWARE_IDENTIFIER = "leftFront";
    public static final String LEFT_BACK_MOTOR_HARDWARE_IDENTIFIER = "leftBack";
    public static final String RIGHT_BACK_MOTOR_HARDWARE_IDENTIFIER = "rightBack";
    public static final String RIGHT_FRONT_MOTOR_HARDWARE_IDENTIFIER = "rightFront";
}
