package org.firstinspires.ftc.team6220;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public abstract class Constants {
    public static double ARM_ELBOW_SERVO_PRESET_POSITION_OVER_BARRIER = -0.1;
    public static double ARM_ELBOW_SERVO_PRESET_POSITION_GROUND = 0.25;
    public static int ARM_BASE_MOTOR_POSITION_GROUND = -1680;
    public static int ARM_BASE_MOTOR_POSITION_FOLD = 0;
    public static int ARM_BASE_MOTOR_POSITION_SUB = -1450;
    public static int SLIDES_MOTOR_GROUND_POSITION = 0;
    public static int SLIDES_MOTOR_POSITION_ONE = 20;
    public static int SLIDES_MOTOR_POSITION_TWO = 40;
    public static int SLIDES_MOTOR_VELOCITY = 8;
    public static int ARM_BASE_MOTOR_VELOCITY = 8;
    public static final float[] RED_COLOR_SENSOR_RANGES = {0.025f, 1f};
    public static final float[] BLUE_COLOR_SENSOR_RANGES = {0.025f, 1f};
    public static final float COLOR_SENSOR_GAIN = 3.8f;
    //public static final Pose2d BASKET_SCORING_POSE = new Pose2d()
    public static final Pose2d MIDDLE_STARTING_POSE = new Pose2d(0, 60, (3*Math.PI)/2);
    public static final Pose2d LEFT_STARTING_POSE = new Pose2d(-24, 60, (3*Math.PI)/2);
    public static final Pose2d RIGHT_STARTING_POSE = new Pose2d(24, 60, (3*Math.PI)/2);
}
