package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

/*
This class contains various constants that are used throughout our code, sorted into general catagories.
 */
@Config
public class Constants {
  
    /* Constants used in TeleOp */

    public static final double TURN_STICK_DEADZONE = 0.01;
    public static final double TURN_POWER_MULTIPLIER = 1.0;
    public static final double DRIVE_FORWARD_MULTIPLIER = 0.7;
    public static final double DRIVE_STRAFE_MULTIPLIER = 1.0;
    public static final double TELEOP_MIN_HEADING_ACCURACY = 2.0; // degrees off from target
    public static final double SLOWMODE_MULTIPLIER = 0.3;
    public static final double INTAKE_POWER_MULTIPLIER = 0.8;
    public static final double DRONE_LAUNCHER_SERVO_PRIMED_POS = 0;
    public static final double DRONE_LAUNCHER_SERVO_LAUNCHING_POS = 0.2;

    public static final double SLIDES_STICK_DEADZONE = 0.01;
    public static final double SLIDE_MANUAL_MULTIPLIER = 0.5;
    public static final double SLIDE_RETURN_POWER_MULTIPLIER = 1.2;
    public static final double SLIDE_RETURN_POWER_OFFSET = -0.05;
    public static final double SLIDE_RETURN_UP_MUL = 0.66;

    public static final double SLIDE_MAX_POSITION = 4000;

    public static final double AUTO_SLIDES_MAX_SPEED = 0.7;
    // the tolerance for lowest and highest position when using stick manual:
    public static final double SLIDE_LIMIT_TOLERANCE = 20;
    // how close the slides need to be to count as "at" a preset during teleop,
    // only used for finding the next preset, NOT involved in moving slides to a position:
    public static final double SLIDE_NEAR_PRESET_RANGE = 100;
    // how close the slides need to be to target when moving to position:
    public static final double SLIDE_TO_POS_TOLERANCE = 50;
    public static final double SLIDE_P_GAIN = 0.001; // Multiplier modifier constant for slide power, used in preset moveSlides method.

    public static final double[] SLIDE_TELEOP_POSITIONS = {
        0,
        2000,
        4000,
    };


    public static final double[] DUMPER_POSITIONS = {
            -0.3, // extended
            0.89 // retracted
    };

    public static final double[] OUTTAKE_GATE_POSITIONS = {
            0.7, // open
            0.0 // closed
    };

    public static final double OUTTAKE_CONVEYOR_POWER = 0.8;
    public static final double[] INBAR_POSITIONS = { // Preset positions that the intake can travel to through use of Dpad Controls.
            0.37, // down
            0.80, // up
    };
    public static final double INBAR_MIN_POSITION = INBAR_POSITIONS[0];
    public static final double INBAR_MAX_POSITION = INBAR_POSITIONS[INBAR_POSITIONS.length - 1];
    public static final double INBAR_INIT_POSITION = 0.85;
    public static final double INBAR_MANUAL_RATE = 0.01;

  
    /* Constants used in AutoFramework */

    // /!\ old x and y factor calibration was for incorrect value of 537.7;
    public static final double TICKS_PER_REVOLUTION = 537.6;
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 3.78; // inches
    public static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    public static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    public static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    // /!\ x and y factors are outdated; was made using incorrect ticks per revolution
    public static final double AUTO_X_FACTOR = 1.0867924528301887;
    public static final double AUTO_Y_FACTOR = 0.9411764705882353;
    public static final double ROBOT_AUTO_SPEED = 0.5;

    // degrees off from target
    public static final double AUTO_MIN_HEADING_ACCURACY = 5.0;
    public static final int AUTO_SLIDES_HEIGHT = 2200;

  
  
    /* other constants */
  
    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(10, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 65, 95);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(140, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(90, 65, 25);
    public static final Scalar borderColors = new Scalar(255,170,0);
    public static final int CAMERA_WIDTH = 1920;
    public static final int CAMERA_HEIGHT = 1080;
    //public static final int CAMERA_WIDTH = 1280; // << for smol camera
    //public static final int CAMERA_HEIGHT = 960;
    public static final int SLICE_DIV_1 = CAMERA_WIDTH/3;
    public static final int SLICE_DIV_2 = (CAMERA_WIDTH/3)*2;
    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

}
