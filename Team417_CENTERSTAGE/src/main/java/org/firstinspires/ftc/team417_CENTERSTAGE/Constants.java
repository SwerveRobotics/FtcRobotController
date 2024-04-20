package org.firstinspires.ftc.team417_CENTERSTAGE;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {
    //time since last loop in seconds
    public static double DELTA_T;
    public static double TIME;
    public static double LOOP_TIME;
    private static double lastT;
    private static final int loopTimeWindow = 50;
    private static Filters filter;

    public static void init() {
        filter = new Filters();
    }

    //update delta time variable
    public static void updateT() {
        double time = BaseOpMode.TIME.seconds();
        DELTA_T = time - lastT;
        lastT = time;

        //throttle DELTA_T, to fake real time, while debugging.
        if (DELTA_T > 50.0 / 1000.0) {
            DELTA_T = 20.0 / 1000.0;
        }
        LOOP_TIME = filter.slidingWindow(loopTimeWindow, DELTA_T);

        TIME += DELTA_T;
    }

    // Camera constants for OpenCV detection
    public static final int CAMERA_IMAGE_WIDTH = 640;
    public static final int CAMERA_IMAGE_HEIGHT = 480;

    // Roi (region of interest) for OpenCV detection
    public static final double xLowerBound = 2.0 / 3;
    public static final double xUpperBound = 1;
    public static final double yLowerBound = 2.0 / 3;
    public static final double yUpperBound = 1;
    public static final Rect roi = new Rect(0, (int) ((2 * CAMERA_IMAGE_HEIGHT) / 3.0), CAMERA_IMAGE_WIDTH, (int) (CAMERA_IMAGE_HEIGHT / 3.0));
    public static final Scalar roiColor = new Scalar(0, 255, 255); // cyan

    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(8, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 60, 50);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(120, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(110, 60, 50);
    public static final Scalar borderColor = new Scalar(0, 255, 0);  // green

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

    //epsilons for path following
    public static final double LINE_APROX_EPSILON = 0.01;
}
