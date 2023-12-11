package org.firstinspires.ftc.team8923_CENTERSTAGE;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class ConstantsOpenCV {

    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(9, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 60, 50);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(120, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(108, 50, 50);
    public static final Scalar borderColor = new Scalar(0, 255, 0);  // green
    public static final int CAMERA_IMAGE_WIDTH = 640;
    public static final int CAMERA_IMAGE_HEIGHT = 480;

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

}
