package org.firstinspires.ftc.team8923_CENTERSTAGE;

import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;

public class ConstantsOpenCV {

    // Color ranges for OpenCV detection
    public static final int CAMERA_IMAGE_WIDTH = 640;
    public static final int CAMERA_IMAGE_HEIGHT = 480;
    public static final Rect roi = new Rect(0, (int) ((2 * CAMERA_IMAGE_HEIGHT) / 3.0), CAMERA_IMAGE_WIDTH, (int) (CAMERA_IMAGE_HEIGHT / 3.0));
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(5, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 60, 50);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(119, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(108, 60, 50);
    public static final Scalar borderColor = new Scalar(0, 255, 0);  // green

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

}
