package org.firstinspires.ftc.team6220_CENTERSTAGE;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {

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
