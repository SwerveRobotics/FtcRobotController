package org.firstinspires.ftc.team6220_PowerPlay;

import android.graphics.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RobotCameraPipeline extends OpenCvPipeline {
    public double xPosition = Constants.CAMERA_CENTER_X;
    public double width = 0.0;

    Mat hierarchy = new Mat();
    Mat mat = new Mat();

    private Color targetColor;

    public enum Color {
        RED,
        BLUE,
        YELLOW,
        ALL
    }

    /**
     * set the color to filter for
     * @param color red, blue, yellow or all
     */
    public void setTargetColor(Color color) {
        targetColor = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        // transform the RGB frame into a HSV frame
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // blur the HSV frame
        Imgproc.GaussianBlur(mat, mat, Constants.BLUR_SIZE, 0);

        List<MatOfPoint> contours = getContoursFromColor();

        // find the largest contour if there is one
        if (contours.size() > 0) {
            double maxVal = 0.0;
            int maxValIdx = 0;

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }

            // crops contour if the area is below a certain size
            if (maxVal >= Constants.CONE_STACK_CONTOUR_MINIMUM_SIZE || maxVal <= 100000) {
                // get the bounding rectangle around the largest contour
                Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));

                // get moments
                Moments moments = Imgproc.moments(contours.get(maxValIdx), false);

                // draw the bounding rectangle on the frame
                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 10);

                if (moments.get_m00() > 0) {
                    xPosition = boundingRect.x + (boundingRect.width * 0.5);
                    width = boundingRect.width;
                }
            } else {
                width = 0.0;
                xPosition = Constants.CAMERA_CENTER_X;
            }
        } else {
            width = 0.0;
            xPosition = Constants.CAMERA_CENTER_X;
        }

        contours.clear();
        return input;
    }

    /**
     * take the frame and use the target color enum to find the contours of that color
     * @param mat the blurred frame
     * @return contour list
     */
    private List<MatOfPoint> getContoursFromColor() {

        switch(targetColor) {
            case RED:
                // mask the blurred frame
                Mat mat1 = new Mat();
                Mat mat2 = new Mat();
                Core.inRange(mat, Constants.LOWER_RED_BOTTOM, Constants.UPPER_RED_BOTTOM, mat1);
                Core.inRange(mat, Constants.LOWER_RED_UPPER, Constants.UPPER_RED_UPPER, mat2);
                // invert ranges if looking for red
                // this is because red is detected on both ends of the hue spectrum(0-20 & 160-180)
                // so we are looking for 20-160 and changing it so that it detects anything but that.
                //Core.bitwise_not(mat, mat);
                Core.bitwise_or(mat1,mat2,mat);
                mat1.release();
                mat2.release();
                break;

            case BLUE:
                Core.inRange(mat, Constants.LOWER_BLUE, Constants.UPPER_BLUE, mat);
                break;

            case YELLOW:
                Core.inRange(mat, Constants.LOWER_YELLOW, Constants.UPPER_YELLOW, mat);
                break;

            case ALL:
                Mat matRed = new Mat(), matBlue = new Mat(), matYellow = new Mat();
                Mat matLower = new Mat();
                Mat matUpper = new Mat();
                Core.inRange(mat, Constants.LOWER_RED_BOTTOM, Constants.UPPER_RED_BOTTOM, matLower);
                Core.inRange(mat, Constants.LOWER_RED_UPPER, Constants.UPPER_RED_UPPER, matUpper);
                // invert ranges if looking for red
                // this is because red is detected on both ends of the hue spectrum(0-20 & 160-180)
                // so we are looking for 20-160 and changing it so that it detects anything but that.
                //Core.bitwise_not(mat, mat);
                Core.bitwise_or(matUpper,matLower,matRed);
                Core.inRange(mat, Constants.LOWER_BLUE, Constants.UPPER_BLUE, matBlue);
                Core.inRange(mat, Constants.LOWER_YELLOW, Constants.UPPER_YELLOW, matYellow);

                // put together all the color masks into one single mask, saved to mat
                Core.bitwise_or(matRed, matBlue, mat); // combine the red and blue masks, save to mat
                Core.bitwise_or(mat, matYellow, mat); // combine the previous combined and the yellow mask, save into mat
                matUpper.release();
                matLower.release();
                matRed.release();
                matBlue.release();
                matYellow.release();
                break;
        }

        // find the contours in the masked frame
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }
}
