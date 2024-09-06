/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Modified by Truong Nguyen
 * 2023/11/03
 * Example program to detect blue or red objects
 *
 * Modified further by Hankang Zhou
 * 2024/08/22
 * Competition program for detecting colored objects
 */

package org.firstinspires.ftc.team417.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This class demonstrates the use of EasyOpenCV to detect color objects in HSV space
 * It uses the USB webcam named "RobotCamera" in the robot configuration, connected to the REV control hub
 */

public class OpenCVColorDetection {
    public OpenCvCamera robotCamera;
    /* Declare OpMode members. */
    public LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    public enum detectColorType {
        BLUE,
        RED,
        UNSET,
    }

    public detectColorType myColor = detectColorType.UNSET;

    // coordinates of largest detected image
    Point targetPoint = new Point(0, 0);
    boolean targetDetected;

    // Define a constructor that allows the OpMode to pass a reference to itself
    //   in order to allow this class to access the camera hardware
    public OpenCVColorDetection(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // set detection color
    public void setDetectColor(detectColorType color) {
        myColor = color;
    }

    // initialize the camera and openCV pipeline
    public void init() {
        // cameraMonitorViewId allows us to see the image pipeline using scrcpy
        //   for easy debugging
        //   You can disable it after testing completes
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        // robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));
        // OR... use internal phone camera
        // phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        robotCamera.setPipeline(new ColorDetectPipeline());

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                startStreaming();
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void stopStreaming() {
        robotCamera.stopStreaming();
    }

    public void startStreaming() {
        robotCamera.startStreaming(Constants.CAMERA_IMAGE_WIDTH, Constants.CAMERA_IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    // Four possibilities for the camera; SideDetected.INITIALIZED means the camera
    //    hasn't had a chance to detect anything yet
    public enum SideDetected {
        INITIALIZED,
        LEFT,
        CENTER,
        RIGHT
    }

    public SideDetected sideDetected = SideDetected.INITIALIZED;

    /* openCV image processing pipeline
     *   detects largest blue or red colored object
     */
    class ColorDetectPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;

        // matrices in the processing pipeline
        Mat roiMat = new Mat();
        Mat blurredMat = new Mat();
        Mat hsvMat = new Mat();
        Mat filteredMat = new Mat();
        Mat contourMask = new Mat();
        Mat outputMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        MatOfPoint currentContour = new MatOfPoint();
        List<MatOfPoint> offsetContoursList = new ArrayList<>();
        MatOfPoint offsetContour = new MatOfPoint();

        @Override
        public Mat processFrame(Mat inputMat) {
            // resize the image to the roi so that stuff like volunteer's shirts aren't detected
            roiMat = inputMat.submat(Constants.roi);

            // blur the image to reduce the impact of noisy pixels
            //   each pixel is "averaged" with its neighboring pixels
            Imgproc.GaussianBlur(roiMat, blurredMat, Constants.BLUR_SIZE, 0);

            // convert image to HSV color space, which is better for detecting red and blue colors
            Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

            // filter out the range of blue or red colors defined in
            //   Constants.BLUE_COLOR_DETECT_MIN_HSV and Constants.BLUE_COLOR_DETECT_MAX_HSV
            //   or Constants.RED_COLOR_DETECT_MIN_HSV and Constants.RED_COLOR_DETECT_MAX_HSV
            switch (myColor) {
                case BLUE:
                    Core.inRange(hsvMat, Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV, filteredMat);
                    break;
                case RED:
                    Core.inRange(hsvMat, Constants.RED_COLOR_DETECT_MIN_HSV, Constants.RED_COLOR_DETECT_MAX_HSV, filteredMat);
                    break;
                default:
            }

            // Clears list from the last loop to use as a empty
            contoursList.clear();

            // create a list of contours surrounding groups of contiguous pixels that were filtered
            Imgproc.findContours(filteredMat, contoursList, contourMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // copy original image to output image for drawing overlay on
            roiMat.copyTo(outputMat);

            // if no contours are detected, do nothing
            int maxAreaContourIndex;
            targetPoint.x = -1;
            targetPoint.y = -1;
            targetDetected = false;
            if (contoursList.size() > 0) {
                // iterate through list of contours, find max area contour
                double maxArea = 0.0;
                maxAreaContourIndex = 0;
                for (int i = 0; i < contoursList.size(); i++) {
                    currentContour = contoursList.get(i);
                    double contourArea = Imgproc.contourArea(currentContour);
                    if (contourArea > maxArea) {
                        maxArea = contourArea;
                        maxAreaContourIndex = i;
                    }
                }
                targetDetected = true;

                // Draw the max area contour at index maxAreaContourIndex for debugging in scrcpy
                Imgproc.drawContours(outputMat, contoursList, maxAreaContourIndex, Constants.borderColor, 2, -1);

                //   draw rectangular bounding box around the max area contour
                //   and draw circle at the center of rectangular bounding box
                Rect boundingRect = Imgproc.boundingRect(contoursList.get(maxAreaContourIndex));
                double boundHeightX = boundingRect.x + boundingRect.width;
                double boundHeightY = boundingRect.y + boundingRect.height;
                Imgproc.rectangle(outputMat, new Point(boundingRect.x, boundingRect.y), new Point(boundHeightX, boundHeightY), Constants.borderColor, 3, Imgproc.LINE_8, 0);
                targetPoint.x = (int) boundingRect.width / 2.0 + boundingRect.x;
                targetPoint.y = (int) boundingRect.height / 2.0 + boundingRect.y;
                Imgproc.circle(outputMat, targetPoint, 10, Constants.borderColor, Imgproc.LINE_4, -1);

                double width = inputMat.size().width;

                // Update sideDetected so that robot can detect position with detectPosition()
                // Takes the left fourth of the screen for left spike mark, right fourth for right, and
                //    otherwise is center
                if (targetPoint.x < width / 4) {
                    sideDetected = SideDetected.LEFT;
                } else if (targetPoint.x > (3 * width) / 4) {
                    sideDetected = SideDetected.RIGHT;
                } else {
                    sideDetected = SideDetected.CENTER;
                }
            }

            roiMat.release();

            // See this image on the computer using scrcpy
            return outputMat;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                robotCamera.pauseViewport();
            } else {
                robotCamera.resumeViewport();
            }
        }
    }

    public SideDetected detectTeamProp() {
        return sideDetected;
    }
}