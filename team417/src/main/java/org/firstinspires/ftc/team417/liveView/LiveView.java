package org.firstinspires.ftc.team417.liveView;

import static java.lang.System.nanoTime;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/** @noinspection StringConcatenationInLoop*/
public class LiveView implements VisionProcessor {
    Telemetry t;
    TelemetryPacket p;
    String message;
    String messageStart;
    String messageEnd;

    private final int defaultScreenWidth = 32;
    private final int defaultScreenHeight = 16;
    private double currScreenWidth = defaultScreenWidth;
    private double currScreenHeight = defaultScreenHeight;

    private final int intensityRangeMin = -128;
    private final int intensityRangeMax = 127;
    private final int intensityRange = intensityRangeMax - intensityRangeMin;

    private final int redMin = 200;
    private final int blueMin = 200;

    double lTime = 0;

    // Big memory buffers are very expensive to allocate so create them once and reuse
    // rather than recreating them on every loop:
    Mat smallRgb = new Mat();
    Mat smallYCrCb = new Mat();
    byte[] buffer = new byte[0]; // Will resize later...

    int width;
    int height;


    public LiveView(Telemetry t, int width, int height) {
        this.t = t;
        this.width = width;
        this.height = height;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    // This routine shows how to get the raw pixels from the bitmap. As a test, it simply
    // gathers some statistics about the bitmap's contents.
    double[] mins = new double[4];
    double[] maxes = new double[4];
    byte[] inspect(Mat mat) {
        return buffer;
    }

    // The system calls processFrame every time the camera acquires a new frame (typically 30
    // frames per second). Note that this is on a different thread than the robot's primary loop:
    @Override
    public Object processFrame(Mat largeRgb, long captureTimeNanos) {
        int pixelCount = smallYCrCb.height() * smallYCrCb.width();
        int channelCount = smallYCrCb.channels();
        int byteCount = pixelCount * channelCount;

        // First resize the frame that came from the camera:
        Imgproc.resize(largeRgb, smallRgb, new org.opencv.core.Size(width, height), 0, 0, Imgproc.INTER_AREA);

        // Convert it to YCrCb (Y is luminance/intensity, Cr is red chroma, Cb is blue chroma):
        Imgproc.cvtColor(smallRgb, smallYCrCb, Imgproc.COLOR_RGB2YCrCb);

        if (buffer.length < byteCount)
            buffer = new byte[byteCount];

        // Initialize the min/max statistics to be extremely crossed:
        for (int i = 0; i < channelCount; i++) {
            mins[i] = Double.MAX_VALUE;
            maxes[i] = 0;
        }

        // Read the pixels from the 'Mat' object into our buffer:
        smallYCrCb.get(0, 0, buffer);

        processYCrCb(buffer);

        return null;
    }

    // I think this is called when the system wants us to preview our processing:
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void initHTML() {
        t.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        messageStart = "<tt><span style='color: #ffffff; background: gray;'>";
        messageEnd = "</span></tt>";
    }

    public void resize(int factor) {

        for (int i = 0; i < factor; i++) {
            messageStart += "<small>";
            messageEnd = "</small>" + messageEnd;
        }

        for (int i = 0; i > factor; i--) {
            if (currScreenWidth > defaultScreenWidth) {
                messageStart = messageStart.replaceFirst("<small>", "");
                messageEnd = messageEnd.replaceFirst("</small>", "");
            }
        }

        currScreenHeight = currScreenHeight * Math.pow(1.2, factor);
        currScreenWidth = currScreenWidth * Math.pow(1.2, factor);
        System.out.println(currScreenHeight + " ," + currScreenWidth + "\n");
        System.out.println(messageStart + " ," + messageEnd + "\n");
    }

    public void allWhiteTest(double width, double height) {
        message = "";

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                message += "█";
            }
            message += "\n";
        }

        sendImage();
    }

    public void trackLoopTime(int windowSize, TelemetryPacket p) {
        double currTime = nanoTime() * 1e-9;
        double deltaTime = currTime - lTime;


        Filter filter = new Filter();
        double aveTime = filter.slidingWindow(deltaTime, windowSize);

        t.addData("deltaTime", deltaTime);
        t.addData("aveTime", aveTime);

        lTime = currTime;
    }

    public int[][][] fadeTest(int width, int height) {
        int blue = 0;

        int[][][] bitMap = new int[height][width][3];
        for (int i = 1; i < height; i++) {
            for (int j = 1; j < width; j++) {
                int[] pixel = {j, blue, 0};
                bitMap[i][j] = pixel;

                blue += 100;
                if (blue > 200)
                    blue = 0;
            }
        }

        for (int[][] i : bitMap) {
            for (int[] j: i) {
                System.out.print(j[0]);
                System.out.print(", ");
            }
            System.out.println();
        }

        return bitMap;
    }

    private int[] lPixel = {0, 0, 0};

    public void processYCrCb(byte[] bitMap) {
        message = "";

        for (int i = 0; i < bitMap.length; i += 3) {
            if (bitMap[i] > intensityRange * 4.0 / 5.0 + intensityRangeMin)
                message += "█";
            else if (bitMap[i] > intensityRange * 3.0 / 5.0 + intensityRangeMin)
                message += "▓";
            else if (bitMap[i] > intensityRange * 2.0 / 5.0 + intensityRangeMin)
                message += "▒";
            else if (bitMap[i] > intensityRange / 5.0 + intensityRangeMin)
                message += "░";
            else
                message += " ";

            if (i % width == width - 1)
                message += "\n";
        }

        sendImage();
    }

    private void sendImage() {
        //t.addLine(messageStart + message + messageEnd);
        System.out.print(messageStart + message + messageEnd);
    }
}
