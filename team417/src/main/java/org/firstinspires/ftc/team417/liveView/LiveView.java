package org.firstinspires.ftc.team417.liveView;

import static java.lang.System.nanoTime;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

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

    private final int intensityRangeMin = 0;
    private final int intensityRangeMax = 255;
    private final int intensityRange = intensityRangeMax - intensityRangeMin;

    //PLACEHOLDER VALUES________________________________________________
    private final int COLOR_EPSILON = 100;
    private final Lab YELLOW = new Lab(0, 0, 0);
    private final Lab BLUE = new Lab(0, 0, 0);
    private final Lab RED = new Lab(0 , 0, 0);

    private final int numRunsKept = 200;

    double lTime = 0;

    // Big memory buffers are very expensive to allocate so create them once and reuse
    // rather than recreating them on every loop:
    Mat smallRgb = new Mat();
    Mat smallYCrCb = new Mat();
    byte[] buffer = new byte[0]; // Will resize later...

    int inputWidth;
    int inputHeight;
    int outputWidth;
    int outputHeight;


    public LiveView(Telemetry t, int width, int height) {
        this.t = t;
        inputWidth = width;
        inputHeight = height;
        outputWidth = width / 2;
        outputHeight = height / 4;
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
        int pixelCount;
        int channelCount;
        int byteCount;

        // First resize the frame that came from the camera:
        Imgproc.resize(largeRgb, smallRgb, new org.opencv.core.Size(inputWidth, inputHeight), 0, 0, Imgproc.INTER_AREA);

        // Convert it to YCrCb (Y is luminance/intensity, Cr is red chroma, Cb is blue chroma):
        Imgproc.cvtColor(smallRgb, smallYCrCb, Imgproc.COLOR_RGB2Lab);

        pixelCount = smallYCrCb.height() * smallYCrCb.width();
        channelCount = smallYCrCb.channels();
        byteCount = pixelCount * channelCount;

        if (buffer.length < byteCount)
            buffer = new byte[byteCount];

        // Read the pixels from the 'Mat' object into our buffer:
        smallYCrCb.get(0, 0, buffer);

        double[][] intensities = new double[inputWidth + 2][inputHeight + 2];

        for (int i = 0; i < buffer.length; i += 3) {
            int x = (i / 3) % inputWidth;
            int y = (i / 3) / inputWidth;

            intensities[x + 1][y + 1] = TypeConversion.unsignedByteToDouble(buffer[i]);
        }

        // First resize the frame that came from the camera:
        Imgproc.resize(largeRgb, smallRgb, new org.opencv.core.Size(outputWidth / 2.0, outputHeight / 4.0), 0, 0, Imgproc.INTER_AREA);

        // Convert it to YCrCb (Y is luminance/intensity, Cr is red chroma, Cb is blue chroma):
        Imgproc.cvtColor(smallRgb, smallYCrCb, Imgproc.COLOR_RGB2Lab);

        pixelCount = smallYCrCb.height() * smallYCrCb.width();
        channelCount = smallYCrCb.channels();
        byteCount = pixelCount * channelCount;

        if (buffer.length < byteCount)
            buffer = new byte[byteCount];

        // Read the pixels from the 'Mat' object into our buffer:
        smallYCrCb.get(0, 0, buffer);

        Lab[][] colors = new Lab[outputWidth + 2][outputHeight + 2];

        for (int i = 0; i < buffer.length; i += 3) {
            int x = (i / 3) % outputWidth;
            int y = (i / 3) / outputWidth;

            double L = TypeConversion.unsignedByteToDouble(buffer[i    ]);
            double a = TypeConversion.unsignedByteToDouble(buffer[i + 1]);
            double b = TypeConversion.unsignedByteToDouble(buffer[i + 2]);

            colors[x + 1][y + 1] = new Lab(L, a, b);
        }

        processYCrCb(intensities, colors);

        return null;
    }

    // I think this is called when the system wants us to preview our processing:
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void initHTML() {
        t.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        messageStart = "<tt><span style='color: #ffffff; background: black;'><b>";
        messageEnd = "</b></span></tt>";
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

    public byte[] fadeTest() {
        ArrayList<Byte> bitMap = new ArrayList<>();
        for (int i = 0; i < inputHeight; i++) {
            for (int j = 0; j < inputWidth; j++) {
                byte y = (byte) ((j * 255) / inputWidth); // Scale `j` to range [0, 255]
                bitMap.add(y);
                bitMap.add((byte) 0);
                bitMap.add((byte) 0);
            }
        }

        byte[] byteMap = new byte[bitMap.size()];
        for (int i = 0; i < bitMap.size(); i++) {
            byteMap[i] = bitMap.get(i);
        }

        return byteMap;
    }

    public void processYCrCb(double[][] intensities, Lab[][] colors) {
        message = "";
        ArrayList<Character> image = new ArrayList<>();

        ArrayList<CromaRun> runs = new ArrayList<>();
        CromaRun currRun = new CromaRun(0, 0, 0, "");

        for (int y = 1; y < outputHeight / 4- 1; y++) {
            for (int x = 1; x < outputWidth / 2 - 1; x++) {
                int charHex = 0x2800;
                
                for (int i = 0; i < 8; i++) {
                    int pixelX = x * 2 + i % 2;
                    int pixelY = y * 4 + i / 2;
                    
                    double value = intensities[pixelX][pixelY];
                    double newValue;

                    if (value > 55) {
                        newValue = 155;

                        if (i == 0)
                            charHex += 0x1;
                        if (i == 1)
                            charHex += 0x8;
                        if (i == 2)
                            charHex += 0x2;
                        if (i == 3)
                            charHex += 0x10;
                        if (i == 4)
                            charHex += 0x4;
                        if (i == 5)
                            charHex += 0x20;
                        if (i == 6)
                            charHex += 0x40;
                        if (i == 7)
                            charHex += 0x80;

                    } else
                        newValue = 27;

                    double deltaValue = value - newValue;

                    intensities[pixelX + 1][pixelY    ] += deltaValue * 7.0 / 16.0;
                    intensities[pixelX - 1][pixelY + 1] += deltaValue * 3.0 / 16.0;
                    intensities[pixelX    ][pixelY + 1] += deltaValue * 5.0 / 16.0;
                    intensities[pixelX + 1][pixelY + 1] += deltaValue / 16.0;

                    String color;
                    if (colors[x][y].equals(YELLOW, COLOR_EPSILON))
                        color = "Yellow";
                    else if (colors[x][y].equals(BLUE, COLOR_EPSILON))
                        color = "Blue";
                    else if (colors[x][y].equals(RED, COLOR_EPSILON))
                        color = "Red";
                    else
                        color = "";

                    if (color.equals(currRun.color))
                        currRun.length += 1;
                    else {
                        if (!currRun.color.isEmpty())
                            runs.add(currRun);
                        currRun = new CromaRun(x, y, 1, color);
                    }
                }
                
                image.add((char) charHex);
            }

            image.add('\n');
        }

        runs.sort(Comparator.comparingInt(run->-run.length));
        System.out.print("Largest run: ");
        System.out.print(runs.get(0));
        System.out.print(" Smallest run: ");
        System.out.print(runs.get(runs.size() - 1));

        if (runs.size() > numRunsKept) {
            runs.subList(numRunsKept, runs.size()).clear();
        }

        runs.sort(Comparator.comparingInt(run->run.x + run.y * outputWidth));

        StringBuilder strImage = new StringBuilder();
        int lEndIndex = 0;

        for (int i = 0; i < runs.size(); i++) {
            int runStart = runs.get(i).y * (outputWidth + 1) + runs.get(i).x;
            int runEnd = runStart + runs.get(i).length;

            strImage.append(image.subList(lEndIndex + 1, runStart));
            strImage.append(String.format("<%s>", runs.get(i).color));
            strImage.append(image.subList(runStart, runEnd + 1));
            strImage.append(String.format("</%s>", runs.get(i).color));

            lEndIndex = runEnd;
        }

        message = strImage.toString();

        sendImage();
    }

    private void sendImage() {
        t.addLine(messageStart + message + messageEnd);
        System.out.print(messageStart + message + messageEnd);
        System.out.print("\n");
    }
}
