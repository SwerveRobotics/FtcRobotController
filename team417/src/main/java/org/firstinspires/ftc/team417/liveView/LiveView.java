package org.firstinspires.ftc.team417.liveView;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class LiveView implements VisionProcessor {
    Telemetry t;
    Gamepad gamepad1;
    String message;
    String messageStart;
    String messageEnd;
    public String image;

    private final int onThreshold = 50;
    private final int onValue = 100;
    private final int offValue = 0;

    //PLACEHOLDER VALUES________________________________________________
    private final int COLOR_EPSILON = 35;
    /*private final Lab YELLOW = new Lab(82.35, -3, 78);
    private final Lab BLUE = new Lab(32.941, 21, -56);
    private final Lab RED = new Lab(50.6, 62, 33);*/

    private final Lab YELLOW = new Lab(61.2, -13, -40);
    private final Lab BLUE = new Lab(29, 34, 28);
    private final Lab RED = new Lab(27.1, 53, -77);
    private final String[] RGB_CODES = {"yellow", "#00bfff", "red"};

    private final int NUM_RUNS_KEPT = 100;

    public boolean tune = false;

    // Big memory buffers are very expensive to allocate so create them once and reuse
    // rather than recreating them on every loop:
    Mat smallRgb = new Mat();
    Mat smallLab = new Mat();
    byte[] buffer = new byte[0]; // Will resize later...

    int inputWidth;
    int inputHeight;
    int outputWidth;
    int outputHeight;


    public LiveView(Telemetry t, int width, int height, Gamepad gamepad1) {
        this.t = t;
        this.gamepad1 = gamepad1;
        inputWidth = width;
        inputHeight = height;
        outputWidth = width / 2;
        outputHeight = height / 4;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    private void newBuffer(Mat largeRgb, int width, int height) {
        // First resize the frame that came from the camera:
        Imgproc.resize(largeRgb, smallRgb, new org.opencv.core.Size(width, height), 0, 0, Imgproc.INTER_AREA);

        // Convert it to YCrCb (Y is luminance/intensity, Cr is red chroma, Cb is blue chroma):
        Imgproc.cvtColor(smallRgb, smallLab, Imgproc.COLOR_RGB2Lab);


        int pixelCount = smallLab.height() * smallLab.width();
        int channelCount = smallLab.channels();
        int byteCount = pixelCount * channelCount;

        if (buffer.length < byteCount)
            buffer = new byte[byteCount];

        // Read the pixels from the 'Mat' object into our buffer:
        smallLab.get(0, 0, buffer);
    }

    // The system calls processFrame every time the camera acquires a new frame (typically 30
    // frames per second). Note that this is on a different thread than the robot's primary loop:
    @Override
    public Object processFrame(Mat largeRgb, long captureTimeNanos) {
        newBuffer(largeRgb, inputWidth, inputHeight);

        double[][] intensities = new double[inputWidth][inputHeight];

        for (int i = 0; i < inputWidth * inputHeight * 3; i += 3) {
            int x = (i / 3) % inputWidth;
            int y = (i / 3) / inputWidth;

            intensities[x][y] = TypeConversion.unsignedByteToDouble(buffer[i]) / 255.0 * 100.0;
        }

        newBuffer(largeRgb, outputWidth, outputHeight);

        Lab[][] colors = new Lab[outputWidth][outputHeight];

        for (int i = 0; i < outputWidth * outputHeight * 3; i += 3) {
            int x = (i / 3) % outputWidth;
            int y = (i / 3) / outputWidth;

            double L = TypeConversion.unsignedByteToDouble(buffer[i + 0]) / 255.0 * 100.0;
            double a = TypeConversion.unsignedByteToDouble(buffer[i + 1]) - 128;
            double b = TypeConversion.unsignedByteToDouble(buffer[i + 2]) - 128;

            colors[x][y] = new Lab(L, a, b);
        }

        processLab(intensities, colors);

        return null;
    }

    // I think this is called when the system wants us to preview our processing:
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void initHTML() {
        t.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        messageStart = "<tt><span style='background: black;'><b>";
        messageEnd = "</b></span></tt>";
        tuneMessage = "";
    }

    public void resize(int factor) {
        StringBuilder start = new StringBuilder();
        StringBuilder end = new StringBuilder();

        for (int i = 0; i < factor; i++) {
            start.append("<small>");
            end.append("</small>");
        }

        messageStart += start.toString();
        messageEnd = end.toString() + messageEnd;
    }

    public void allWhiteTest(double width, double height) {
        StringBuilder message = new StringBuilder();

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                message.append((char) 0x283F);
            }
            message.append('\n');
        }

        this.message = message.toString();

        sendImage();
    }

    public double[][] fadeTest() {
        double[][] bitMap = new double[inputWidth + 2][inputHeight + 2];

        for (int y = 1; y < inputHeight - 1; y++) {
            for (int x = 1; x < inputWidth - 1; x++) {
                bitMap[x][y] = Math.round(((double) x / inputWidth) * 255);
            }
        }

        return bitMap;
    }

    private String toString(List<Character> chars) {
        StringBuilder string = new StringBuilder();

        for (char c : chars) {
            string.append(c);
        }

        return string.toString();
    }

    String tuneMessage;
    boolean aPressed = false;
    boolean changeMessage = true;

    private int tuneColorValues(int x, int y, int charHex, Lab[][] colors) {
        int u = 40;
        int v = 20;



        if (((x == u) && (y == v-1)) ||
                ((x == u) && (y == v+1)) ||
                ((x == u-1) && (y == v)) ||
                ((x == u+1) && (y==v))) {
            charHex = '█';
        }

        if (gamepad1.a && !aPressed)
            changeMessage = !changeMessage;

        aPressed = gamepad1.a;

        if ((x == u) && (y == v) & changeMessage)
            tuneMessage = "\n----------------------------------------\nL: " + colors[x][y].L + " a: " + colors[x][y].a + " b: " + colors[x][y].b + "\n";

        return charHex;
    }

    public void processLab(double[][] intensities, Lab[][] colors) {
        ArrayList<Character> image = new ArrayList<>(outputWidth * outputHeight);

        ArrayList<CromaRun> runs = new ArrayList<>();

        for (int y = 0; y < outputHeight; y++) {
            CromaRun currRun = new CromaRun(0, 0, 0, CromaRun.Colors.OTHER);

            for (int x = 0; x < outputWidth; x++) {
                int charHex = 0x2800;

                for (int i = 0; i < 8; i++) {
                    int pixelX = x * 2 + i % 2;
                    int pixelY = y * 4 + i / 2;

                    double value = intensities[pixelX][pixelY];
                    double newValue;

                    if (value > onThreshold) {
                        newValue = onValue;

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
                        newValue = offValue;

                    double deltaValue = value - newValue;

                    if (pixelX < inputWidth - 1)
                        intensities[pixelX + 1][pixelY] += deltaValue * 7.0 / 16.0;
                    if (pixelY < inputHeight - 1) {
                        if (pixelX > 0)
                            intensities[pixelX - 1][pixelY + 1] += deltaValue * 3.0 / 16.0;
                        intensities[pixelX][pixelY + 1] += deltaValue * 5.0 / 16.0;
                        if (pixelX < inputWidth - 1)
                            intensities[pixelX + 1][pixelY + 1] += deltaValue / 16.0;
                    }
                }

                CromaRun.Colors color;
                if (colors[x][y].equals(YELLOW, COLOR_EPSILON))
                    color = CromaRun.Colors.YELLOW;
                else if (colors[x][y].equals(BLUE, COLOR_EPSILON))
                    color = CromaRun.Colors.BLUE;
                else if (colors[x][y].equals(RED, COLOR_EPSILON))
                    color = CromaRun.Colors.RED;
                else
                    color = CromaRun.Colors.OTHER;

                if (color == currRun.color)
                    currRun.length += 1;
                else {
                    if (currRun.color != CromaRun.Colors.OTHER)
                        runs.add(currRun);
                    currRun = new CromaRun(x, y, 1, color);
                }

                if (tune)
                    charHex = tuneColorValues(x, y, charHex, colors);

                image.add((char) charHex);
            }

            if (currRun.color != CromaRun.Colors.OTHER)
                runs.add(currRun);

            image.add('\n');
        }

        runs.sort(Comparator.comparingInt(run -> -run.length));

        if (runs.size() > NUM_RUNS_KEPT) {
            runs.subList(NUM_RUNS_KEPT, runs.size()).clear();
        }

        runs.sort(Comparator.comparingInt(run->run.x + run.y * outputWidth));

        StringBuilder strImage = new StringBuilder();
        int lastEndIndex = 0;

        for (CromaRun run : runs) {
            int runStart = run.y * (outputWidth + 1) + run.x;
            int runEnd = runStart + run.length;

            if (lastEndIndex != runStart) {
                strImage.append(toString(image.subList(lastEndIndex, runStart)));
            }

            strImage.append(String.format("<font color='%s'>", RGB_CODES[run.color.getValue()]));
            strImage.append(toString(image.subList(runStart, runEnd)));
            strImage.append("</font>");

            lastEndIndex = runEnd;
        }

        strImage.append("</span><span style='color: #FFFFFF; background: black;'>");
        strImage.append(toString(image.subList(lastEndIndex, image.size())));

        message = strImage.toString();

        sendImage();
    }

    private void sendImage() {
        t.setMsTransmissionInterval(100);
        image = messageStart + message + messageEnd + tuneMessage;
    }
}
