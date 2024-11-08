package org.firstinspires.ftc.team417.liveView;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/** @noinspection StringConcatenationInLoop*/
public class LiveView {
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

    private final int redMin = 200;
    private final int blueMin = 200;

    double lTime = 0;

    public LiveView(Telemetry t) {
        this.t = t;
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

    public void processBitmap(int[][][] bitMap) {
        message = "";

        for (int[][] row : bitMap) {
            for (int[] pixel : row) {
                if (pixel[1] > blueMin && lPixel[1] < blueMin)
                    message += "<blue>";
                else if (pixel[1] < blueMin && lPixel[1] > blueMin)
                    message += "</blue>";
                else if (pixel[2] > redMin && lPixel[2] < redMin)
                    message += "<red>";
                else if (pixel[2] < redMin && lPixel[2] > redMin)
                    message += "</red>";


                if (pixel[0] > intensityRange * 4.0 / 5.0 + intensityRangeMin)
                    message += "█";
                else if (pixel[0] > intensityRange * 3.0 / 5.0 + intensityRangeMin)
                    message += "▓";
                else if (pixel[0] > intensityRange * 2.0 / 5.0 + intensityRangeMin)
                    message += "▒";
                else if (pixel[0] > intensityRange / 5.0 + intensityRangeMin)
                    message += "░";
                else
                    message += " ";

                lPixel = pixel.clone();
            }
            message += "\n";
        }

        sendImage();
    }

    private void sendImage() {
        //t.addLine(messageStart + message + messageEnd);
        System.out.print(messageStart + message + messageEnd);
    }
}
