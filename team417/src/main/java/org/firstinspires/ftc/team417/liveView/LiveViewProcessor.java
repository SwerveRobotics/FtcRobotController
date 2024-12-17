package org.firstinspires.ftc.team417.liveView;

import static java.lang.System.nanoTime;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

// This is a test processor that downsizes, color converts and then uses the CPU to analyze
// camera input.
public class LiveViewProcessor implements VisionProcessor {
    // Some statistics, can be removed:
    double lastTime;
    int frameCount;
    int totalCount;

    // Big memory buffers are very expensive to allocate so create them once and reuse
    // rather than recreating them on every loop:
    Mat smallRgb = new Mat();
    Mat smallYCrCb = new Mat();
    byte[] buffer = new byte[0]; // Will resize later...

    // Get current time, in seconds:
    public double time() {
        return nanoTime() * 1e-9;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    // This routine shows how to get the raw pixels from the bitmap. As a test, it simply
    // gathers some statistics about the bitmap's contents.
    double[] mins = new double[4];
    double[] maxes = new double[4];
    void inspect(Mat mat, int totalCount) {
        int pixelCount = mat.height() * mat.width();
        int channelCount = mat.channels();
        int byteCount = pixelCount * channelCount;
        if (buffer.length < byteCount)
            buffer = new byte[byteCount];

        // Initialize the min/max statistics to be extremely crossed:
        for (int i = 0; i < channelCount; i++) {
            mins[i] = Double.MAX_VALUE;
            maxes[i] = 0;
        }

        // Read the pixels from the 'Mat' object into our buffer:
        mat.get(0, 0, buffer);

        // Process every pixel. Note that every channel returns a value in the range [0, 255]:
        int i = 0;
        for (int pixel = 0; pixel < pixelCount; pixel++) {
            for (int z = 0; z < channelCount; z++) {
                double value = TypeConversion.unsignedByteToInt(buffer[i]);
                mins[z] = Math.min(mins[z], value);
                maxes[z] = Math.max(maxes[z], value);
                i++;
            }
        }

        // Print statistics the first few times we're called:
        if (totalCount < 3) {
            System.out.printf("Ranges for %dx%dx%d bitmap:\n", mat.width(), mat.height(), mat.channels());
            for (int x = 0; x < mat.channels(); x++) {
                System.out.printf("  Channel %d: [%.2f, %.2f]\n", x, mins[x], maxes[x]);
            }
        }
    }

    // The system calls processFrame every time the camera acquires a new frame (typically 30
    // frames per second). Note that this is on a different thread than the robot's primary loop:
    @Override
    public Object processFrame(Mat largeRgb, long captureTimeNanos) {
        final int WIDTH = 320;
        final int HEIGHT = 240;

        // First resize the frame that came from the camera:
        Imgproc.resize(largeRgb, smallRgb, new org.opencv.core.Size(WIDTH, HEIGHT), 0, 0, Imgproc.INTER_AREA);

        // Convert it to YCrCb (Y is luminance/intensity, Cr is red chroma, Cb is blue chroma):
        Imgproc.cvtColor(smallRgb, smallYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Process the bitmap:
        inspect(smallYCrCb, totalCount);

        // Track some stats:
        totalCount++;
        frameCount++;
        if (time() - lastTime >= 1) {
            if (totalCount < 200) {
                lastTime = time();
                System.out.printf("%d frames of %.0fx%.0f with %d channels!\n",
                        frameCount, smallYCrCb.size().width, smallYCrCb.size().height, smallYCrCb.channels());
                frameCount = 0;
            }
        }
        return null;
    }

    // I think this is called when the system wants us to preview our processing:
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}