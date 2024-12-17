package org.firstinspires.ftc.team417;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417.liveView.LiveViewProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {

    Mat testImage; // Our canonical test image

    // Do one-time initialization code:
    void initializeTest() {
        testImage = Imgcodecs.imread("/sdcard/live_view.jpg");
        if ((testImage.size().width != 0) && (testImage.size().height != 0)) {
            System.out.printf("Test image successfully loaded!\n");
        } else {
            System.out.printf("ERROR: Couldn't find the test image in the robot's storage.\n");
        }

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Specify the camera's name as set in the Robot Configuration:
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera")); // "webcam"

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the system's built-in RC preview (LiveView).  Set "false" to omit camera
        // monitoring.
        builder.enableLiveView(true);

        // Set and enable the live-view processor.
        builder.addProcessor(new LiveViewProcessor());

        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();
    }
}
