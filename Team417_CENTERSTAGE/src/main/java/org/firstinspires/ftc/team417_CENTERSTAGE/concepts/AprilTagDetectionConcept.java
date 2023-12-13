package org.firstinspires.ftc.team417_CENTERSTAGE.concepts;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseTeleOp;
import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagInfoDump;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.AprilTagInfo;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Pose;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/*
    Concept: this is for testing the April-Tag concept.
*/

@Disabled
@TeleOp(name = "Concept: AprilTag")
public class AprilTagDetectionConcept extends BaseTeleOp {

    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    // For this concept: supposed location of the robot
    Pose robotPoseEstimate = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {
        initializeHardware();
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driveUsingControllers(true);
                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU. (not)
                //sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    // Calculates the pose estimate based on a detection of an april tag
    //     and the stored information of said april tag (position etc.)
    // Also telemeters relevant info
    // Note that this is relative to the center of the camera
    public Pose calculatePoseEstimate(AprilTagDetection detection, AprilTagInfo aprilTagInfo) {
        double d, beta, gamma, relativeX, relativeY, absoluteX, absoluteY, absoluteTheta;

        d = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
        telemetry.addData("d", d);

        gamma = Math.atan2(detection.ftcPose.x, detection.ftcPose.y);
        telemetry.addData("gamma", gamma);

        beta = gamma + Math.toRadians(detection.ftcPose.yaw); //(or gamma - detection.ftcPose.yaw) if that doesn't work)
        telemetry.addData("beta", beta);

        relativeX = d * Math.cos(beta) + aprilTagInfo.x;
        telemetry.addData("relativeX", relativeX);

        relativeY = -d * Math.sin(beta) + aprilTagInfo.y;
        telemetry.addData("relativeY", relativeY);

        absoluteX = relativeX * Math.cos(Math.toRadians(aprilTagInfo.yaw)) - relativeY * Math.sin(Math.toRadians(aprilTagInfo.yaw));
        telemetry.addData("absoluteX", absoluteX);

        absoluteY = relativeX * Math.sin(Math.toRadians(aprilTagInfo.yaw)) + relativeY * Math.cos(Math.toRadians(aprilTagInfo.yaw));
        telemetry.addData("absoluteY", absoluteY);

        absoluteTheta = Math.toRadians(aprilTagInfo.yaw) - Math.toRadians(detection.ftcPose.yaw) + Math.PI;
        telemetry.addData("absoluteTheta", absoluteTheta);

        Pose pose = new Pose(absoluteX, absoluteY, absoluteTheta);

        return pose;
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double x = 0;
        double y = 0;
        double theta = 0;

        // Iterates through detections and finds any the the robot "knows"
        // Then it replaces the pose estimate with pose estimate from that april tag
        for (AprilTagDetection detection : currentDetections) {
            AprilTagInfo aprilTagInfo = AprilTagInfoDump.findTagWithId(detection.id);
            if (aprilTagInfo != null) {
                x = detection.ftcPose.x;
                y = detection.ftcPose.y;
                theta = detection.ftcPose.yaw;
                robotPoseEstimate = calculatePoseEstimate(detection, aprilTagInfo);
                break;
            }
        }

        // Telemeters the current pose estimate
        telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)", robotPoseEstimate.x, robotPoseEstimate.y, Math.toDegrees(robotPoseEstimate.theta)));

        telemetry.addData("\n# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & yaw) (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        // Telemeters the pose info to FTC dashboard so that it draws the robot pose
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        c.setStroke("#3F51B5");
        MecanumDrive.drawRobot(c, new Pose2d(robotPoseEstimate.x, robotPoseEstimate.y, robotPoseEstimate.theta));
        c.setStroke("#3F5100");
        MecanumDrive.drawRobot(c, new Pose2d(x, y, theta));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(p);

    }   // end method telemetryAprilTag()

}   // end class

