package org.firstinspires.ftc.team417_CENTERSTAGE.apriltagsnew;

import static org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive.isDevBot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.AprilTagInfo;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.CameraInfo;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.InfoWithDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.PoseWithID;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedList;

@Config
public class AprilTagPoseEstimator {
    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public HardwareMap myHardwareMap;   // gain access to camera in hardwareMap

    public Telemetry telemetry;   // gain access to telemetry

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the status light.
     */
    public DigitalChannel statusLight;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    // Which camera we're using
    public CameraInfo camera;

    // List of poses
    public final LinkedList<PoseWithID> poseHistory = new LinkedList<>();

    // robotPoseEstimate is the pose estimate OF THE CURRENT CLASS
    Pose2d robotPoseEstimate = new Pose2d(0, 0, 0);

    public AprilTagPoseEstimator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.myHardwareMap = hardwareMap;
        this.telemetry = telemetry;
        init();
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void init() {
        camera = CameraInfoDump.camera("DevBotFastCamera");

        // Create the AprilTag processor.
        AprilTagProcessor.Builder aprilTagBuilder = new AprilTagProcessor.Builder();

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

        if (camera.lensIntrinsics != null) {
            aprilTagBuilder.setLensIntrinsics(camera.lensIntrinsics.fx, camera.lensIntrinsics.fy, camera.lensIntrinsics.cx, camera.lensIntrinsics.cy);
        }

        aprilTag = aprilTagBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        // Changed from 3 (default) to 1
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(myHardwareMap.get(WebcamName.class, camera.robotName));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Sets the status light for latency testing, etc.
        if (isDevBot) {
            statusLight = myHardwareMap.get(DigitalChannel.class, "green");
            statusLight.setMode(DigitalChannel.Mode.OUTPUT);
        }

        if (statusLight != null) {
            statusLight.setState(false);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        // Used max resolution
        builder.setCameraResolution(camera.resolution);

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
    public Pose2d calculatePoseEstimate(AprilTagDetection detection, AprilTagInfo aprilTagInfo, CameraInfo cameraInfo) {
        // See November notebook 11/20/2023 for more info on the math used here

        // Declaring variables
        double adjustedCameraInfoX, adjustedCameraInfoY, d, beta, gamma, relativeX, relativeY, absoluteX, absoluteY, absoluteTheta;

        System.out.println("Start");

        // adjustedCameraInfoX and adjustedCameraInfoY - camera offset after being adjusted for camera rotation
        adjustedCameraInfoX = cameraInfo.offset.x * Math.cos(Math.toRadians(cameraInfo.rotation)) - cameraInfo.offset.y * Math.sin(Math.toRadians(cameraInfo.rotation));
        adjustedCameraInfoY = cameraInfo.offset.x * Math.sin(Math.toRadians(cameraInfo.rotation)) + cameraInfo.offset.y * Math.cos(Math.toRadians(cameraInfo.rotation));

        // d - absolute distance from April-tag to robot
        d = Math.hypot(detection.ftcPose.x + adjustedCameraInfoX, detection.ftcPose.y + adjustedCameraInfoY);

        System.out.println("d" + d);

        // gamma - angle of center of camera direction to april tag direction
        gamma = Math.atan2(detection.ftcPose.x + adjustedCameraInfoX, detection.ftcPose.y + adjustedCameraInfoY);

        System.out.println("gamma" + gamma);

        // beta - yaw of robot relative to the tag
        beta = gamma + Math.toRadians(detection.ftcPose.yaw);

        System.out.println("detection yaw" + detection.ftcPose.yaw);

        System.out.println("beta" + beta);

        // relativeX - x of robot without compensating for yaw
        relativeX = d * Math.cos(beta) + aprilTagInfo.x;

        System.out.println(relativeX);

        // relativeY - y of robot without compensating for yaw
        relativeY = -d * Math.sin(beta) + aprilTagInfo.y;

        System.out.println(relativeY);

        // absoluteX - x of robot
        absoluteX = (relativeX - aprilTagInfo.x) * Math.cos(Math.toRadians(aprilTagInfo.yaw)) - (relativeY - aprilTagInfo.y) * Math.sin(Math.toRadians(aprilTagInfo.yaw)) + aprilTagInfo.x;

        System.out.println(absoluteX);

        // absoluteY - y of robot
        absoluteY = (relativeX - aprilTagInfo.x) * Math.sin(Math.toRadians(aprilTagInfo.yaw)) + (relativeY - aprilTagInfo.y) * Math.cos(Math.toRadians(aprilTagInfo.yaw)) + aprilTagInfo.y;

        System.out.println(absoluteY);

        // absoluteTheta - yaw of robot
        absoluteTheta = Math.toRadians(aprilTagInfo.yaw - detection.ftcPose.yaw + cameraInfo.rotation) + Math.PI;

        System.out.println(absoluteTheta);

        return new Pose2d(absoluteX, absoluteY, absoluteTheta);
    }

    // Draws the last 100 poses as circles, squares, or lines with different colors
    // Depends on the April Tag ID and the level of trust
    public void drawPoseHistory(Canvas c, double radius) {

        for (PoseWithID t : poseHistory) {
            if (t.trust) {
                c.setStroke("#00ff00");
            } else {
                c.setStroke("#ff0000");
            }
            switch (t.ID) {
                case 3:
                    c.strokeCircle(t.pose.position.x, t.pose.position.y, radius);
                    break;
                case 2:
                    c.strokeRect(t.pose.position.x - radius, t.pose.position.y - radius, radius, radius);
                    break;
                default:
                    c.strokeLine(t.pose.position.x - 0.5 * radius, t.pose.position.y - 0.5 * radius, t.pose.position.x + 0.5 * radius, t.pose.position.y + 0.5 * radius);
                    break;
            }
        }
    }

    public void drawPoseHistory(Canvas c) {
        drawPoseHistory(c, 4);
    }

    /**
     * Decides if (a) detection(s) is/are trustworthy and if to update robotPoseEstimate based on it/them
     */

    public Pose2d trustVerification(ArrayList<Pose2d> poseArray) {
        // Takes the average pose of the detected poses

        double sumX = 0;double sumY = 0; double sumReal= 0; double sum Imag = 0;

        for (pose : poseArray) {
            sumX += 10;
        }

        // TODO: Remove the magic constants
        // If any pose is more than 4 inches away or 5 degrees apart, distrust all poses
        boolean distanceIsSmall = poseArray.stream()
                .allMatch(pose ->
                        Math.hypot(pose.position.x - averagePose.position.x,
                                pose.position.y - averagePose.position.y) < 8);
        boolean thetaDifferenceIsSmall = poseArray.stream()
                .allMatch(pose ->
                        Math.abs(pose.heading.log() - averagePose.heading.log()) < 5);
        if (!(distanceIsSmall && thetaDifferenceIsSmall)) return null;
        return averagePose;
    }

    ArrayList<InfoWithDetection> knownAprilTagsDetected = new ArrayList<>();

    /**
     * Produce a pose estimate from current frames and update it
     */
    @SuppressLint("DefaultLocale")
    public void updatePoseEstimate() {
        // Use detections only if fresh
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();

        if (currentDetections == null) return;

        // Iterates through detections and finds any the the robot "knows"
        // Then it replaces the pose estimate with pose estimate from that april tag
        knownAprilTagsDetected.clear();

        for (AprilTagDetection detection : currentDetections) {
            AprilTagInfo aprilTagInfo = AprilTagInfoDump.findTagWithId(detection.id);
            if (aprilTagInfo != null) {
                knownAprilTagsDetected.add(new InfoWithDetection(aprilTagInfo, detection));
            }
        }

        boolean detecting;
        Pose2d poseEstimate;
        ArrayList<Pose2d> poseArray = new ArrayList<>();
        if (knownAprilTagsDetected.size() > 0) {
            for (InfoWithDetection iwd : knownAprilTagsDetected) {
                poseEstimate = calculatePoseEstimate(iwd.detection, iwd.info, camera);
                poseArray.add(poseEstimate);
            }
            robotPoseEstimate = trustVerification(poseArray);
            for (int i = 0; i < knownAprilTagsDetected.size(); i++) {
                poseHistory.add(new PoseWithID(poseArray.get(i),
                        knownAprilTagsDetected.get(i).info.id,
                        robotPoseEstimate != null));
            }
            detecting = true;
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }
        } else {
            robotPoseEstimate = null;
            detecting = false;
        }

        // setState is reversed (true = off, false = on) so the extra ! in front is necessary
        // Cannot fix this, SDK problem
        if (statusLight != null) {
            statusLight.setState(!detecting);
        }

        // Telemeters the current pose estimate
        if (telemetry != null && robotPoseEstimate != null) {
            telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)", robotPoseEstimate.position.x, robotPoseEstimate.position.y, Math.toDegrees(robotPoseEstimate.heading.log())));
        }
    } // end method telemetryAprilTag()
}
