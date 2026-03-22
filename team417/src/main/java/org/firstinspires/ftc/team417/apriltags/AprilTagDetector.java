package org.firstinspires.ftc.team417.apriltags;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417.CompetitionAuto;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.Closeable;
import java.util.Comparator;
import java.util.List;

public class AprilTagDetector implements Closeable {
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /**
     * The variable for how long ago the detection last changed.
     */
    private double timeOfLastChange = System.nanoTime() / 1_000_000_000.0;

    /**
     * The variable for how many detections were and what the ID was in the last cycle.
     */
    private int lastDetectionsSize = -1;
    private int lastId = -1;

    /**
     * Initialize the AprilTag processor.
     */
    public AprilTagDetector(HardwareMap hardwareMap) {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "camera"), aprilTag);
    }

    /**
     * Default is no verbosity.
     */
    public Pattern detectPatternAndTelemeter(CompetitionAuto.Alliance alliance, Telemetry telemetry) {
        return detectPatternAndTelemeter(alliance, telemetry, false);
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public Pattern detectPatternAndTelemeter(CompetitionAuto.Alliance alliance, Telemetry telemetry, boolean verbose) {
        if (verbose) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

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
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }

        // Get the pattern:
        Pattern pattern = detectPattern(alliance);

        String patternDisplay;

        // The `\\u...` are escape sequences for green and purple circle emojis.
        // \uD83D\uDFE3 -> Purple circle
        // \uD83D\uDFE2 -> Green circle
        // \u26AA -> White circle
        switch (pattern) {
            case PPG:
                patternDisplay = "\uD83D\uDFE3\uD83D\uDFE3\uD83D\uDFE2";
                break;
            case PGP:
                patternDisplay = "\uD83D\uDFE3\uD83D\uDFE2\uD83D\uDFE3";
                break;
            case GPP:
                patternDisplay = "\uD83D\uDFE2\uD83D\uDFE3\uD83D\uDFE3";
                break;
            default:
                patternDisplay = "\u26AA\u26AA\u26AA";
                break;
        }

        double elapsedTime = timeOfLastChange - (System.nanoTime() / 1_000_000_000.0);

        // Summarize the most important detection info in one line:
        switch (alliance) {
            case RED:
                telemetry.addLine(String.format("%s (%d IDs, leftmost ID = %d for %f sec.)",
                        patternDisplay, lastDetectionsSize, lastId, timeOfLastChange));
                break;
            case BLUE:
                telemetry.addLine(String.format("%s (%d IDs, rightmost ID = %d for %f sec.)",
                        patternDisplay, lastDetectionsSize, lastId, timeOfLastChange));
                break;
        }

        return pattern;
    }   // end method telemetryAprilTag()

    /**
     * Detect the correct color pattern and return it.
     */
    public Pattern detectPattern(CompetitionAuto.Alliance alliance) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Remove all AprilTags that don't have ID 21, 22, or 23
        // (This is because the obelisk only has AprilTags with IDs 21, 22, and 23)
        // (The remaining IDs, 20 and 24, are for localization only)
        currentDetections.removeIf(detection ->
                detection.id != 21 && detection.id != 22 && detection.id != 23
        );

        // AprilTagDetection objects contain the x (right), y (forward), and z (up) distance
        //  relative to the robot. When we're on the red alliance, we want the leftmost valid
        //  AprilTag, and when we're on the blue alliance, we want the rightmost valid AprilTag.
        //  This is because, in our near position, we see two AprilTags on the obelisk: the front
        //  AprilTag and the side AprilTag closest to our color goal.
        // For more information about the info the AprilTagDetection object contains, see this link:
        //  https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
        AprilTagDetection detection = null;
        switch (alliance) {
            case RED:
                // Set detection to the leftmost (min x) detection relative to the robot
                // If there are no detections, set it to null
                detection = currentDetections.stream()
                        .min(Comparator.comparingDouble(aprilTagDetection -> aprilTagDetection.ftcPose.x)).orElse(null);
                break;
            case BLUE:
                // Set detection to the rightmost (max x) detection relative to the robot
                // If there are no detections, set it to null
                detection = currentDetections.stream()
                        .max(Comparator.comparingDouble(aprilTagDetection -> aprilTagDetection.ftcPose.x)).orElse(null);
        }

        if (detection == null) {
            lastId = -1;
            timeOfLastChange = System.nanoTime() / 1_000_000_000.0;
            return Pattern.UNKNOWN;
        }

        int currentDetectionsSize = currentDetections.size();

        if (lastDetectionsSize != currentDetectionsSize || detection.id != lastId) {
            timeOfLastChange = System.nanoTime() / 1_000_000_000.0;
        }

        lastDetectionsSize = currentDetectionsSize;
        lastId = detection.id;

        switch (detection.id) {
            case 21:
                return Pattern.GPP;
            case 22:
                return Pattern.PGP;
            case 23:
                return Pattern.PPG;
            default:
                return Pattern.UNKNOWN;
        }
    }

    /**
     * Release the resources taken up by the vision portal.
     */
    @Override
    public void close() {
        visionPortal.close();
    }
}
