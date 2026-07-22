/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.team417.apriltags;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team417.CompetitionAuto;
import org.firstinspires.ftc.team417.TransferState;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * This class is used to detect AprilTags using the Limelight3A Vision Sensor.
 * It is currently used for our pose updating using the april tag.
 */
public class LimelightAprilTagDetector implements Closeable {
    /**
     * The variable to store our instance of the Mecanum drive.
     */
    private final MecanumDrive drive;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private final Limelight3A limelight;

    /**
     * The variable for how long ago the detection last changed.
     */
    private double timeOfLastChange;

    /**
     * The variable for how many detections were and what the ID was in the last cycle.
     */
    private int lastDetectionsSize = -1;
    private int lastId = -1;

    /**
     * Whether to do pose correction or not.
     */
    public boolean poseCorrectEnabled;

    /**
     * The constant for how far away for a correction pose to be at max to be considered valid.
     */
    private final double CORRECTION_RANGE = 6;

    /**
     * Variables for capturing the details of the last correction.
     */
    public volatile double lastXDistance = 0;
    public volatile double lastYDistance = 0;
    public volatile boolean lastWithinRange = false;

    /**
     * Initialize the AprilTag processor.
     */
    public LimelightAprilTagDetector(HardwareMap hardwareMap, MecanumDrive drive) {
        // Create the AprilTag processor.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(7);

        limelight.start();

        this.drive = drive;
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
    @SuppressLint("DefaultLocale")
    public Pattern detectPatternAndTelemeter(CompetitionAuto.Alliance alliance, Telemetry telemetry, boolean verbose) {
        LLResult result = limelight.getLatestResult();

        if (verbose) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if (result != null && result.isValid()) {
                // Access general information
                Pose3D botPose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botPose.toString());


                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                telemetry.addData("# AprilTags Detected", fiducialResults.size());

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("AprilTags", "ID: %d, Family: %s, X: %.2f, Y: %.2f, Pose: %s", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getRobotPoseFieldSpace());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }

        // Get the pattern:
        Pattern pattern = detectPattern(alliance, result);

        String patternDisplay;

        // The `\\u...` are escape sequences for green and purple circle emojis.
        // \uD83D\uDFE3 -> Purple circle
        // \uD83D\uDFE2 -> Green circle
        // ⚪ -> White circle
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
                patternDisplay = "⚪⚪⚪";
                break;
        }

        double elapsedTime = (System.currentTimeMillis() / 1000.0) - timeOfLastChange;

        // Summarize the most important detection info in one line:
        switch (alliance) {
            case RED:
                telemetry.addLine(String.format("%s (%d IDs, leftmost ID = %d for %f sec.)",
                        patternDisplay, lastDetectionsSize, lastId, elapsedTime));
                break;
            case BLUE:
                telemetry.addLine(String.format("%s (%d IDs, rightmost ID = %d for %f sec.)",
                        patternDisplay, lastDetectionsSize, lastId, elapsedTime));
                break;
        }

        return pattern;
    }   // end method telemetryAprilTag()

    /**
     * We want the Limelight to get its own result as default.
     */
    public Pattern detectPattern(CompetitionAuto.Alliance alliance) {
        return  detectPattern(alliance, null);
    }

    /**
     * Detect the correct color pattern and return it.
     */
    public Pattern detectPattern(CompetitionAuto.Alliance alliance, LLResult result) {
        if (result == null)
            result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> currentDetections = new ArrayList<>();

        if (result != null && result.isValid())
            currentDetections = result.getFiducialResults();

        // Remove all AprilTags that don't have ID 21, 22, or 23
        // (This is because the obelisk only has AprilTags with IDs 21, 22, and 23)
        // (The remaining IDs, 20 and 24, are for localization only)
        currentDetections.removeIf(detection ->
                detection.getFiducialId() != 21
                        && detection.getFiducialId() != 22
                        && detection.getFiducialId() != 23
        );

        // FiducialResult objects contain the x (left) and y (up) degrees relative to the robot.
        //  When we're on the red alliance, we want the leftmost valid
        //  AprilTag, and when we're on the blue alliance, we want the rightmost valid AprilTag.
        //  This is because, in our near position, we see two AprilTags on the obelisk: the front
        //  AprilTag and the side AprilTag closest to our color goal.
        // For more information about the info the AprilTagDetection object contains, see this link:
        //  https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html
        LLResultTypes.FiducialResult detection = null;
        switch (alliance) {
            case RED:
                // Set detection to the leftmost (min x degrees) detection relative to the robot
                // If there are no detections, set it to null
                detection = currentDetections.stream()
                        .min(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetXDegrees)).orElse(null);
                break;
            case BLUE:
                // Set detection to the rightmost (max x degrees) detection relative to the robot
                // If there are no detections, set it to null
                detection = currentDetections.stream()
                        .max(Comparator.comparingDouble(LLResultTypes.FiducialResult::getTargetXDegrees)).orElse(null);
        }

        if (detection == null) {
            lastId = -1;
            timeOfLastChange = System.currentTimeMillis() / 1_000.0;
            return Pattern.UNKNOWN;
        }

        int currentDetectionsSize = currentDetections.size();

        if (lastDetectionsSize != currentDetectionsSize || detection.getFiducialId() != lastId) {
            timeOfLastChange = System.currentTimeMillis() / 1_000.0;
        }

        lastDetectionsSize = currentDetectionsSize;
        lastId = detection.getFiducialId();

        switch (detection.getFiducialId()) {
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
     * Detect the pose of the robot with the AprilTag.
     */
    public Pose2d detectRobotPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose_MT2();

            return new Pose2d(
                    pose.getPosition().x * 39.37, // Convert meters to inches
                    pose.getPosition().y * 39.37, // Convert meters to inches
                    pose.getOrientation().getYaw(AngleUnit.RADIANS));
        }

        return null;
    }

    /**
     * Feed in the yaw from the IMU for MT2.
     */
    public void updateRobotYaw(double yaw) {
        limelight.updateRobotOrientation(Math.toDegrees(yaw));
    }

    // Resets the robot pose only if the robot is not moving and the new pose is within a certain
    //  distance from the old.
    @SuppressLint("DefaultLocale")
    public void tryResetRobotPose(Telemetry telemetry) {
        boolean notMoving = isZero(drive.poseVelocity.linearVel.x)
                && isZero(drive.poseVelocity.linearVel.y);

        if (notMoving && poseCorrectEnabled && TransferState.trustPose) {

            Pose2d pose = detectRobotPose();

            if (pose != null) {
                boolean closeEnough = Math.pow(pose.position.x - drive.pose.position.x, 2)
                        + Math.pow(pose.position.y - drive.pose.position.y, 2)
                        <= Math.pow(CORRECTION_RANGE, 2);

                lastWithinRange = closeEnough;
                lastXDistance = pose.position.x - drive.pose.position.x;
                lastYDistance = pose.position.y - drive.pose.position.y;

                telemetry.log().add(String.format("Last pose correction %s (%.2f\", %.2f\")",
                        lastWithinRange ? "✅" : "❌", lastXDistance, lastYDistance));

                if (closeEnough) {
                    drive.setPose(pose);
                }
            }
        }
    }

    // Sees if a number is within one one-hundredths of zero
    private static boolean isZero(double z) {
        return Math.abs(z) < 0.01;
    }

    /**
     * Release the resources taken up by the vision portal.
     */
    @Override
    public void close() {
        limelight.stop();
    }
}
