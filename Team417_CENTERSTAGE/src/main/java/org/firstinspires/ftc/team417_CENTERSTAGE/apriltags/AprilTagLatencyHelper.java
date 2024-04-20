package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Timestamped;

import java.util.ArrayList;

public class AprilTagLatencyHelper {
    // robotPoseEstimate is the pose estimate OF THE CURRENT CLASS
    public Pose2d robotPoseEstimate;

    // To keep a record of twists to be used by April Tag latency compensation 
    public ArrayList<Timestamped<Twist2d>> twistList;
    
    // To keep a record of poses to be used by latency compensation
    public ArrayList<Timestamped<Pose2d>> poseList;

    // For the April Tag latency calculation
    public ElapsedTime clock = new ElapsedTime();

    double fps = 1;

    // If the frame is new to MecanumDrive or not
    boolean newResult = false;

    // Latency between april tag shown and detection in ms
    public static double CAMERA_LATENCY = 640;

    public AprilTagLatencyHelper() {
        // To keep a record of twists from MecanumDrive
        twistList = new ArrayList<>();
        poseList = new ArrayList<>();
    }

    public void updateFPS(double fps) {
        this.fps = fps;
    }

    public void addTwist(Twist2dDual<Time> twist) {
        Twist2d twistValue = twist.value();
        
        twistList.add(0, new Timestamped<Twist2d>(twistValue, clock.milliseconds()));

        // Keep only twists from camera latency + spf ago
        if (twistList.size() > 0) {
            Timestamped<Twist2d> oldestTwist = twistList.get(twistList.size() - 1);
            double currentTime = clock.milliseconds();
            while (oldestTwist.timestamp < currentTime - (CAMERA_LATENCY
                    + (1000 / fps) + 100)) {
                twistList.remove(oldestTwist);
                oldestTwist = twistList.get(twistList.size() - 1);
                currentTime = clock.milliseconds();
            }
        }
    }

    public void addPose(Pose2d pose) {
        Pose2d copy = new Pose2d(pose.position.x, pose.position.y, pose.heading.log());

        poseList.add(0, new Timestamped<Pose2d>(copy, clock.milliseconds()));

        // Keep only twists from camera latency + spf ago
        if (poseList.size() > 0) {
            Timestamped<Pose2d> oldestPose = poseList.get(poseList.size() - 1);
            double currentTime = clock.milliseconds();
            while (oldestPose.timestamp < currentTime - (CAMERA_LATENCY
                    + (1000 / fps) + 100)) {
                poseList.remove(oldestPose);
                oldestPose = poseList.get(poseList.size() - 1);
                currentTime = clock.milliseconds();
            }
        }
    }

    // Did was robotPoseEstimate null last April Tag loop?
    boolean wasNullLastLoop = true;
    
    // Changes robotPoseEstimate to be exported by refinePose()
    public void updateMecanumDrive(Pose2d poseEstimate) {
        robotPoseEstimate = poseEstimate;

        if (robotPoseEstimate != null) {
            wasNullLastLoop = false;
            newResult = true;

            // Latency compensation code
            double currentTime = clock.milliseconds();

            // Get the first twist to be added the robot's position
            Timestamped<Twist2d> lastTwist = twistList.get(0);

            // Add all the twists since the April Tag frame to now to the robot pose
            for (int i = 1; lastTwist.timestamp >= currentTime -
                    CAMERA_LATENCY
                    && i < twistList.size(); i++) {
                robotPoseEstimate.plus(lastTwist.object);
                lastTwist = twistList.get(i);
                currentTime = clock.milliseconds();

            }
            addPose(robotPoseEstimate);
        } else if (!wasNullLastLoop) {
            wasNullLastLoop = true;
            /*
            wasNullLastLoop = true;
            newResult = true;
            
            // Latency compensation code
            double currentTime = clock.milliseconds();

            // Finds the pose from CAMERA_LATENCY ms ago and sets robotPoseEstimate to it
            Timestamped<Pose2d> lastPose = poseList.get(0);

            for (int i = 1; lastPose.timestamp > currentTime -
                    CAMERA_LATENCY
                    && i < twistList.size(); i++) {
                lastPose = poseList.get(i);
            }

            robotPoseEstimate = lastPose.object;
                
            // Get the first twist to be added the robot's position
            Timestamped<Twist2d> lastTwist = twistList.get(0);

            // Add all the twists since the April Tag frame to now to the robot pose
            for (int i = 1; lastTwist.timestamp >= currentTime -
                    CAMERA_LATENCY
                    && i < twistList.size(); i++) {
                robotPoseEstimate.plus(lastTwist.object);
                lastTwist = twistList.get(i);
                currentTime = clock.milliseconds();
            }
            */
        }
    }

    public Pose2d refinePose() {
        if (newResult) {
            newResult = false;
            return robotPoseEstimate;
        } else {
            return null;
        }
    }
}
