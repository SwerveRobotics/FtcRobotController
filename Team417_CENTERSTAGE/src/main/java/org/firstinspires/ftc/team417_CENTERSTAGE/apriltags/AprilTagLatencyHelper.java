package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.TwistWithTimestamp;

import java.util.ArrayList;

public class AprilTagLatencyHelper {
    // robotPoseEstimate is the pose estimate OF THE CURRENT CLASS
    public Pose2d robotPoseEstimate;

    // To keep a record of twists to be used by April Tag latency compensation 
    public ArrayList<TwistWithTimestamp> twistList;

    // For the April Tag latency calculation
    public ElapsedTime clock = new ElapsedTime();

    public double fps = 1;

    // Latency between april tag shown and detection in ms
    public static double CAMERA_LATENCY = 640;

    public AprilTagLatencyHelper() {
        // To keep a record of twists from MecanumDrive
        twistList = new ArrayList<>();
    }

    public void updateFPS(double fps) {
        this.fps = fps;
    }

    public void addTwist(Twist2dDual<Time> twist) {
        twistList.add(0, new TwistWithTimestamp(twist, clock.milliseconds()));

        // Keep only twists from less than five seconds ago
        if (twistList.size() > 0) {
            TwistWithTimestamp oldestTwist = twistList.get(twistList.size() - 1);
            double currentTime = clock.milliseconds();
            while (oldestTwist.timestamp < currentTime - (CAMERA_LATENCY
                    + (1000 / fps) + 200)) {
                twistList.remove(oldestTwist);
                oldestTwist = twistList.get(twistList.size() - 1);
                currentTime = clock.milliseconds();
            }
        }
    }

    public void updateMecanumDrive(Pose2d poseEstimate) {
        robotPoseEstimate = poseEstimate;

        // Latency compensation code
        double currentTime = clock.milliseconds();

        // Get the first twist to be added the robot's position
        TwistWithTimestamp lastTwist = twistList.get(0);

        // Add all the twists since the April Tag frame to now to the robot pose
        for (int i = 1; lastTwist.timestamp >= currentTime -
                CAMERA_LATENCY
                && i < twistList.size(); i++) {
            robotPoseEstimate.plus(lastTwist.twist.value());
            lastTwist = twistList.get(i);
            currentTime = clock.milliseconds();
        }
    }

    public Pose2d refinePose() {
        return robotPoseEstimate;
    }
}
