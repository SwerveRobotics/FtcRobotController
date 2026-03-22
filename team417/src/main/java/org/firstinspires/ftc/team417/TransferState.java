package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Pose2d;

// Stores the values that need to be transferred from auto to teleop
public class TransferState {
    public static CompetitionAuto.Alliance chosenAlliance;
    public static PixelColor[] storedColors;
    public static Pose2d pose;
    public static boolean trustPose = false;
    public static boolean usePoseCorrection = true;
}
