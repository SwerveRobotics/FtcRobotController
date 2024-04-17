package org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseWithID {
    public Pose2d pose;
    public int ID;
    public boolean trust;

    public PoseWithID(Pose2d pose, int ID, boolean trust) {
        this.pose = pose;
        this.ID = ID;
        this.trust = trust;
    }
}
