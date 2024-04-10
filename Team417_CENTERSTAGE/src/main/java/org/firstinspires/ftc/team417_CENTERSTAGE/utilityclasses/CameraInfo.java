package org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses;

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;

// Stores data about April Tags
public class CameraInfo {
    //Initializing variables
    public String realName; // What we colloquially call the robot
    public String robotName; // The name in config
    public Vector2d offset; // Inches
    public double rotation; // Degrees
    public Size resolution;
    public LensIntrinsics lensIntrinsics;
    public double latency; // Milliseconds
    public double fov; // Degrees

    // Creates an april tag with all the info
    public CameraInfo(String realName, String robotName, Vector2d offset, double rotation, Size resolution, LensIntrinsics lensIntrinsics, double latency, double fov) {
        this.realName = realName;
        this.robotName = robotName;
        this.offset = offset;
        this.rotation = rotation;
        this.resolution = resolution;
        this.lensIntrinsics = lensIntrinsics;
        this.latency = latency;
        this.fov = fov;
    }
}
