package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

public class DistanceLocalizer {
    public UltrasonicDistanceSensor leftDistance, rightDistance;
    public DistanceSensorInfo leftInfo, rightInfo;
    public Vector2d correction;
    public MecanumDrive drive;

    public DistanceLocalizer(MecanumDrive drive) {
        this.drive = drive;
        this.correction = new Vector2d(0, 0);
    }

    public void updateIfPossible() {

    }

    public Pose2d correct(Pose2d pose) {
        return new Pose2d(
                pose.position.x + correction.x,
                pose.position.y + correction.y,
                pose.heading.log());
    }
}
