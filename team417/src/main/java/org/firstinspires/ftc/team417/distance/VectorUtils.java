package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class VectorUtils {
    public static Vector2d rotate(Vector2d vector, double theta) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        // Applying the rotation matrix
        double rotatedX = vector.x * cosTheta - vector.y * sinTheta;
        double rotatedY = vector.x * sinTheta + vector.y * cosTheta;

        return new Vector2d(rotatedX, rotatedY);
    }

    public static Vector2d calculateAverage(ArrayList<Vector2d> vectors) {
        if (vectors == null || vectors.isEmpty()) {
            return new Vector2d(0, 0);
        }

        double sumX = 0;
        double sumY = 0;

        for (Vector2d vector : vectors) {
            sumX += vector.x;
            sumY += vector.y;
        }

        int count = vectors.size();
        return new Vector2d(sumX / count, sumY / count);
    }
}
