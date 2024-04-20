package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import com.acmerobotics.roadrunner.Vector2d;

//points as doubles
public class DPoint {
    public double x;
    public double y;

    public DPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d toVector(DPoint point2) {
        return new Vector2d(x - point2.x, y - point2.y);
    }
    public Vector2d toVector(double x2, double y2) {
        return toVector(new DPoint(x2, y2));
    }

    public DPoint plus(double num) {
        return new DPoint(x + num, y + num);
    }

    public DPoint plus(Vector2d num) {
        return new DPoint(x + num.x, y + num.y);
    }
}
