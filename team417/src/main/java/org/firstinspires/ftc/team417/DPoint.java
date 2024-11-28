package org.firstinspires.ftc.team417;

import com.acmerobotics.roadrunner.Vector2d;

public class DPoint {
    public double x;
    public double y;

    public DPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public DPoint plus(Vector2d vel) {
        return new DPoint(x + vel.x, y + vel.y);
    }

    public DPoint minus(Vector2d vel) {
        return new DPoint(x - vel.x, y - vel.y);
    }

    public DPoint minus(DPoint point) {
        return new DPoint(x - point.x, y - point.y);
    }

    public Vector2d toVector2d() {
        return new Vector2d(x, y);
    }

    public static DPoint to(Vector2d point) {
        return new DPoint(point.x, point.y);
    }
}
