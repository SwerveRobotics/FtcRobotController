package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

//points as doubles
public class DPoint {
    public double x;
    public double y;

    public DPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public DPoint plus(DPoint p1, DPoint p2) {
        return new DPoint(p1.x + p2.x, p1.y + p2.y);
    }

    public DPoint plus(DPoint p1) {
        return new DPoint(x + p1.x, y + p1.y);
    }

    public DPoint times(DPoint p, double num) {
        return new DPoint(p.x * num, p.y * num);
    }

    public DPoint times(double num) {
        return new DPoint(x * num, y * num);
    }
}
