package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

//The control points of bezier curves.
public class BezierControl {
    public DPoint p0;
    public DPoint p1;
    public DPoint p2;
    public DPoint p3;

    public BezierControl(DPoint p0, DPoint p1, DPoint p2, DPoint p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
    }

    public DPoint[] toArray() {
        return new DPoint[] {p0, p1, p2, p3};
    }
}
