package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;

import java.util.ArrayList;

//The control points of bezier curves.
public class Bezier {
    public DPoint[] controlPoints = new DPoint[4];

    public Bezier(DPoint p0, DPoint p1, DPoint p2, DPoint p3) {
        controlPoints[0] = p0;
        controlPoints[1] = p1;
        controlPoints[2] = p2;
        controlPoints[3] = p3;
    }

    //returns the midpoints between and array of points as an array of points.
    private DPoint[] findMidPoint(DPoint[] points) {
        DPoint[] midPoints = new DPoint[points.length - 1];

        for (int i = 0; i < points.length - 1; i++) {
            midPoints[i] = new DPoint((points[i].x + points[i + 1].x) / 2.0, (points[i].y + points[i + 1].y) / 2.0);
        }

        return midPoints;
    }

    //Turns an bezier curve into an array of smaller bezier curves that resemble linear vectors.
    //epsilon = the acceptable differance between a bezier curve and a linear vector in inches.
    private ArrayList<DPoint> flatten(DPoint[] p, ArrayList<DPoint> resultPoints, double epsilon) {
        DPoint[] l = findMidPoint(p);
        DPoint[] q = findMidPoint(l);
        DPoint c = findMidPoint(q)[0];

        double error = Math.max(Math.hypot(2 * p[1].x - p[2].x - p[0].x, 2 * p[1].y - p[2].y - p[0].y),
                Math.hypot(2 * p[2].x - p[3].x - p[1].x, 2 * p[2].y - p[3].y - p[1].y));

        if (error < epsilon) {
            resultPoints.add(p[3]);
        } else {
            resultPoints = flatten(new DPoint[] {p[0], l[0], q[0], c}, resultPoints, epsilon);
            resultPoints = flatten(new DPoint[] {c, q[1], l[2], p[3]}, resultPoints, epsilon);
        }

        return resultPoints;
    }

   public ArrayList<DPoint> lineApproximation(double epsilon) {
        return flatten(controlPoints, new ArrayList<>(), epsilon);
   }
}
