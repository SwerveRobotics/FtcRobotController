package org.firstinspires.ftc.team417_CENTERSTAGE.pathTraversal;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

//The control points of bezier curves.
public class Bezier {
    public DPoint[] controlPoints = new DPoint[4];
    public ArrayList<DPoint> linearPoints;

    public Bezier(DPoint p0, DPoint p1, DPoint p2, DPoint p3, double epsilon) {
        controlPoints[0] = p0;
        controlPoints[1] = p1;
        controlPoints[2] = p2;
        controlPoints[3] = p3;

        flatten(controlPoints, new ArrayList<>(), epsilon);
        linearPoints.add(0, controlPoints[0]);
    }

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

        linearPoints = resultPoints;

        return resultPoints;
    }

    public void graph(Canvas canvas) {
        for (int i = 0; i < linearPoints.size() - 1; i++)
            canvas.strokeLine(linearPoints.get(i).x, linearPoints.get(i).y,
                              linearPoints.get(i + 1).x, linearPoints.get(i + 1).y);
    }

    public void graph(Canvas canvas, DPoint currentPos, int currentLine) {
        canvas.strokeLine(currentPos.x, currentPos.y, linearPoints.get(currentLine).x, linearPoints.get(currentLine).y);

        for (int i = currentLine; i < linearPoints.size() - 2; i++)
            canvas.strokeLine(linearPoints.get(i).x, linearPoints.get(i).y,
                    linearPoints.get(i + 1).x, linearPoints.get(i + 1).y);
    }

    public double length(int startingPoint) {
        double distSum = 0;

        for (int i = startingPoint; i < linearPoints.size() - 1; i++) {
            distSum += Math.hypot(linearPoints.get(i).x - linearPoints.get(i + 1).x,
                                  linearPoints.get(i).y - linearPoints.get(i + 1).y);
        }

        return distSum;
    }

    public double length(int startingPoint, DPoint currentPos) {
        Vector2d currentVect = currentPos.toVector(linearPoints.get(startingPoint));
        return length(startingPoint) + currentVect.norm();
    }
}
