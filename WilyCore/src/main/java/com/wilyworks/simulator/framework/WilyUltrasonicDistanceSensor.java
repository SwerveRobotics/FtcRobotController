package com.wilyworks.simulator.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Point;

import java.util.Random;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

class Segment {
    public Point p1, p2;
    public Segment(double x1, double y1, double x2, double y2) {
        p1 = new Point(x1, y1);
        p2 = new Point(x2, y2);
    }
}

class Ray {
    public Point origin, direction;
    public Ray(Point origin, Point direction) {
        this.origin = origin;
        this.direction = direction;
    }
}

public class WilyUltrasonicDistanceSensor extends UltrasonicDistanceSensor {
    final double DETECTION_ANGLE = Math.toRadians(80); // 80 degree spread according to the specs
    final double MAX_DISTANCE = 75; // 150 inches is the theoretical maximum range

    static final Segment[] WALL_SEGMENTS = {
            // Clockwise starting in the upper-right corner:
            new Segment(72, 72, 72, -72),
            new Segment(72, -72, -72, -72),
            new Segment(-72, -72, -72, 72),
            new Segment(-72, 72, 72, 72),
    };
    Random random = new Random();

    WilyWorks.Config.DistanceSensor descriptor;

    public WilyUltrasonicDistanceSensor(String deviceName) {
        super(null, false);

        for (WilyWorks.Config.DistanceSensor distance: WilyCore.config.distanceSensors) {
            if (distance.name.equals(deviceName)) {
                descriptor = distance;
            }
        }
        if ((descriptor == null) && (WilyCore.config.distanceSensors.length > 0)) {
            throw new IllegalArgumentException(String.format("Couldn't find '%s' in WilyConfig.distanceSensors.", deviceName));
        }
    }


    // A method to check if a ray intersects a line segment and return the intersection point or null
    // Courtesy of Copilot.
    public Point raySegmentIntersection(Ray ray, Segment segment) {
        // Get the vectors of the ray and the segment
        Point r = ray.direction;
        Point s = segment.p2.subtract(segment.p1);

        // Solve the equation: ray.origin + t * r = segment.p1 + u * s
        // If a solution exists, the ray and the segment intersect
        double denominator = r.cross(s);
        // If the denominator is zero, the ray and the segment are parallel
        if (denominator == 0) {
            return null;
        }
        // Otherwise, find the values of t and u
        Point q = segment.p1.subtract(ray.origin);
        double t = q.cross(s) / denominator;
        double u = q.cross(r) / denominator;
        // The ray and the segment intersect if 0 <= t and 0 <= u <= 1
        if (t >= 0 && u >= 0 && u <= 1) {
            // The intersection point is ray.origin + t * r
            return new Point(ray.origin.x + t * r.x, ray.origin.y + t * r.y);
        }
        // Otherwise, there is no intersection
        return null;
    }

    @Override
    public double getDistance(DistanceUnit distanceUnit) {
        if (descriptor == null)
            return 0;

        // Distance sensors reflect the true pose:
        Pose2d truePose = WilyCore.getPose(0, true); // Assume zero latency
        Point sensorPoint = new Point(descriptor.x, descriptor.y)
                .rotate(truePose.heading.log())
                .add(new Point(truePose.position));
        double sensorAngle = descriptor.orientation + truePose.heading.log();

        Point sensorDirection = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
        Ray sensorRay = new Ray(sensorPoint, sensorDirection);

        // Find the first wall segment that the sensor is pointing straight at, according to
        // the current pose:
        double distance = 0;
        for (Segment segment : WALL_SEGMENTS) {
            // Add this sensor and segment to the eligible list if the sensor points at it
            // and the distance is less than MAX_DISTANCE:
            Point hitPoint = raySegmentIntersection(sensorRay, segment);
            if (hitPoint != null) {
                double hitPointDistance = sensorPoint.distance(hitPoint);
                if (hitPointDistance < MAX_DISTANCE) {
                    distance = hitPointDistance;
                    if (WilyCore.enableSensorError) {
                        // 95% (2 standard deviations) of error comes within 0.5 inches:
                        distance += 0.5 / 2 * random.nextGaussian();
                    }
                    break;
                }
            }
        }

        // Return the distance:
        return distanceUnit.fromUnit(DistanceUnit.INCH, distance);
    }
}
