package com.wilyworks.simulator.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Globals;
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
    final double FIELD_OF_VIEW = Math.toRadians(40); // Specs say 80 degrees but real is less
    final double MAX_DISTANCE = 75; // 150 inches is the theoretical maximum range
    final double MIN_REFLECTION_ANGLE = Math.toRadians(45); // Reflection angle must be more than this

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

    // Calculate the hit point of the ray to the segment if the reflection angle is within
    // acceptable range. Returns Double.MAX_VALUE if there's no intersection:
    private double distanceToSegment(Segment segment, Point sensorPoint, double sensorAngle) {
        double distance = Double.MAX_VALUE;
        double segmentAngle = segment.p2.subtract(segment.p1).atan2();
        double reflectionAngle = Math.abs(Globals.normalizeAngle(segmentAngle - sensorAngle));
        if ((reflectionAngle > MIN_REFLECTION_ANGLE) && (reflectionAngle < Math.PI - MIN_REFLECTION_ANGLE)) {
            Point sensorDirectionVector = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
            Ray sensorRay = new Ray(sensorPoint, sensorDirectionVector);

            // Add this sensor and segment to the eligible list if the sensor points at it
            // and the distance is less than MAX_DISTANCE:
            Point hitPoint = raySegmentIntersection(sensorRay, segment);
            if (hitPoint != null) {
                distance = sensorPoint.distance(hitPoint);
                if (WilyCore.enableSensorError) {
                    // 95% (2 standard deviations) of error comes within +/- 0.5 inches:
                    distance += WilyCore.config.distanceSensorError / 2 * random.nextGaussian();
                }
            }
        }
        return distance;
    }

    @Override
    public double getDistance(DistanceUnit distanceUnit) {
        // Return zero if we have no WilyConfig description about the placement of the sensor:
        if (descriptor == null)
            return 0;

        // Distance sensors reflect the true pose:
        Pose2d truePose = WilyCore.getPose(0, true); // Assume zero latency
        Point sensorPoint = new Point(descriptor.x, descriptor.y)
                .rotate(truePose.heading.log())
                .add(new Point(truePose.position));
        double sensorAngle = descriptor.orientation + truePose.heading.log();

        // Find the first wall segment that the sensor is pointing straight at, according to
        // the current pose:
        double distance = Double.MAX_VALUE;
        double halfFov = FIELD_OF_VIEW / 2.0;
        for (Segment segment : WALL_SEGMENTS) {
            // Take the closest distance reading of any intersection:
            distance = Math.min(distance, distanceToSegment(segment, sensorPoint, sensorAngle - halfFov));
            distance = Math.min(distance, distanceToSegment(segment, sensorPoint, sensorAngle));
            distance = Math.min(distance, distanceToSegment(segment, sensorPoint, sensorAngle + halfFov));
        }

        // Return the distance:
        return (distance > MAX_DISTANCE) ? 0 : distanceUnit.fromUnit(DistanceUnit.INCH, distance);
    }
}
