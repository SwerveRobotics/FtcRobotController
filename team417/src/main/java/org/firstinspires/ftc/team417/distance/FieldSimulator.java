package org.firstinspires.ftc.team417.distance;

import static org.firstinspires.ftc.team417.distance.VectorUtils.rotate;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// Simulates the field, helping to determine when to use the distance sensors to correct the pose.
public class FieldSimulator {
    final static double FIELD_SIZE = 144;

    public static Pose2d compose(Pose2d base, Pose2d relative) {
        // Rotate relative coordinates by base heading
        Vector2d rotated = rotate(relative.position, base.heading.log());

        // Add rotated coordinates to base position
        Vector2d sum = base.position.plus(rotated);

        // Add headings (and normalize to 0-2π range)
        double newHeading = (base.heading.log() + relative.heading.log()) % (2 * Math.PI);
        if (newHeading < 0) newHeading += (2 * Math.PI);

        return new Pose2d(sum, newHeading);
    }

    /** @noinspection UnnecessaryLocalVariable*/
    // Finds the intersection with a square of a certain side length and a ray.
    public static IntersectionResult findIntersection(Pose2d ray, double squareSize) {
        // Convert heading to radians and calculate direction vector
        double headingRad = ray.heading.log();
        double dx = Math.cos(headingRad);
        double dy = Math.sin(headingRad);

        // Square boundaries (assuming centered at origin)
        double halfSize = squareSize / 2;
        double left = -halfSize;
        double right = halfSize;
        double bottom = -halfSize;
        double top = halfSize;

        // If ray starts outside square, return NONE
        if (ray.position.x < left || ray.position.x > right || ray.position.y < bottom || ray.position.y > top) {
            return new IntersectionResult(FieldSide.NONE, -1);
        }

        // Calculate distances to each side
        double distToRight = (dx > 0) ? (right - ray.position.x) / dx : Double.POSITIVE_INFINITY;
        double distToLeft = (dx < 0) ? (left - ray.position.x) / dx : Double.POSITIVE_INFINITY;
        double distToTop = (dy > 0) ? (top - ray.position.y) / dy : Double.POSITIVE_INFINITY;
        double distToBottom = (dy < 0) ? (bottom - ray.position.y) / dy : Double.POSITIVE_INFINITY;

        // Find the smallest positive distance
        double minDist = Double.POSITIVE_INFINITY;
        FieldSide hitSide = FieldSide.NONE;

        if (distToRight > 0 && distToRight < minDist) {
            minDist = distToRight;
            hitSide = FieldSide.RIGHT;
        }
        if (distToLeft > 0 && distToLeft < minDist) {
            minDist = distToLeft;
            hitSide = FieldSide.LEFT;
        }
        if (distToTop > 0 && distToTop < minDist) {
            minDist = distToTop;
            hitSide = FieldSide.TOP;
        }
        if (distToBottom > 0 && distToBottom < minDist) {
            minDist = distToBottom;
            hitSide = FieldSide.BOTTOM;
        }

        return new IntersectionResult(hitSide,
                minDist);
    }

    public static IntersectionResult findIntersection(Pose2d pose) {
        return findIntersection(pose, FIELD_SIZE);
    }

    public static IntersectionResult findIntersection(Pose2d pose, Pose2d offset) {
        Pose2d combined = compose(pose, offset);
        return findIntersection(combined);
    }
}