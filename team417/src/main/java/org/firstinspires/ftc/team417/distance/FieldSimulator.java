package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Pose2d;

public class FieldSimulator {
    final static double FIELD_SIZE = 144;

    public static Pose2d compose(Pose2d base, Pose2d relative) {
        // Convert base heading to radians
        double baseHeadingRad = base.heading.log();

        // Rotate relative coordinates by base heading
        double rotatedX = relative.position.x * Math.cos(baseHeadingRad) -
                relative.position.y * Math.sin(baseHeadingRad);
        double rotatedY = relative.position.x * Math.sin(baseHeadingRad) +
                relative.position.y * Math.cos(baseHeadingRad);

        // Add rotated coordinates to base position
        double newX = base.position.x + rotatedX;
        double newY = base.position.y + rotatedY;

        // Add headings (and normalize to 0-360 range)
        double newHeading = (base.heading.log() + relative.heading.log()) % 360;
        if (newHeading < 0) newHeading += 360;

        return new Pose2d(newX, newY, newHeading);
    }

    /** @noinspection UnnecessaryLocalVariable*/
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
                minDist == Double.POSITIVE_INFINITY ? -1 : minDist);
    }

    public static IntersectionResult findIntersection(Pose2d pose) {
        return findIntersection(pose, FIELD_SIZE);
    }

    public static IntersectionResult findIntersection(Pose2d pose, Pose2d offset) {
        Pose2d combined = compose(pose, offset);
        return findIntersection(combined);
    }
}