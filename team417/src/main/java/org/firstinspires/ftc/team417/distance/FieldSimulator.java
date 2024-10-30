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

    public static void main(String[] args) {
        FieldSimulator simulator = new FieldSimulator();

        // Test cases for `findIntersection`
        System.out.println("Testing `findIntersection` on square boundary:");

        // Test case 1: Heading directly towards the right side from the center
        System.out.println(simulator.findIntersection(new Pose2d(0, 0, Math.toRadians(0))));  // Expected: Right side

        // Test case 2: Heading directly towards the left side from the center
        System.out.println(simulator.findIntersection(new Pose2d(0, 0, Math.toRadians(180)))); // Expected: Left side

        // Test case 3: Heading directly towards the top side from the center
        System.out.println(simulator.findIntersection(new Pose2d(0, 0, Math.toRadians(90))));  // Expected: Top side

        // Test case 4: Heading directly towards the bottom side from the center
        System.out.println(simulator.findIntersection(new Pose2d(0, 0, Math.toRadians(270)))); // Expected: Bottom side

        // Test case 5: Start outside on the right and move leftward (no intersection)
        System.out.println(simulator.findIntersection(new Pose2d(80, 0, Math.toRadians(180)))); // Expected: None

        // Test case 6: Start inside, heading bottom-right (intersects bottom side first)
        System.out.println(simulator.findIntersection(new Pose2d(10, 10, Math.toRadians(315)))); // Expected: Bottom side

        // Test case 7: Diagonal heading from near bottom-left to top-right
        System.out.println(simulator.findIntersection(new Pose2d(-50, -50, Math.toRadians(45)))); // Expected: Right side or Top side

        // Test case 8: From top-left corner, heading diagonally down (exits left side first)
        System.out.println(simulator.findIntersection(new Pose2d(-72, 72, Math.toRadians(225)))); // Expected: Left side

        // Test case 9: Just outside top, pointing down
        System.out.println(simulator.findIntersection(new Pose2d(0, 80, Math.toRadians(270)))); // Expected: None

        // Test case 10: Center, 135-degree heading (intersects left side)
        System.out.println(simulator.findIntersection(new Pose2d(0, 0, Math.toRadians(135)))); // Expected: Left side

        // Test `compose` functionality with offset tests
        System.out.println("\nTesting `compose` with relative positions:");

        // Test case 11: Offset from (0, 0) facing 0 degrees, with offset (10, 10, 90 degrees)
        System.out.println(simulator.compose(new Pose2d(0, 0, 0), new Pose2d(10, 10, Math.toRadians(90)))); // Expect 10, 10, 90 degrees

        // Test case 12: Offset from (10, 10) facing 90 degrees, with offset (10, 0, 0 degrees)
        System.out.println(simulator.compose(new Pose2d(10, 10, Math.toRadians(90)), new Pose2d(10, 0, Math.toRadians(0)))); // Expect position with y offset

        // Test case 13: Offset facing diagonally, compound rotations
        System.out.println(simulator.compose(new Pose2d(5, 5, Math.toRadians(45)), new Pose2d(10, 10, Math.toRadians(30))));

        // Additional cases to ensure robustness:

        // Test case 14: Small heading difference
        System.out.println(simulator.compose(new Pose2d(1, 1, Math.toRadians(1)), new Pose2d(1, 1, Math.toRadians(1))));

        // Test case 15: Zero offset
        System.out.println(simulator.compose(new Pose2d(1, 1, Math.toRadians(0)), new Pose2d(0, 0, Math.toRadians(0))));

        // Test case 16: Large angle, near 360 wraparound
        System.out.println(simulator.compose(new Pose2d(1, 1, Math.toRadians(350)), new Pose2d(1, 1, Math.toRadians(15))));

        // Test case 17: Pose completely outside, with small offset back into field
        System.out.println(simulator.findIntersection(new Pose2d(80, 80, Math.toRadians(135)), new Pose2d(-1, -1, 0)));

        // Test case 18: At bottom boundary, aiming up
        System.out.println(simulator.findIntersection(new Pose2d(0, -72, Math.toRadians(90))));

        // Test case 19: Heading right, just inside field boundary
        System.out.println(simulator.findIntersection(new Pose2d(71, 0, Math.toRadians(0))));

        // Test case 20: Moving diagonally near right boundary
        System.out.println(simulator.findIntersection(new Pose2d(71, 10, Math.toRadians(45))));
    }
}