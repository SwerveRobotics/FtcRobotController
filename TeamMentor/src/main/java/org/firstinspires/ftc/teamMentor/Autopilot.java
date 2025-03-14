package org.firstinspires.ftc.teamMentor;


import static org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

/**
 * Math helper for points and vectors.
 */
class Point { // Can't derive from vector2d because it's marked as final (by default?)
    public double x, y;
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point(Vector2d vector) {
        x = vector.x;
        y = vector.y;
    }
    public Point(VectorF vector) {
        x = vector.get(0);
        y = vector.get(1);
    }
    Vector2d vector2d() { return new Vector2d(x, y); }
    public Point add(Point other) {
        return new Point(this.x + other.x, this.y + other.y);
    }
    public Point subtract(Point other) {
        return new Point(this.x - other.x, this.y - other.y);
    }
    public Point rotate(double theta) {
        return new Point(Math.cos(theta) * x - Math.sin(theta) * y,
                Math.sin(theta) * x + Math.cos(theta) * y);
    }
    public Point negate() { return new Point(-x, -y); }
    public Point multiply(double scalar) { return new Point(x * scalar, y * scalar); }
    public Point divide(double scalar) { return new Point(x / scalar, y / scalar); }
    public double dot(Point other) {
        return this.x * other.x + this.y * other.y;
    }
    public double cross(Point other) {
        return this.x * other.y - this.y * other.x;
    }
    public double distanceTo(Point other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }
    public double length() {
        return Math.hypot(x, y);
    }
    public double atan2() { return Math.atan2(y, x); } // Rise over run
    public String toString() { return String.format("(%.2f, %.2f)", x, y); }
}

class Segment {
    public Point p1, p2;
    public Segment(double x1, double y1, double x2, double y2) {
        p1 = new Point(x1, y1);
        p2 = new Point(x2, y2);
    }
    public Segment(Point p1, Point p2) {
        this.p1 = p1;
        this.p2 = p2;
    }
    public String toString() { return String.format("Segment(%s, %s)", p1, p2); }
}

class Ray {
    public Point origin, direction;
    public Ray(Point origin, Point direction) {
        this.origin = origin;
        this.direction = direction;
    }
}

// Line segment intersection code courtesy of Gemini. It determines if two line segments intersect.
class LineSegmentIntersection {
    public static boolean doSegmentsIntersect(Segment s1, Segment s2) {
        return doLineSegmentsIntersect(s1.p1, s1.p2, s2.p1, s2.p2);
    }

    private static boolean doLineSegmentsIntersect(Point p1, Point q1, Point p2, Point q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    private static boolean onSegment(Point p, Point q, Point r) {
        if (q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
            q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y))
            return true;
        return false;
    }

    private static int orientation(Point p, Point q, Point r) {
        double val = (q.y - p.y) * (r.x - q.x) -
                     (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;
        return (val > 0) ? 1 : 2;
    }
}

// Bread crumbs for the robot to follow to pick up the submersible, in order of closest to the
// goal to farthest.
class Waypoints {
    static final Point[] CLOCKWISE_TO_PICKUP = new Point[] {
            new Point(-48, 0), // Closest, middle left
            new Point(-48, -48), // Bottom left
            new Point(48, -48), // Bottom right
            new Point(48, -12), // middle right
            new Point(48, 48), // Farther, top right
    };
    static final Point[] COUNTERCLOCKWISE_TO_PICKUP = new Point[] {
            new Point(-48, 0),
            new Point(-48, 48),
            new Point(48, 48),
            new Point(48, 12),
    };
    static final Point[] CLOCKWISE_TO_BASKET = new Point[] {
            new Point(48, -48), // Closest - Bottom right
            new Point(48, 0), // Farthest - Middle right
    };
    static final Point[] COUNTERCLOCKWISE_TO_BASKET = new Point[] {
            new Point(-48, 0), // Closest - Middle left
            new Point(-48, 48), // Farthest - Top left
    };
}

class Submersible {
    static final double FUDGE = 2; // Fudge factor for the submersible, inches
    static final Segment[] SEGMENTS = new Segment[] {
            new Segment(-25 - FUDGE, 24, 25 + FUDGE, 24),
            new Segment(-25 - FUDGE, -24, 25 + FUDGE, -24),
            new Segment(0, 24, 0, -24)
    };
}

class Util {
    // Normalize an angle to the range (-PI, PI]:
    static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2*Math.PI;
        while (angle <= -Math.PI)
            angle += 2*Math.PI;
        return angle;
    }
}

/**
 * Class to autonomously drive to a requested target pose, by way of waypoints if necessary.
 */
class Autopilot {
    final double DECELERATION_LATENCY_FUDGE = 0.02; // Latency fudge factor for deceleration, seconds
    final double ABUT_SPEED = 5; // Speed to drive into the wall when abutting, inches/s
    final double MAX_ABUT_DISTANCE = 5; // Maximum distance to abut, inches
    final double ABUT_DONE_DURATION = 0.3; // Time of no movement to consider abutment done, seconds
    final double ABUTMENT_VELOCITY_EPSILON = 2; // Maximum velocity to be considered stopped, inches/s
    final double ABUTMENT_HEADING_EPSILON = Math.toRadians(3); // Maximum heading error to be considered abutted, radians
    final double STOPPING_DISTANCE_EPSILON = 0.75; // For stopping, inches
    final double STOPPING_ANGLE_EPSILON = Math.toRadians(1); // For stopping, radians

    final MecanumDrive drive; // Object for driving motors and measuring voltage
    final Telemetry telemetry; // For debug output
    final Poser poser; // Pose estimator
    final Runnable onAbutment; // Callback when abutment is complete

    Vector2d abutOffset = new Vector2d(0, 0); // Offset to apply to pose to move robot to wall when abutting
    boolean abutComplete = false; // True when abutment is complete
    Pose2d finalTarget; // Target pose
    AutoPilotState state; // Basket, pickup, or none
    boolean abutAtFinal; // True if the target abuts a wall
    double acceleration; // Acceleration to use, positive value, inches/s^2
    double deceleration; // Deceleration to use, negative value, inches/s^2
    double previousTime; // Previous time in seconds
    PoseVelocity2d previousVelocity; // Velocity vector in field coordinates
    double candidateAbutmentTime; // Accumulated time of no movement, in seconds

    // Call this when ready to engage AutoPilot. Do not pre-create because this saves the current
    // velocity.
    Autopilot(MecanumDrive drive, Telemetry telemetry, Poser poser, Runnable onAbutment) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.poser = poser;
        this.onAbutment = onAbutment;

        // Our algorithm does not depend on acceleration being very accurate because we dynamically
        // adjust if the robot lags in real life. Consequently, make it very aggressive:
        this.acceleration = 3 * PARAMS.maxProfileAccel;
        this.deceleration = PARAMS.minProfileAccel;

        previousTime = System.nanoTime() * 1e-9;
        previousVelocity = poser.getPose().times(poser.getPoseVelocity()); // Make velocity field-relative
    }

    // Call this when initiating a new magnet run. This uses the current velocity so cannot be
    // pre-called.
    void setTarget(Pose2d target, AutoPilotState state, boolean abut) {
        this.finalTarget = target;
        this.state = state;
        this.abutAtFinal = abut;

        poser.setTarget(target); // Tell the pose estimator where we're going
    }

    // Return 1 if the point is on one side of the line, -1 if on the other, and 0 if on the line.
    static int getSide(Segment s, Point p) {
        double cross = (s.p2.x - s.p1.x) * (p.y - s.p1.y) - (s.p2.y - s.p1.y) * (p.x - s.p1.x);
        return (cross > 0) ? 1 : (cross < 0) ? -1 : 0;
    }

    // Determine if the robot has a clear path from the current pose to the destination point.
    boolean unobstructedView(Canvas canvas, Pose2d current, Point destinationPoint, Bounds bounds) {
        double theta = current.heading.log();
        Point currentCenter = new Point(current.position);

        // Vectors to the each corner from the center:
        Point p0 = new Point(bounds.xMin, bounds.yMin).rotate(theta);
        Point p1 = new Point(bounds.xMax, bounds.yMin).rotate(theta);
        Point p2 = new Point(bounds.xMax, bounds.yMax).rotate(theta);
        Point p3 = new Point(bounds.xMin, bounds.yMax).rotate(theta);

        Segment[] paths = {
                new Segment(p0.add(currentCenter), p0.add(destinationPoint)),
                new Segment(p1.add(currentCenter), p1.add(destinationPoint)),
                new Segment(p2.add(currentCenter), p2.add(destinationPoint)),
                new Segment(p3.add(currentCenter), p3.add(destinationPoint))
        };
        for (Segment path: paths) {
            canvas.setFill("#a0a0a0");
            canvas.fillCircle(path.p1.x, path.p1.y, 1);
        }

        for (Segment s: Submersible.SEGMENTS) {
            for (Segment path: paths) {
                if (LineSegmentIntersection.doSegmentsIntersect(s, path)) {
                    canvas.setStroke("#00ff00");
                    canvas.strokeLine(path.p1.x, path.p1.y, path.p2.x, path.p2.y);
                    canvas.setStroke("#ff0000");
                    canvas.strokeLine(s.p1.x, s.p1.y, s.p2.x, s.p2.y);

                    // This intersection is a false-positive if the robot is slightly straddling
                    // the submersible segment due to positioning error. See if this is the case,
                    // by determining if the robot's center is on one side of the ray defined
                    // by the submersible segment and the robot's corner is on the other.
                    // Determine this by taking the cross-product of the vector from the robot's
                    // center to the submersible segment with the vector from the robot's corner
                    // to the submersible segment. If the signs are different, then the robot is
                    // straddling the segment.
                    if (getSide(s, currentCenter) == getSide(s, path.p1))
                        return false;
                }
            }
        }
         return true;
    }

    // Normalize a vector when the length is already known, avoiding divide-by-zero:
    static Vector2d normalize(Vector2d v, double length) {
        return (length != 0) ? v.div(length) : v;
    }

    // This version does not consider walls. 'userVelocity' is field-relative, voltage-based.
    private boolean straightTo(
            Canvas canvas, // For debug rendering
            Vector2d userInput, // User field-relative gamepad inputs, voltage-based
            Point targetPoint, // Destination point
            double targetHeading // Destination heading
    ) {
        Pose2d currentPose = poser.getPose();
        Pose2d abuttingPose = new Pose2d(currentPose.position.plus(abutOffset), currentPose.heading);

        // Remember that volts = Ks*signum(vel) + Kv*vel + Ka*accel
        double batteryVoltage = WilyWorks.isSimulating ? 13 : drive.getVoltage();
        double fullVoltageSpeed = (batteryVoltage - PARAMS.kS) / PARAMS.kV; // Assume no acceleration

        double time = System.nanoTime() * 1e-9;
        double deltaT = time - previousTime;

        // Position calculations -------------------------------------------------------------------
        // Radial (along-the-track) vector towards the target:
        Vector2d positionalRadialVector = targetPoint.vector2d().minus(abuttingPose.position);
        double positionalRadialLength = positionalRadialVector.norm();
        Vector2d normalizedRadialVector = normalize(positionalRadialVector, positionalRadialLength);

        // Tangent (cross-track) vector -90 degrees from radial:
        Vector2d positionalTangentVector = new Vector2d(positionalRadialVector.y, -positionalRadialVector.x);
        double tangentLength = positionalTangentVector.norm();
        Vector2d normalizedTangentVector = normalize(positionalTangentVector, tangentLength);

        // Angular distance to the target:
        double positionalAngularDelta = Util.normalizeAngle(targetHeading - currentPose.heading.log());


        // Decide if we're done:

        boolean abut = !abutComplete && abutAtFinal && ((targetPoint.x == finalTarget.position.x) &&
                                                        (targetPoint.y == finalTarget.position.y));
        telemetry.addLine(String.format("<big>Abut: %b, Complete: %b, At final: %b</big>",
                abut, abutComplete, abutAtFinal));

        if (!abut) {
            telemetry.addLine(String.format("<big>Stop: %.1f\" (%.1f, %.1f), angle: %.1f\u00b0</big>",
                    positionalRadialLength,
                    positionalRadialVector.x,
                    positionalRadialVector.y,
                    Math.toDegrees(positionalAngularDelta)));

            // When not abutting, we're done if the distance is small enough to the goal and there's
            // no user input:
            if ((positionalRadialLength < STOPPING_DISTANCE_EPSILON) &&
                    (Math.abs(positionalAngularDelta) < STOPPING_ANGLE_EPSILON) &&
                    (userInput.x == 0) && (userInput.y == 0)) {

                drive.setDrivePowers(drive.pose, drive.poseVelocity, null, new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }
        } else {
            // X value of the robot center if it's abutting the submersible wall:
            double abutmentX = FieldSpecs.ABUTMENT_X - WilyConfig.ROBOT_LENGTH / 2;

            // Wily Works doesn't support wall collisions properly yet, so just hack it here:
            if (WilyWorks.isSimulating) {
                Pose2d truePose = WilyWorks.getPose(true);
                if (truePose.position.x > abutmentX) {
                    WilyWorks.setStartPose(new Pose2d(abutmentX, truePose.position.y, 0), null);
                }
            }

            // When abutting, look at the odometry velocity to see if we're stopped in the real
            // world. We don't use 'previousVelocity' because that's computed and doesn't
            // necessarily reflect the real world.
            PoseVelocity2d poseVelocity = poser.getPoseVelocity();
            double measuredVelocity = Math.hypot(poseVelocity.linearVel.x, poseVelocity.linearVel.y);
            double measuredHeading = currentPose.heading.log();
            double headingError = Math.abs(Util.normalizeAngle(targetHeading - measuredHeading));

            telemetry.addLine(String.format("<big>Abut distance: %.1f, Vel: %.1f, Angle: .1f\u00b0</big>",
                positionalRadialLength, measuredVelocity, headingError));

            if ((positionalRadialLength > MAX_ABUT_DISTANCE) ||
                    (measuredVelocity > ABUTMENT_VELOCITY_EPSILON) ||
                    (headingError > ABUTMENT_HEADING_EPSILON)) {

                candidateAbutmentTime = 0; // We're not stopped and abutting
            } else {
                candidateAbutmentTime += deltaT;
                if (candidateAbutmentTime > ABUT_DONE_DURATION) {

                    System.out.printf("Pre-abutmentX: fusedX: %.1f, odomX: %.1f, fusedFixupsX: %d, filteredOffset.x: %.1f\n",
                            poser.fusedPose.position.x,
                            poser.odometryPose.position.x,
                            poser.xFixups.size(),
                            poser.filteredOffset.x);
                    System.out.printf("Pre-abutmentX: fusedY: %.1f, odomY: %.1f, fusedFixupsY: %d, filteredOffset.y: %.1f\n",
                            poser.fusedPose.position.y,
                            poser.odometryPose.position.y,
                            poser.yFixups.size(),
                            poser.filteredOffset.y);

                    // We are stopped and abutting, so register the abutment:
                    poser.registerAbutmentX(abutmentX, 0);

                    System.out.printf("Pose-abutmentX: fusedY: %.1f, odomY: %.1f, fusedFixups: %d, filteredOffset: %.1f\n",
                            poser.fusedPose.position.y,
                            poser.odometryPose.position.y,
                            poser.yFixups.size(),
                            poser.filteredOffset.y);

                    abutComplete = true; // Let the robot move along the wall
                    abut = false;
                    onAbutment.run(); // Notify the UI so that it can rumble the gamepad
                    // The robot can continue to move along the wall. The usual termination code
                    // will kick in.
                }
            }
        }

        // Compute the current speed:
        double previousSpeed = Math.hypot(previousVelocity.linearVel.x, previousVelocity.linearVel.y);

        // Compute the angle from the robot's current direction to the target:
        double theta = Util.normalizeAngle(Math.atan2(positionalRadialVector.y, positionalRadialVector.x)
                                         - Math.atan2(previousVelocity.linearVel.y, previousVelocity.linearVel.x));
        Vector2d compositeVelocityVector;
        String velocityColor;
        if ((Math.abs(theta) > Math.toRadians(45)) && (Math.abs(theta) < Math.toRadians(135)) &&
                (previousSpeed > 0)) {
            // When moving orthogonally to the target, do a retrograde burn until the velocity
            // vector starts pointing to the target or reaches zero. This will make for more
            // efficient trajectories and prevent orbiting.
            double retrogradeSpeed = Math.max(0, previousSpeed - acceleration * deltaT);
            Vector2d normalizedVelocity = normalize(previousVelocity.linearVel, previousSpeed);
            compositeVelocityVector = normalizedVelocity.times(retrogradeSpeed);
            velocityColor = "#a020f0"; // purple
        } else {
            // Compute the component velocities. If a velocity is negative, that means that the
            // robot is currently moving away from the target in that component direction.
            double tangentVelocity = previousSpeed * Math.sin(theta);
            double radialVelocity = previousSpeed * Math.sin(theta);

            // Decrease the magnitude of the tangent speed. Stop at zero.
            double tangentSpeed = Math.max(0, Math.abs(tangentVelocity) + deceleration * deltaT);
            tangentVelocity = Math.signum(tangentVelocity) * tangentSpeed;
            Vector2d tangentVelocityVector = normalizedTangentVector.times(tangentVelocity);

            // Calculate the maximum speed directly towards the target assuming constant
            // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
            // current velocity, u is the initial velocity, a is the acceleration, s is distance
            // traveled. We apply it in reverse.
            //
            // But first, without compensation for latency when computing deceleration, we suffer from
            // overshoot and oscillation. Project into the future to compensate:
            double latencyCompensatedDistance = Math.max(0, positionalRadialLength - Math.abs(radialVelocity) * DECELERATION_LATENCY_FUDGE);
            double decreasingCompositeSpeed = Math.sqrt(-2 * deceleration * latencyCompensatedDistance);
            double increasingCompositeSpeed = previousSpeed + acceleration * deltaT;

            // Set the new composite speed as the minimum of the three:
            double compositeSpeed = Math.min(Math.min(increasingCompositeSpeed, decreasingCompositeSpeed), fullVoltageSpeed);

            // Calculate the new composite velocity vector:
            double alpha = Math.PI / 2; // Use 100% of the tangent velocity when avoiding NaNs
            if (tangentSpeed < compositeSpeed) // Avoid NaN when speed is zero because stopped or unusual cases
                alpha = Math.asin(tangentSpeed / compositeSpeed);
            compositeVelocityVector = normalizedRadialVector
                    .times(compositeSpeed * Math.cos(alpha)).plus(tangentVelocityVector);

            // Show the tangent velocity vector:
            canvas.setStroke("#ffc0cb"); // Tangent is pink
            canvas.strokeLine(0, 0, tangentVelocityVector.x, tangentVelocityVector.y);
            velocityColor = "#404040"; // Velocity is grey
        }

        // Heading calculations --------------------------------------------------------------------
        double angularVelocity = previousVelocity.angVel;

        // Calculate the angular velocity as a positive magnitude:
        if (positionalAngularDelta < 0) {
            double increasingAngularSpeed = angularVelocity - PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = -PARAMS.maxAngVel;
            double angularApproach = -Math.sqrt(2 * PARAMS.maxAngAccel * Math.abs(positionalAngularDelta));
            angularVelocity = Math.max(Math.max(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        } else {
            double increasingAngularSpeed = angularVelocity + PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = PARAMS.maxAngVel;
            double angularApproach = Math.sqrt(2 * PARAMS.maxAngAccel * Math.abs(positionalAngularDelta));
            angularVelocity = Math.min(Math.min(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        }

        // Combine ---------------------------------------------------------------------------------
        // Add the component velocities to get our new velocity:
        PoseVelocity2d autoVelocity = new PoseVelocity2d(compositeVelocityVector, angularVelocity);
        double blendFactor = userInput.norm();
        Vector2d userRedOrientedVelocity = userInput.times(fullVoltageSpeed);
        Point userFieldOrientedVelocity = new Point(userRedOrientedVelocity).rotate(Orientation.getRotation() - Math.PI/2);

        // Blend the user and auto velocities, capping at the maximum velocity:
        PoseVelocity2d blendedVelocity = new PoseVelocity2d(new Vector2d(
                (1 - blendFactor) * autoVelocity.linearVel.x + (blendFactor) * userFieldOrientedVelocity.x,
                (1 - blendFactor) * autoVelocity.linearVel.y + (blendFactor) * userFieldOrientedVelocity.y),
                autoVelocity.angVel);

        drive.setDrivePowers(drive.pose, drive.poseVelocity, null, blendedVelocity);

        if (!abut) {
            // Zero out any abutment offset for the next loop:
            abutOffset = new Vector2d(0, 0);
        } else {
            // Ensure that the robot is moving towards the wall for the next loop:
            double blendedSpeed = Math.hypot(blendedVelocity.linearVel.x, blendedVelocity.linearVel.y);
            if ((blendedSpeed < ABUT_SPEED) && (positionalRadialLength < MAX_ABUT_DISTANCE)) {
                double abutSpeed = ABUT_SPEED - blendedSpeed;
                abutOffset = new Vector2d(abutOffset.x - abutSpeed * deltaT, 0);
            }
        }

        canvas.setStroke(velocityColor);
        canvas.strokeLine(0, 0, autoVelocity.linearVel.x, autoVelocity.linearVel.y);

        // Remember stuff for the next iteration:
        previousVelocity = blendedVelocity;
        previousTime = time;
        return true;
    }

    // Get the distance to the target, in inches:
    double distanceToFinalTarget() {
        return finalTarget.position.minus(drive.pose.position).norm();
    }

    // Drive to the target specified by setTarget(), considering waypoints. Return false when done,
    // true while still working on it. 'userInput' is the user's field-relative gamepad inputs
    // (-1 to 1).
    public boolean update(Canvas canvas) { return update(canvas, new Vector2d(0, 0)); }
    public boolean update(Canvas canvas, Vector2d userInput) {
        Bounds bounds = new Bounds(); // Consider deprecating?
        double fudge = 1;
        bounds.xMin = -WilyConfig.ROBOT_LENGTH/2 - fudge;
        bounds.xMax = WilyConfig.ROBOT_LENGTH/2 + fudge;
        bounds.yMin = -WilyConfig.ROBOT_WIDTH/2 - fudge;
        bounds.yMax = WilyConfig.ROBOT_WIDTH/2 + fudge;

        // The close-enough threshold accommodates for epsilon errors when the final target is
        // very close to a boundary.
        Pose2d pose = poser.getPose();
        Point visibleTarget = null;

        if (visibleTarget == null) {
            canvas.setFill("#00ff00");
            canvas.fillCircle(finalTarget.position.x, finalTarget.position.y, 2);

            if (unobstructedView(canvas, pose, new Point(finalTarget.position), bounds)) {
                visibleTarget = new Point(finalTarget.position);
            }
        }
        Point[] waypoints = null;
        if (visibleTarget == null) {
            // Choose our path to the target:
            boolean clockwise = (drive.pose.position.x > drive.pose.position.y);
            if (state == AutoPilotState.PICKUP) {
                if (drive.pose.position.x < 0)
                    clockwise = true;
                waypoints = clockwise ? Waypoints.CLOCKWISE_TO_PICKUP : Waypoints.COUNTERCLOCKWISE_TO_PICKUP;
            } else if (state == AutoPilotState.BASKET) {
                waypoints = clockwise ? Waypoints.CLOCKWISE_TO_BASKET : Waypoints.COUNTERCLOCKWISE_TO_BASKET;
            }

            if (waypoints != null) {
                for (Point waypoint : waypoints) {
                    if (unobstructedView(canvas, pose, waypoint, bounds)) {
                        visibleTarget = waypoint;
                        break; // ====>
                    }
                }
            }
        }

        // If there's no visible target, set the target to the robot's current position. That
        // will keep the updates going, and will allow the human driver to override:
        if (visibleTarget == null) {
            visibleTarget = new Point(pose.position);
        }

        canvas.setFill("#0000ff");
        canvas.fillCircle(visibleTarget.x, visibleTarget.y, 1);

        // @@@ Disable ultrasound sensors when close to the final destination!
        return straightTo(canvas, userInput, visibleTarget, finalTarget.heading.log());
    }
}