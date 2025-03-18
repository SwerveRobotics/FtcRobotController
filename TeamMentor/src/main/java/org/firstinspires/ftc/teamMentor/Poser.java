package org.firstinspires.ftc.teamMentor;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.GoBildaPinpointDriver;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.util.ArrayList;
import java.util.LinkedList;

/**
 * Structure for tracking ultrasonic sensor fixups in one dimension.
 */
class Fixup {
    double offset; // Offset from the odometry pose
    double time; // Time when the fixup was created, in seconds
    Fixup(double offset) {
        this.offset = offset;
        this.time = nanoTime() * 1e-9; // Convert to seconds
    }
}

/**
 * This class manages the robot's pose estimate. It uses odometry sensors to estimate the robot's
 * position and orientation on the field. It also uses ultrasonic sensors to correct the odometry
 * estimate when the robot is near a wall. The pose estimate is smoothed over time to prevent sudden
 * jumps in the estimate.
 */
public class Poser {
    final double ANGLE_TOLERANCE = Math.toRadians(30); // Tolerance for acceptable basket angle, radians
    final double HALF_CONE_ANGLE = Math.toRadians(10); // Width of the sensor cone, radians
    final double MAX_DISTANCE = 48; // Tolerance for sensor distance from wall, inches
    final double MIN_DISTANCE = 1; // Minimum useful distance from ultrasonic sensor, inches
    final double MAX_DISTANCE_FROM_EXPECTED = 8; // Maximum distance from expected, inches
    final int FILTER_COUNT = 20; // Number of samples needed to filter
    final int FIXUP_RETENTION_TIME = 3; // Time to retain fixups, seconds
    final double MAX_SMOOTHING_VELOCITY = 12; // Smoothing velocity to prevent sudden jumps, inches per second
    final double DISABLE_ULTRASONICS_DISTANCE = 12; // When less than this distance from the target, disable ultrasonic corrections

    public static Ultrasonic[] ultrasonics = { // Ultrasonic sensors
            new Ultrasonic("leftSonic45", new Point(7.5, 6), Math.toRadians(45), 0.35),
            new Ultrasonic("rightSonic45", new Point(7.5, -6), Math.toRadians(-45), 1.07)
    };

    public enum Confidence {NONE, LOW, HIGH} // Confidence in the pose estimate

    private static Poser poser = null; // Global poser object for persisting state from Auto to TeleOp

    public GoBildaPinpointDriver pinpointDriver; // PinPoint driver, can be null if none present
    public SparkFunOTOS otosDriver; // OTOS driver, can be null if none present
    public Pose2d odometryPose; // Most recent raw pose from the odometry sensor
    public Pose2d fusedPose; // Most recent fused odometry and ultrasonic pose, smoothed over time
    public Point filteredOffset = new Point(0, 0); // Raw offset computed from the ultrasonic sensors

    private final Telemetry telemetry; // Telemetry object for debugging
    private final Runnable onHighConfidence; // Callback when confidence transitions to high
    private boolean teleOp; // True if this was last used by a TeleOp; false if by Auto
    private Confidence confidence; // Confidence in the pose estimate
    private PoseVelocity2d poseVelocity; // Most recent pose velocity returned from the sensor, robot-relative
    private double lastTime; // Time of the last ultrasonic update, seconds
    private ArrayList<Segment> beams = new ArrayList<>(); // Beams from the ultrasonic sensors for rendering
    private Pose2d target; // Target pose for the robot to disable ultrasonic sensor corrections

    LinkedList<Fixup> xFixups = new LinkedList<>(); // Corrections to the x coordinate
    LinkedList<Fixup> yFixups = new LinkedList<>(); // Corrections to the y coordinate

    // Structure describing the ultrasonic distance sensors:
    public static class Ultrasonic {
        public String name; // Device's name in the robot configuration
        public double x; // Positive 'x' is towards the front of the robot, negative towards the back
        public double y; // Positive 'y' is towards the left of the robot, negative towards the right
        public double orientation; // Orientation of the sensor relative to the front of the robot, in radians
        public double fudge; // Fudge factor for the sensor, inches

        UltrasonicDistanceSensor sensor; // The driver instantiation for this sensor
        Pose2d odometryPoseWhenTriggered = new Pose2d(0, 0, 0); // Pose when sensor was last triggered

        Ultrasonic(String name, Point configuration, double orientation, double fudge) {
            this.name = name;
            this.x = configuration.x;
            this.y = configuration.y;
            this.orientation = orientation;
            this.fudge = fudge;
        }
    }

    // Singleton pattern to get the Poser object. The initial pose should be specified for Auto and
    // should be null for TeleOp.
    public static Poser getPoser(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, Pose2d pose) {
        // In a match, we need to persist the Poser object from Auto to TeleOp. So if we're in
        // TeleOp and the pose is null and there's a persisted poser object that was created by
        // Auto, we return that object. In all other cases, create a new Poser object.
        if ((pose == null) && (poser != null) && (!poser.teleOp)) {
            poser.teleOp = true;
            return poser;
        }
        poser = new Poser(hardwareMap, telemetry, pose, () -> gamepad.rumble(200)); // Persist in a static global
        return poser;
    }

    // Pose should be null for TeleOp.
    private Poser(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose, Runnable onHighConfidence) {
        this.telemetry = telemetry;
        this.onHighConfidence = onHighConfidence;

        // initializeOtosDriver(hardwareMap);
        initializePinpointDriver(hardwareMap);
        setPose(pose, (pose != null) ? Confidence.HIGH : Confidence.NONE);

        for (Ultrasonic ultrasonic : ultrasonics) {
            ultrasonic.sensor = hardwareMap.tryGet(UltrasonicDistanceSensor.class, ultrasonic.name);
            if (ultrasonic.sensor == null) {
                RobotLog.addGlobalWarningMessage("Missing sensor: " + ultrasonic.name);
            }
        }
    }

    // Set the specified pose to the sensor.
    public void setPose(Pose2d pose, Confidence confidence) {
        if (pose == null)
            pose = new Pose2d(0, 0, 0);

        if (WilyWorks.isSimulating)
            confidence = Confidence.HIGH; // Make simulating easier

        this.confidence = confidence;
        this.odometryPose = pose;
        this.fusedPose = pose;
        this.filteredOffset = new Point(0, 0);

        xFixups.clear();
        yFixups.clear();
        for (Ultrasonic ultrasonic : ultrasonics) {
            ultrasonic.odometryPoseWhenTriggered = pose;
        }

        if (otosDriver != null) {
            otosDriver.setPosition(new SparkFunOTOS.Pose2D(
                    pose.position.x, pose.position.y, pose.heading.toDouble()));
        }
        // Set the pose on the Pinpoint tracking sensor:
        if (pinpointDriver != null) {
            pinpointDriver.setPosition(new Pose2D(
                    DistanceUnit.INCH, pose.position.x, pose.position.y,
                    AngleUnit.RADIANS, pose.heading.toDouble()));
        }
        // Let the simulator know:
        WilyWorks.setStartPose(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
        // Test with velocity: WilyWorks.setStartPose(new Pose2d(0, -48, Math.PI), new PoseVelocity2d(new Vector2d(60, 60), 0));
    }

    // Initialize the Pinpoint sensor if we have one.
    public void initializePinpointDriver(HardwareMap hardwareMap) {
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        MecanumDrive.Params.Pinpoint params = MecanumDrive.PARAMS.pinpoint;

        pinpointDriver.setOffsets(params.xOffset, params.yOffset);
        pinpointDriver.setEncoderResolution(params.ticksPerMm);
        pinpointDriver.setEncoderDirections(
                params.xReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                params.yReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    // Initialize the OTOS sensor if we have one. Derived from configureOtos(). The
    // pose is the initial pose where the robot will start on the field.
    public void initializeOtosDriver(HardwareMap hardwareMap) {
        otosDriver = hardwareMap.get(SparkFunOTOS.class, "pinpoint");
        MecanumDrive.Params.Otos params = MecanumDrive.PARAMS.otos;

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        otosDriver.setLinearUnit(DistanceUnit.INCH);
        otosDriver.setAngularUnit(AngleUnit.RADIANS);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        otosDriver.setOffset(params.offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otosDriver.setLinearScalar(params.linearScalar);
        otosDriver.setAngularScalar(params.angularScalar);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otosDriver.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otosDriver.resetTracking();
    }

    // Rotate a vector by a given angle:
    static public Vector2d rotateVector(Vector2d vector, double theta) {
        return new Point(vector).rotate(theta).vector2d();
    }

    // Update the pose estimates from the odometry sensors.
    void updateFromOdometry() {
        Pose2d oldOdometryPose = odometryPose;
        if (pinpointDriver != null) {
            // Query the driver for position and velocity:
            Stats.beginIo();
            pinpointDriver.update();
            Stats.endIo();

            Stats.beginIo();
            Pose2D pose2D = pinpointDriver.getPosition();
            Pose2D poseVelocity2D = pinpointDriver.getVelocity();
            Stats.endIo();

            // Convert to Road Runner format, remembering that the Pinpoint tracking sensor
            // reports velocity as field-relative but Road Runner wants it robot-relative:
            odometryPose = new Pose2d(
                    pose2D.getX(DistanceUnit.INCH),
                    pose2D.getY(DistanceUnit.INCH),
                    pose2D.getHeading(AngleUnit.RADIANS));
            double xVelocity = poseVelocity2D.getX(DistanceUnit.INCH);
            double yVelocity = poseVelocity2D.getY(DistanceUnit.INCH);
            poseVelocity = new PoseVelocity2d(
                    rotateVector(new Vector2d(xVelocity, yVelocity), -odometryPose.heading.toDouble()),
                    poseVelocity2D.getHeading(AngleUnit.RADIANS));

            if ((Double.isNaN(odometryPose.position.x) || Double.isNaN(odometryPose.position.y) ||
                    Double.isNaN(odometryPose.heading.toDouble())) ||
                    (Double.isNaN(poseVelocity.linearVel.x) || Double.isNaN(poseVelocity.linearVel.y) ||
                            Double.isNaN(poseVelocity.angVel))) {

                System.out.printf("PinPoint NaN! pose: %s, velocity: %s\n", odometryPose, poseVelocity);
                throw new RuntimeException("PinPoint NaN!");
            }
        } else if (otosDriver != null) {
            // Get the current pose and current pose velocity from the optical tracking sensor.
            // Reads over the I2C bus are very slow so for performance we query the velocity only
            // if we'll actually need it - i.e., if using non-zero velocity gains:
            SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D(0, 0, 0);
            SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D(0, 0, 0);
            SparkFunOTOS.Pose2D acceleration = new SparkFunOTOS.Pose2D(0, 0, 0);

            // This single call is faster than separate calls to getPosition() and getVelocity():
            otosDriver.getPosVelAcc(position, velocity, acceleration);

            // Convert to Road Runner format, remembering that the optical tracking sensor
            // reports velocity as field-relative but Road Runner wants it robot-relative:
            odometryPose = new Pose2d(position.x, position.y, position.h);
            poseVelocity = new PoseVelocity2d(
                    rotateVector(new Vector2d(velocity.x, velocity.y), -odometryPose.heading.toDouble()),
                    velocity.h);
        }
        fusedPose = new Pose2d(
                fusedPose.position.x + odometryPose.position.x - oldOdometryPose.position.x,
                fusedPose.position.y + odometryPose.position.y - oldOdometryPose.position.y,
                odometryPose.heading.log());
    }

    // Structure describing a field wall.
    static class Wall {
        double normal;
        Segment segment;
        Wall(double normal, Segment segment) {
            this.normal = normal;
            this.segment = segment;
        }
    }

    // Field walls. Remove 4 inches around each basket to avoid interference.
    final Wall[] WALLS = {
            new Wall(Math.toRadians(180), new Segment(new Point(-72, -68), new Point(-72, 72))),
            new Wall(Math.toRadians(90), new Segment(new Point(-72, 72), new Point(68, 72))),
            new Wall(Math.toRadians(0), new Segment(new Point(72, 68), new Point(72, -72))),
            new Wall(Math.toRadians(-90), new Segment(new Point(72, -72), new Point(-68, -72)))
    };

    // A method to check if a ray intersects a line segment and return the intersection point or null
    // Courtesy of Copilot.
    Point raySegmentIntersection(Ray ray, Segment segment) {
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

    // Measure the ultrasonic sensor and update the fused pose if the sensor is pointing at a wall.
    // Returns true if a valid measurement was done, false otherwise.
    boolean measureUltrasonic(Ultrasonic ultrasonic) {
        // Don't process ultrasonic sensors if we don't have any:
        if (ultrasonic.sensor == null)
            return false; // ====>

        // Compute the location and angle of the sensor in field space using our best fused
        // estimate:
        Point sensorPoint = new Point(ultrasonic.x, ultrasonic.y)
                .rotate(fusedPose.heading.log())
                .add(new Point(fusedPose.position));
        double sensorAngle = ultrasonic.orientation + fusedPose.heading.log();

        // Find a wall that the sensor is pointing straight at, according to the current pose:
        Ray sensorRay = new Ray(sensorPoint, new Point(Math.cos(sensorAngle), Math.sin(sensorAngle)));
        Wall wall = null;
        Point intersection = null;
        for (Wall checkWall : WALLS) {
            intersection = raySegmentIntersection(sensorRay, checkWall.segment);
            if (intersection != null) {
                // Ensure that the entire cone points to the same wall:
                Ray sensorRay1 = new Ray(sensorPoint, new Point(Math.cos(sensorAngle - HALF_CONE_ANGLE),
                        Math.sin(sensorAngle - HALF_CONE_ANGLE)));
                Ray sensorRay2 = new Ray(sensorPoint, new Point(Math.cos(sensorAngle + HALF_CONE_ANGLE),
                        Math.sin(sensorAngle + HALF_CONE_ANGLE)));
                if ((raySegmentIntersection(sensorRay1, checkWall.segment) != null) &&
                        (raySegmentIntersection(sensorRay2, checkWall.segment) != null)) {
                    wall = checkWall;
                }
                break; // Found an intersecting wall, don't bother looking for more
            }
        }
        if (wall == null)
            return false; // ====>

        // Don't process unless we're within acceptable tolerances for accurate sensor readings:
        double expectedDistance = sensorPoint.distanceTo(intersection);
        if ((expectedDistance > MAX_DISTANCE) || (expectedDistance < MIN_DISTANCE))
            return false; // ====>

        double deltaAngle = Util.normalizeAngle(sensorAngle - wall.normal);
        if (Math.abs(deltaAngle) > ANGLE_TOLERANCE)
            return false; // ====>

        // Ask the hardware for the distance reading! This is a read-and-trigger operation
        // so it always returns the result of the *previous* trigger.
        //
        // At this point, we switch from using the fused pose to just the odometry pose as
        // the reference.
        Pose2d originatingOdometryPose = ultrasonic.odometryPoseWhenTriggered;

        Stats.beginIo();
        double measuredDistance = ultrasonic.sensor.getDistance(DistanceUnit.INCH) + ultrasonic.fudge;
        Stats.endIo();

        ultrasonic.odometryPoseWhenTriggered = odometryPose;

        // Remember the beam for rendering:
        beams.add(new Segment(sensorPoint, intersection));

        // The sensor usually returns 0 when it can't get a measurement, so this case is quite
        // frequent:
        if (measuredDistance < MIN_DISTANCE)
            return false; // ====>

        // The sensor reading is unreliable if the distance is too far from the expected value
        // (as might happen if it's picking up a sample on the field instead of the wall, for
        // example):
        if (Math.abs(measuredDistance - expectedDistance) > MAX_DISTANCE_FROM_EXPECTED)
            return false; // ===>

        // Calculate the distance in the dominant axis, taking the cone into account:
        double effectiveAngle = 0;
        if (deltaAngle > HALF_CONE_ANGLE) {
            effectiveAngle = deltaAngle - HALF_CONE_ANGLE;
        } else if (deltaAngle < -HALF_CONE_ANGLE) {
            effectiveAngle = deltaAngle + HALF_CONE_ANGLE;
        }
        double axisDistance = measuredDistance * Math.cos(effectiveAngle);
        Point originatingPoint = new Point(ultrasonic.x, ultrasonic.y)
                .rotate(originatingOdometryPose.heading.log())
                .add(new Point(originatingOdometryPose.position));

        // Add the fixup to the appropriate list:
        if ((wall.normal == Math.toRadians(0)) || (wall.normal == Math.toRadians(180))) {
            double x = intersection.x - axisDistance * Math.cos(wall.normal);
            xFixups.add(new Fixup(x - originatingPoint.x));
        } else if ((wall.normal == Math.toRadians(90)) || (wall.normal == Math.toRadians(-90))) {
            double y = intersection.y - axisDistance * Math.sin(wall.normal);
            yFixups.add(new Fixup(y - originatingPoint.y));
        }

        // Trim the fixup lists to remove outdated entries:
        double currentTime = nanoTime() * 1e-9;
        while (!xFixups.isEmpty() && (currentTime - xFixups.getFirst().time > FIXUP_RETENTION_TIME))
            xFixups.removeFirst();
        while (!yFixups.isEmpty() && (currentTime - yFixups.getFirst().time > FIXUP_RETENTION_TIME))
            yFixups.removeFirst();
        return true;
    }

    // Use the odometry sensors to create a new pose fused from the odometry sensors.
    void updateFromUltrasonic() {
        if (confidence != Confidence.NONE) {
            for (Ultrasonic ultrasonic : ultrasonics) {
                measureUltrasonic(ultrasonic);
            }

            // Compute a new filtered offset unless we're too close to a driving target. This prevents
            // the robot from gyrating at its stopping point due to pose correction from the sensors.
            if ((target == null) || (fusedPose.position.minus(target.position).norm() > DISABLE_ULTRASONICS_DISTANCE)) {
                // If we have enough samples, compute the average correction:
                filteredOffset = new Point(0, 0);
                if (xFixups.size() >= FILTER_COUNT)
                    filteredOffset.x = xFixups.stream().mapToDouble(a -> a.offset).average().orElse(0);
                if (yFixups.size() >= FILTER_COUNT)
                    filteredOffset.y = yFixups.stream().mapToDouble(a -> a.offset).average().orElse(0);

                // If we have enough samples in both dimensions, transition to high confidence:
                if ((confidence != Confidence.HIGH) &&
                        (xFixups.size() >= FILTER_COUNT) &&
                        (yFixups.size() >= FILTER_COUNT)) {
                    confidence = Confidence.HIGH;
                    onHighConfidence.run();
                }
            }
        }

        // The fused pose is the odometry pose plus the offset from the ultrasonic sensors.
        // The offset is smoothed to prevent sudden jumps in the pose estimate.
        double currentTime = nanoTime() * 1e-9;
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        Point correctionVector = new Point(odometryPose.position)
                .add(filteredOffset)
                .subtract(new Point(fusedPose.position));
        double correctionLength = correctionVector.length();
        double maxCorrection = Math.min(MAX_SMOOTHING_VELOCITY * deltaTime, correctionLength);
        Point odometryCorrection = new Point(0, 0);
        if (correctionLength != 0) {
            odometryCorrection = correctionVector.multiply(maxCorrection / correctionLength);
        }

        fusedPose = new Pose2d(
                fusedPose.position.x + odometryCorrection.x,
                fusedPose.position.y + odometryCorrection.y,
                odometryPose.heading.log());
    }

    // Update the pose estimate. If quiesce is true, the ultrasonic sensors
    // will not be activated.
    public void update(boolean quiesceUltrasonic) {
        beams.clear();
        updateFromOdometry();
        updateFromUltrasonic();
    }

    // Do the debug drawing for the robot's pose and the ultrasonic beams.
    public void draw(Canvas canvas) {
        // Draw the overhead view of the robot chassis:
        double cx = Specs.Robot.WIDTH/2;
        double cy = Specs.Robot.LENGTH/2;
        double[] robotPoints = new double[]{-cx, cy, cx, cy, cx, -cy, -cx, -cy, -cx, cy};

        // Check for annoying NaNs that hang FTC Dashboard:
        if (Double.isNaN(fusedPose.position.x) || Double.isNaN(fusedPose.position.y) ||
                Double.isNaN(fusedPose.heading.log()) ||
                Double.isNaN(odometryPose.position.x) || Double.isNaN(odometryPose.position.y) ||
                Double.isNaN(odometryPose.heading.log())) {

            System.out.printf("Poser NaN! fusedPose: %s, odometryPose: %s\n", fusedPose, odometryPose);
            throw new RuntimeException("Poser NaN!");
        }

        // Subtract 90 degrees to account for Dashboard's goofy convention:
        canvas.setTranslation(fusedPose.position.y, -fusedPose.position.x);

        canvas.setRotation(fusedPose.heading.log() - Math.toRadians(90));
        canvas.setStroke("#404040");
        canvas.setStrokeWidth(1);
        canvas.strokePolyline(Xform.getX(robotPoints), Xform.getY(robotPoints));
        canvas.strokeLine(0, 0, cy, 0); // Dash on the front of the robot

        // Reset the transform:
        canvas.setRotation(Math.toRadians(-90)); // Undo goofy Dashboard rotation
        canvas.setTranslation(0, 0);

        // Draw the ultrasonic beams:
        canvas.setStroke("#cccccc");
        for (Segment beam: beams) {
            canvas.strokeLine(beam.p1.x, beam.p1.y, beam.p2.x, beam.p2.y);
            canvas.strokeCircle(beam.p2.x, beam.p2.y, 0.5);
        }

        // Draw the correction as a red line between the odometry pose and the fused pose:
        canvas.setStroke("#ff0000");
        canvas.strokeLine(odometryPose.position.x, odometryPose.position.y,
                fusedPose.position.x, fusedPose.position.y);
    }

    // Reset the fudge factors for the ultrasonic sensors for purposes of tuning:
    public void zeroFudges() {
        for (Ultrasonic ultrasonic: ultrasonics) {
            ultrasonic.fudge = 0;
        }
    }

    // Register an abutment in the X direction when the robot is stopped.
    public void registerAbutmentX(double x, double heading) {
        setPose(new Pose2d(x, fusedPose.position.y, heading), Confidence.HIGH);
    }

    // Set the target so that the poser can disable ultrasonic sensor corrections when the robot
    // is close. This prevents the robot from gyrating at its stopping point due to pose
    // correction from the sensors.
    public void setTarget(Pose2d target) { // Can be null
        this.target = target;
    }

    // Return the latest pose results:
    public Pose2d getPose() { return fusedPose; } // Field-relative
    public PoseVelocity2d getPoseVelocity() { return poseVelocity; } // Robot-relative
    public Confidence getConfidence() { return confidence; }
}
