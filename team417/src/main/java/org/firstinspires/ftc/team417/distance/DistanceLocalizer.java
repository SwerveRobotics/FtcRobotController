package org.firstinspires.ftc.team417.distance;

import static org.firstinspires.ftc.team417.distance.VectorUtils.calculateAverage;
import static org.firstinspires.ftc.team417.distance.VectorUtils.rotate;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

// Localizes and returns a pose estimate based on two distance sensors.
public class DistanceLocalizer {
    public UltrasonicDistanceSensor leftDistance, rightDistance;
    public DistanceSensorInfo leftInfo, rightInfo;

    public MecanumDrive drive;

    public Vector2d correction;
    public Vector2d targetCorrection;
    public ArrayList<Vector2d> history = new ArrayList<>();
    final int MAX_HISTORY_SIZE = 10;

    final double MAXIMUM_CORRECTION_VELOCITY = 5; // Inches per second

    // Ultrasonic sensors tend to interfere with each other when fired at the same time.
    double latestLeft = 0; // Latest distance from left sensor
    double latestRight = 0; // Latest distance from right sensor
    boolean leftTurn = true; // If it's the left sensor's turn to get a value

    final DistanceUnit unit = DistanceUnit.INCH;

    final double ANGLE_OF_POSITIVE_CORNER = 0.25 * Math.PI; // Angle facing (72, 72)
    final double RELIABLE_DISTANCE = 48; // Corner is defined as this distance away from walls
    final double MIN_RELIABLE_ANGLE = 7 * Math.PI / 18; // In radians
    final double MAX_RELIABLE_ANGLE = 11 * Math.PI / 36; // In radians

    final ElapsedTime clock = new ElapsedTime();

    private static final Map<FieldSide, Map<FieldSide, Double>> angleMap = new EnumMap<>(FieldSide.class);

    static {
        // Initializing inner maps for each adjacent FieldSide pair with respective angles
        EnumMap<FieldSide, Double> topMap = new EnumMap<>(FieldSide.class);
        topMap.put(FieldSide.RIGHT, 0.0); // 0 degrees
        angleMap.put(FieldSide.TOP, topMap);

        EnumMap<FieldSide, Double> leftMap = new EnumMap<>(FieldSide.class);
        leftMap.put(FieldSide.TOP, Math.PI / 2); // 90 degrees
        angleMap.put(FieldSide.LEFT, leftMap);

        EnumMap<FieldSide, Double> bottomMap = new EnumMap<>(FieldSide.class);
        bottomMap.put(FieldSide.LEFT, Math.PI); // 180 degrees
        angleMap.put(FieldSide.BOTTOM, bottomMap);

        EnumMap<FieldSide, Double> rightMap = new EnumMap<>(FieldSide.class);
        rightMap.put(FieldSide.BOTTOM, 3 * Math.PI / 2); // 270 degrees
        angleMap.put(FieldSide.RIGHT, rightMap);
    }

    public DistanceLocalizer(UltrasonicDistanceSensor leftDistance,
                             DistanceSensorInfo leftInfo,
                             UltrasonicDistanceSensor rightDistance,
                             DistanceSensorInfo rightInfo,
                             MecanumDrive drive) {
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.leftInfo = leftInfo;
        this.rightInfo = rightInfo;
        this.drive = drive;
        this.correction = new Vector2d(0, 0);
        clock.reset();
    }

    long lastLoopTime = 0;

    public Vector2d updateIfPossible() {
        long time = clock.nanoseconds();
        double delta = (time - lastLoopTime) / 1_000_000.0; // Delta is in milliseconds
        lastLoopTime = time;

        getNextDistance(); // Get distances from distance sensors, one each loop

        // Find distances to sides and which sides they are to
        IntersectionResult leftIntersection = FieldSimulator.findIntersection(drive.pose, leftInfo.getPose());
        IntersectionResult rightIntersection = FieldSimulator.findIntersection(drive.pose, rightInfo.getPose());

        // Since distance sensors can't tell you which quadrant you're in
        double heading = normalizeToFirstQuadrant(drive.pose.heading.log());

        boolean sameSide = leftIntersection.side == rightIntersection.side;
        boolean closeEnough = leftIntersection.distance < RELIABLE_DISTANCE && rightIntersection.distance < RELIABLE_DISTANCE;
        boolean angleIsEnough;
        if (MIN_RELIABLE_ANGLE < MAX_RELIABLE_ANGLE) {
            angleIsEnough = MIN_RELIABLE_ANGLE <= heading && heading <= MAX_RELIABLE_ANGLE;
        } else {
            angleIsEnough = !(MAX_RELIABLE_ANGLE <= heading && heading <= MIN_RELIABLE_ANGLE);
        }

        Vector2d detectedRelativePosition, detectedPosition, detectedCorrection;
        if (!sameSide && closeEnough && angleIsEnough) {
            // Detected position relative to the corner
            detectedRelativePosition = new Vector2d(
                    calculateDistance(latestRight, heading, rightInfo),
                    calculateDistance(latestLeft, heading, leftInfo));
        } else {
            return correction;
        }

        Vector2d fieldVector = new Vector2d(FieldSimulator.FIELD_SIZE / 2, FieldSimulator.FIELD_SIZE / 2);

        // Position relative to the center, but doesn't account for which corner it is
        Vector2d unrotatedPosition = (fieldVector).minus(detectedRelativePosition);

        double theta = calculateTheta(leftIntersection.side, rightIntersection.side);

        // Position detected by this iteration of the loop
        detectedPosition = rotate(unrotatedPosition, theta);

        // Correction recommended by this iteration of the loop
        detectedCorrection = detectedPosition.minus(drive.pose.position);

        history.add(detectedCorrection);

        while (history.size() > MAX_HISTORY_SIZE) {
            history.remove(0);
        }

        targetCorrection = calculateAverage(history);

        double xDiff = targetCorrection.x - correction.x;
        double yDiff = targetCorrection.y - correction.y;

        // Delta is in milliseconds
        double maxCorrection = MAXIMUM_CORRECTION_VELOCITY * delta / 1000.0;

        correction = new Vector2d(
                // Correction x and max correction, whichever is absolutely larger,
                // taking the sign of the correction
                correction.x +
                        (Math.abs(xDiff) < Math.abs(maxCorrection)
                                ? xDiff : Math.copySign(maxCorrection, xDiff)),
                // Correction y and max correction, whichever is absolutely larger,
                // taking the sign of the correction
                correction.y +
                        (Math.abs(yDiff) < Math.abs(maxCorrection)
                                ? yDiff : Math.copySign(maxCorrection, yDiff))
        );

        return correction;
    }

    // Method to calculate theta based on two FieldSide values
    public static double calculateTheta(FieldSide side1, FieldSide side2) {
        // First, try to get the angle by treating side1 as the primary key
        Double angle = Objects.requireNonNull(angleMap.getOrDefault(side1, new EnumMap<>(FieldSide.class))).get(side2);

        // If not found, try the reverse order (side2 as the primary key)
        if (angle == null) {
            angle = Objects.requireNonNull(angleMap.getOrDefault(side2, new EnumMap<>(FieldSide.class))).get(side1);
        }

        return (angle != null) ? angle : -1; // Return -1 if no valid angle is found
    }

    // Ultrasound sensors have been found to disrupt each other.
    // Firing only one each loop should mitigate the effect.
    void getNextDistance() {
        if (leftTurn) {
            double tentative = leftDistance.getDistance(unit);
            latestLeft = tentative > 0 ? tentative : latestLeft;
        } else {
            double tentative = rightDistance.getDistance(unit);
            latestRight = tentative > 0 ? tentative : latestRight;
        }
        leftTurn = !leftTurn;
    }

    double calculateDistance(double distance, double heading, DistanceSensorInfo info) {
        // Calculate the heading relative to the line that bisects the right angle of the corner
        // heading because I'm using right is positive, where RoadRunner uses left is positive
        double relative = -heading + ANGLE_OF_POSITIVE_CORNER;

        // Distance from the sensor to the wall
        double sensorToWall = distance * Math.cos(relative);

        // n := distance involving the y-value of the info
        double n = info.getYOffset() * Math.cos(relative - info.getThetaOffset());

        // m := distance involving the x-value of the info
        double m = -info.getXOffset() * Math.sin(relative - info.getThetaOffset());

        return sensorToWall + n + m;
    }

    double normalizeToFirstQuadrant(double theta) {
        // Normalize to [0, 2π) so we're working with a clean base
        theta = theta % (2 * Math.PI);
        if (theta < 0) {
            theta += 2 * Math.PI;
        }

        // Rotate until it's within [0, π/2]
        theta = theta % (Math.PI / 2);

        return theta;
    }
}
