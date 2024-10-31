package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

public class DistanceLocalizer {
    public UltrasonicDistanceSensor leftDistance, rightDistance;
    public DistanceSensorInfo leftInfo, rightInfo;

    public MecanumDrive drive;

    public Vector2d correction;
    public Vector2d targetCorrection;
    public ArrayList<Vector2d> history = new ArrayList<>();
    final int MAX_HISTORY_SIZE = 10;

    final double MAXIMUM_CORRECTION_VELOCITY = 2;

    double latestLeft = 0;
    double latestRight = 0;
    boolean leftTurn = true;

    final DistanceUnit unit = DistanceUnit.INCH;

    final double ANGLE_OF_POSITIVE_CORNER = 0.25 * Math.PI;
    final double RELIABLE_DISTANCE = 24;

    private static final Map<FieldSide, Map<FieldSide, Double>> angleMap = new EnumMap<>(FieldSide.class);

    static {
        // Initializing inner maps for each adjacent FieldSide pair with respective angles
        EnumMap<FieldSide, Double> topMap = new EnumMap<>(FieldSide.class);
        topMap.put(FieldSide.RIGHT, Math.PI / 4); // 45 degrees
        angleMap.put(FieldSide.TOP, topMap);

        EnumMap<FieldSide, Double> leftMap = new EnumMap<>(FieldSide.class);
        leftMap.put(FieldSide.TOP, 3 * Math.PI / 4); // 135 degrees
        angleMap.put(FieldSide.LEFT, leftMap);

        EnumMap<FieldSide, Double> bottomMap = new EnumMap<>(FieldSide.class);
        bottomMap.put(FieldSide.LEFT, 5 * Math.PI / 4); // 225 degrees
        angleMap.put(FieldSide.BOTTOM, bottomMap);

        EnumMap<FieldSide, Double> rightMap = new EnumMap<>(FieldSide.class);
        rightMap.put(FieldSide.BOTTOM, 7 * Math.PI / 4); // 315 degrees
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
    }

    public void updateIfPossible() {
        getNextDistance();

        Pose2d estimatedPose = new Pose2d(drive.pose.position.plus(correction), drive.pose.heading.log());

        IntersectionResult leftIntersection = FieldSimulator.findIntersection(estimatedPose, leftInfo.getPose());
        IntersectionResult rightIntersection = FieldSimulator.findIntersection(estimatedPose, rightInfo.getPose());

        boolean sameSide = leftIntersection.side == rightIntersection.side;

        Vector2d detectedRelativePosition, detectedPosition, detectedCorrection;
        double heading = drive.pose.heading.log();
        if (!sameSide && leftIntersection.distance < RELIABLE_DISTANCE && rightIntersection.distance < RELIABLE_DISTANCE) {
            detectedRelativePosition = new Vector2d(
                    calculateDistance(latestLeft, heading, leftInfo),
                    calculateDistance(latestRight, heading, rightInfo));
        } else {
            return;
        }

        detectedPosition = rotate((new Vector2d(FieldSimulator.FIELD_SIZE / 2, FieldSimulator.FIELD_SIZE / 2)).minus(detectedRelativePosition), calculateTheta(leftIntersection.side, rightIntersection.side));

        detectedCorrection = detectedPosition.minus(drive.pose.position);

        history.add(detectedCorrection);

        while (history.size() > MAX_HISTORY_SIZE) {
            history.remove(0);
        }

        targetCorrection = calculateAverage(history);

        correction = new Vector2d(
                Math.min(targetCorrection.x, MAXIMUM_CORRECTION_VELOCITY),
                Math.min(targetCorrection.y, MAXIMUM_CORRECTION_VELOCITY)
        );
    }

    public static Vector2d rotate(Vector2d vector, double theta) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        // Applying the rotation matrix
        double rotatedX = vector.x * cosTheta - vector.y * sinTheta;
        double rotatedY = vector.x * sinTheta + vector.y * cosTheta;

        return new Vector2d(rotatedX, rotatedY);
    }

    public static Vector2d calculateAverage(ArrayList<Vector2d> vectors) {
        if (vectors == null || vectors.isEmpty()) {
            return new Vector2d(0, 0);
        }

        double sumX = 0;
        double sumY = 0;

        for (Vector2d vector : vectors) {
            sumX += vector.x;
            sumY += vector.y;
        }

        int count = vectors.size();
        return new Vector2d(sumX / count, sumY / count);
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
            latestLeft = leftDistance.getDistance(unit);
        } else {
            latestRight = rightDistance.getDistance(unit);
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
}
