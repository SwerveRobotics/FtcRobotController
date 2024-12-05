package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Map;

// Localizes and returns a pose estimate based on two distance sensors.
public class DistanceLocalizer {
    public boolean enabled;

    public UltrasonicDistanceSensor leftDistance, rightDistance;
    public DistanceSensorInfo leftInfo, rightInfo;

    public MecanumDrive drive;

    public boolean correcting = false;

    public Vector2d correction;
    public Double xTargetCorrection;
    public Double yTargetCorrection;
    public ArrayList<Double> xHistory = new ArrayList<Double>();
    public ArrayList<Double> yHistory = new ArrayList<Double>();
    final int MAX_HISTORY_SIZE = 10;

    final double MAXIMUM_CORRECTION_VELOCITY = 10; // Inches per second

    // Ultrasonic sensors tend to interfere with each other when fired at the same time.
    double latestLeft = 0; // Latest distance from left sensor
    double latestRight = 0; // Latest distance from right sensor
    boolean leftTurn = true; // If it's the left sensor's turn to get a value

    final DistanceUnit unit = DistanceUnit.INCH;

    final double RELIABLE_DISTANCE = 48; // Corner is defined as this distance away from walls
    final double MAX_RELIABLE_ANGLE = Math.PI / 8; // In radians

    final ElapsedTime clock = new ElapsedTime();

    private static final Map<FieldSide, Double> angleMap = new EnumMap<>(FieldSide.class);

    static {
        angleMap.put(FieldSide.TOP, 0.0); // 0 degrees
        angleMap.put(FieldSide.RIGHT, Math.PI / 2); // 90 degrees
        angleMap.put(FieldSide.BOTTOM, Math.PI); // 180 degrees
        angleMap.put(FieldSide.LEFT, 3 * Math.PI / 2); // 270 degrees
        angleMap.put(FieldSide.NONE, null); // Robot should not be outside of the field
    }

    public DistanceLocalizer(UltrasonicDistanceSensor leftDistance,
                             DistanceSensorInfo leftInfo,
                             UltrasonicDistanceSensor rightDistance,
                             DistanceSensorInfo rightInfo,
                             MecanumDrive drive, boolean enabled) {
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.leftInfo = leftInfo;
        this.rightInfo = rightInfo;
        this.drive = drive;
        this.enabled = enabled;
        this.correction = new Vector2d(0, 0);
        clock.reset();
    }

    long lastLoopTime = 0;

    public Vector2d updateIfPossible() {
        if (enabled) {
            long time = clock.nanoseconds();
            double delta = (time - lastLoopTime) / 1_000_000.0; // Delta is in milliseconds
            lastLoopTime = time;

            getNextDistance(); // Get distances from distance sensors, one each loop

            // Find distances to sides and which sides they are to
            IntersectionResult leftIntersection = FieldSimulator.findIntersection(drive.pose, leftInfo.getPose());
            IntersectionResult rightIntersection = FieldSimulator.findIntersection(drive.pose, rightInfo.getPose());

            // Since distance sensors can't tell you which quadrant you're in
            // Since the "top" is considered to be PI / 2 in theta
            // Heading has top as 0, while theta has top as Math.PI / 2
            double rawHeading = drive.pose.heading.log();
//        double num1 = (rawHeading - (Math.PI / 4)) / (Math.PI / 2);
//        double num2 = Math.floor(num1);
//        double theta = (num2 * (Math.PI / 2)) % (2 * Math.PI);
//        if (theta < 0) {
//            theta += 2 * Math.PI;
//        }
            Double leftAngle = angleMap.get(leftIntersection.side);

            Double leftTheta = leftAngle == null ? null : ((2 * Math.PI) - leftAngle) % (2 * Math.PI);

            Double leftHeading = leftTheta == null ? null : rawHeading - leftTheta - Math.PI / 2;
            leftHeading = leftHeading == null ? null : leftHeading % (2 * Math.PI);

            Double rightAngle = angleMap.get(rightIntersection.side);

            Double rightTheta = rightAngle == null ? null : ((2 * Math.PI) - rightAngle) % (2 * Math.PI);

            Double rightHeading = rightTheta == null ? null : rawHeading - rightTheta - Math.PI / 2;
            rightHeading = rightHeading == null ? null : rightHeading % (2 * Math.PI);

            boolean sameSide = leftIntersection.side == rightIntersection.side;
            boolean leftCloseEnough = leftIntersection.distance < RELIABLE_DISTANCE;
            boolean rightCloseEnough = rightIntersection.distance < RELIABLE_DISTANCE;
            Double leftRelativeAngle = leftTheta == null ? null : normalizeToPiOver2(leftTheta + Math.PI / 2 - (rawHeading - leftInfo.getThetaOffset()));
            Double rightRelativeAngle = rightTheta == null ? null : normalizeToPiOver2(rightTheta + Math.PI / 2 - (rawHeading - rightInfo.getThetaOffset()));

            double[] leftFactor = angleToUnitVectorWithEpsilon(angleMap.get(leftIntersection.side));
            double[] rightFactor = angleToUnitVectorWithEpsilon(angleMap.get(rightIntersection.side));

            Double xRelativePosition, yRelativePosition;
            Double xAbsolutePosition = null, yAbsolutePosition = null;

            // If sensors face the same side
            if (sameSide) {
                // If both are close enough, choose the one that's more straight on
                if (leftCloseEnough && rightCloseEnough) {
                    // If the left sensor is more straight on to the field wall
                    if (leftRelativeAngle != null && rightRelativeAngle != null && Math.abs(leftRelativeAngle) < Math.abs(rightRelativeAngle)) {
                        // If left sensor satisfies angle requirement
                        if (Math.abs(leftRelativeAngle) < MAX_RELIABLE_ANGLE) {
                            Double[] absolutePosition = calculatePosition(leftFactor, latestLeft, leftHeading, leftTheta, leftInfo);
                            if (xAbsolutePosition == null) {
                                xAbsolutePosition = absolutePosition[0];
                            }
                            if (yAbsolutePosition == null) {
                                yAbsolutePosition = absolutePosition[1];
                            }
                        }
                    } else {
                        // If right sensor satisfies angle requirement
                        if (rightRelativeAngle != null && Math.abs(rightRelativeAngle) < MAX_RELIABLE_ANGLE) {
                            Double[] absolutePosition = calculatePosition(rightFactor, latestRight, rightHeading, rightTheta, rightInfo);
                            if (xAbsolutePosition == null) {
                                xAbsolutePosition = absolutePosition[0];
                            }
                            if (yAbsolutePosition == null) {
                                yAbsolutePosition = absolutePosition[1];
                            }
                        }
                    }
                } else if (leftCloseEnough) {
                    // If left sensor satisfies angle requirement
                    if (leftRelativeAngle != null && Math.abs(leftRelativeAngle) < MAX_RELIABLE_ANGLE) {
                        Double[] absolutePosition = calculatePosition(leftFactor, latestLeft, leftHeading, leftTheta, leftInfo);
                        if (xAbsolutePosition == null) {
                            xAbsolutePosition = absolutePosition[0];
                        }
                        if (yAbsolutePosition == null) {
                            yAbsolutePosition = absolutePosition[1];
                        }
                    }
                } else if (rightCloseEnough) {
                    // If right sensor satisfies angle requirement
                    if (rightRelativeAngle != null && Math.abs(rightRelativeAngle) < MAX_RELIABLE_ANGLE) {
                        Double[] absolutePosition = calculatePosition(rightFactor, latestRight, rightHeading, rightTheta, rightInfo);
                        if (xAbsolutePosition == null) {
                            xAbsolutePosition = absolutePosition[0];
                        }
                        if (yAbsolutePosition == null) {
                            yAbsolutePosition = absolutePosition[1];
                        }
                    }
                }
            } else { // If sensors don't face the same side
                // If left sensor is close enough and satisfies angle requirement
                if (leftRelativeAngle != null && leftCloseEnough && Math.abs(leftRelativeAngle) < MAX_RELIABLE_ANGLE) {
                    Double[] absolutePosition = calculatePosition(leftFactor, latestLeft, leftHeading, leftTheta, leftInfo);
                    if (xAbsolutePosition == null) {
                        xAbsolutePosition = absolutePosition[0];
                    }
                    if (yAbsolutePosition == null) {
                        yAbsolutePosition = absolutePosition[1];
                    }
                }
                // If right sensor is close enough and satisfies angle requirement
                if (rightRelativeAngle != null && rightCloseEnough && Math.abs(rightRelativeAngle) < MAX_RELIABLE_ANGLE) {
                    Double[] absolutePosition = calculatePosition(rightFactor, latestRight, rightHeading, rightTheta, rightInfo);
                    if (xAbsolutePosition == null) {
                        xAbsolutePosition = absolutePosition[0];
                    }
                    if (yAbsolutePosition == null) {
                        yAbsolutePosition = absolutePosition[1];
                    }
                }
            }

            Double xDetectedCorrection = xAbsolutePosition == null ? null
                    : xAbsolutePosition - drive.pose.position.x;
            Double yDetectedCorrection = yAbsolutePosition == null ? null
                    : yAbsolutePosition - drive.pose.position.y;

            if (drive.pose.position.y < 0) {
                xDetectedCorrection = null;
                yDetectedCorrection = null;
            }

            if (xDetectedCorrection != null) {
                xHistory.add(xDetectedCorrection);

                while (xHistory.size() > MAX_HISTORY_SIZE) {
                    xHistory.remove(0);
                }
            }

            if (yDetectedCorrection != null) {
                yHistory.add(yDetectedCorrection);

                while (yHistory.size() > MAX_HISTORY_SIZE) {
                    yHistory.remove(0);
                }
            }

            correcting = xDetectedCorrection != null || yDetectedCorrection != null;

            xTargetCorrection = xHistory.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(0.0);
            yTargetCorrection = yHistory.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(0.0);

            double xDiff = xTargetCorrection - correction.x;
            double yDiff = yTargetCorrection - correction.y;

            // Delta is in milliseconds
            double maxCorrection = MAXIMUM_CORRECTION_VELOCITY * delta / 1000.0;

            correction = new

                    Vector2d(
                    // Correction x and max correction, whichever is absolutely larger,
                    // taking the sign of the correction
                    correction.x +
                            (Math.abs(xDiff) < Math.

                                    abs(maxCorrection)
                                    ? xDiff : Math.copySign(maxCorrection, xDiff)),
                    // Correction y and max correction, whichever is absolutely larger,
                    // taking the sign of the correction
                    correction.y +
                            (Math.abs(yDiff) < Math.

                                    abs(maxCorrection)
                                    ? yDiff : Math.copySign(maxCorrection, yDiff))
            );

            return correction;
        }
        return new Vector2d(0, 0);
    }

    Double[] calculatePosition(double[] factor, double latest, double heading, double theta, DistanceSensorInfo info) {
        double xRelativePosition, yRelativePosition;
        Double xAbsolutePosition = null, yAbsolutePosition = null;
        if (factor[0] * latest != 0) {
            xRelativePosition = calculateDistance(latest, heading, info, false ^ switchXY(theta));
            double xField = (factor[0] * FieldSimulator.FIELD_SIZE / 2);
            double xDirectionalPosition = (switchXY(theta) ? factor[0] * xRelativePosition : xRelativePosition);
            xAbsolutePosition = xField - xDirectionalPosition;
        } else if (factor[1] * latest != 0) {
            yRelativePosition = calculateDistance(latest, heading, info, true ^ switchXY(theta));
            double yField = factor[1] * FieldSimulator.FIELD_SIZE / 2;
            double yDirectionalPosition = switchXY(theta) ? factor[1] * yRelativePosition : yRelativePosition;
            yAbsolutePosition = yField - yDirectionalPosition;
        }
        return new Double[]{xAbsolutePosition, yAbsolutePosition};
    }

    boolean switchXY(double theta) {
        return isEpsilonEqual(theta, Math.PI / 2) || isEpsilonEqual(theta, 3 * Math.PI / 2);
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

    public static double calculateDistance(double distance, double heading, DistanceSensorInfo
            info, boolean gettingY) {
        // Because I'm using right is positive, where RoadRunner uses left is positive
        double relative = -heading + info.getThetaOffset();

        double sensorToWall, n, m;
        if (gettingY) {
            // Distance from the sensor to the wall
            sensorToWall = distance * Math.cos(relative);

            // n := distance involving the y-value of the info
            n = info.getYOffset() * Math.cos(-heading);

            // m := distance involving the x-value of the info
            m = -info.getXOffset() * Math.sin(-heading);
        } else {
            // Distance from the sensor to the wall
            sensorToWall = distance * Math.sin(relative);

            // n := distance involving the y-value of the info
            n = -info.getYOffset() * Math.sin(-heading);

            // m := distance involving the x-value of the info
            m = info.getXOffset() * Math.cos(-heading);
        }

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

    public static double normalizeToPiOver2(double angle) {
        // Normalize to -π to π first
        angle = ((angle + Math.PI) % (2 * Math.PI)) - Math.PI;

        // Now normalize to -π/2 to π/2 by checking if angle is outside that range
        if (angle > Math.PI / 2) {
            return angle - Math.PI;  // If > π/2, subtract π to bring into range
        } else if (angle < -Math.PI / 2) {
            return angle + Math.PI;  // If < -π/2, add π to bring into range
        }

        return angle;  // Already within range
    }
    static final double EPSILON = 0.1;

    public static double[] angleToUnitVectorWithEpsilon(Double angle) {
        if (angle == null) {
            return new double[]{0, 0};
        }

        double x = Math.sin(angle);  // Calculate x-component
        double y = Math.cos(angle);  // Calculate y-component

        // Check if x is close to zero, then reset to zero
        if (Math.abs(x) < EPSILON) {
            x = 0;
        }

        // Check if y is close to zero, then reset to zero
        if (Math.abs(y) < EPSILON) {
            y = 0;
        }

        return new double[]{x, y};
    }

    public static boolean isEpsilonEqual(double a, double b) {
        return Math.abs(a - b) <= EPSILON;
    }
}
