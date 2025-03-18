/**
* MentorBot arm control.
*/
package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.RobotLog;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamMentor.roadrunner.RobotAction;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


/**
 * Joint enumerations.
 */
class Id {
    public static final int TURRET = 0;
    public static final int SHOULDER = 1;
    public static final int ELBOW1 = 2;
    public static final int ELBOW2 = 3;
    public static final int ELBOW3 = 4;
    public static final int WRIST = 5;
    public static final int CLAW = 6;
    /////////////////////////////////
    public static final int COUNT = 7;

    static final String[][] DEVICE_NAMES = {
        new String[]{"turret"},                 // Id.TURRET
        new String[]{"shoulderA", "shoulderB"}, // Id.SHOULDER
        new String[]{"elbow1"},                 // Id.ELBOW1
        new String[]{"elbow2"},                 // Id.ELBOW2
        new String[]{"elbow3"},                 // Id.ELBOW3
        new String[]{"wrist"},                  // Id.WRIST
        new String[]{"claw"}                    // Id.CLAW
    };
}

/**
 * Structure for saving and loading arm calibration data that can be tuned via the calibration
 * program.
 */
class Calibration {
    double maxSpeed; // Radians per seconds
    double acceleration; // Radians per second squared
    double deceleration; // Radians per second squared (a negative value)
    double minReach; // Minimum distance at which the arm can touch the floor, in inches
    JointCalibration[] jointCalibrations = new JointCalibration[Id.COUNT]; // Calibration for each joint
    Fudge[] fudges = new Fudge[]{}; // Measured correction factors, in sorted order of increasing distance

    // Calibration data for each joint:
    static class JointCalibration {
        double start; // Servo position when starting
        double positionA; // Position at location A, [0, 1]
        double positionB; // Position at location B, [0, 1]
        double degreesA; // Angle at location A, degrees
        double degreesB; // Angle at location B, degrees
        double min; // Minimum allowable position
        double max; // Maximum allowable position

        double radiansToPosition(double radians) {
            double deltaAngle = Math.toRadians(degreesB - degreesA);
            double deltaPosition = positionB - positionA;
            double position0 = positionA - Math.toRadians(degreesA) * deltaPosition / deltaAngle;
            return radians * deltaPosition / deltaAngle + position0;
        }
        double positionToRadians(double position) {
            double deltaAngle = Math.toRadians(degreesB - degreesA);
            double deltaPosition = positionB - positionA;
            double position0 = positionA - Math.toRadians(degreesA) * deltaPosition / deltaAngle;
            return (position - position0) * deltaAngle / deltaPosition;
        }
        boolean isValid() {
            if ((min >= max) || (min < 0) || (max > 1))
                return false;
            if ((positionA == positionB) ||
                    (start < min) || (start > max) ||
                    (positionA < min) || (positionA > max) ||
                    (positionB < min) || (positionB > max))
                return false;
            if ((degreesA == degreesB) ||
                    (degreesA < -180) || (degreesA > 180) ||
                    (degreesB < -180) || (degreesB > 180))
                return false;
            return true;
        }
    }
    // Measured fudge factor
    static class Fudge {
        double measuredDistance; // Actual distance of the arm extension when the fudge was measured
        double computedDistance; // Theoretical distance of the arm extension when the fudge was measured
        double heightCorrection; // Correction to add to the requested height
        Fudge(double measuredDistance, double computedDistance, double heightCorrection) {
            this.measuredDistance = measuredDistance;
            this.computedDistance = computedDistance;
            this.heightCorrection = heightCorrection;
        }
    }

    Calibration() {
        setDefaultKinematics();
        for (int i = 0; i < jointCalibrations.length; i++) {
            jointCalibrations[i] = new JointCalibration();
        }
    }

    // Apply some default kinematics:
    void setDefaultKinematics() {
        maxSpeed = Specs.Arm.MAX_SERVO_SPEED; // Radians per second
        acceleration = maxSpeed / 0.1; // Up to max speed in 0.1 seconds
        deceleration = -maxSpeed / 0.1; // Down to 0 in 0.1 seconds
    }

    // Apply an identity fudge to the arm, which means no correction:
    void setDefaultFudges() {
        minReach = 10;
        fudges = new Fudge[]{new Fudge(42, 42, 0)};
    }

    // Validate the calibration data:
    public boolean isValidKinematics() {
        return (maxSpeed > 0) && (acceleration > 0) && (deceleration < 0);
    }

    // Choose somewhat arbitrary default values for simulation testing:
    static Calibration getDefaultCalibration() {
        Calibration calibration = new Calibration();
        calibration.setDefaultKinematics();
        calibration.setDefaultFudges();

        for (int i = 0; i < Id.COUNT; i++) {
            JointCalibration joint = calibration.jointCalibrations[i];
            joint.min = 0.0;
            joint.max = 1.0;
            joint.start = 0.0;
            joint.positionA = 0.0;
            joint.positionB = 1.0;
            joint.degreesA = 0.0;
            joint.degreesB = 180;
        }

        calibration.jointCalibrations[Id.TURRET].degreesA = 15;
        calibration.jointCalibrations[Id.TURRET].degreesB = -15;
        calibration.jointCalibrations[Id.TURRET].start = 0.5;

        calibration.jointCalibrations[Id.ELBOW1].degreesA = 170;
        calibration.jointCalibrations[Id.ELBOW1].degreesB = -170;

        calibration.jointCalibrations[Id.ELBOW2].degreesA = -170;
        calibration.jointCalibrations[Id.ELBOW2].degreesB = 170;

        calibration.jointCalibrations[Id.ELBOW3].degreesA = 170;
        calibration.jointCalibrations[Id.ELBOW3].degreesB = -170;

        calibration.jointCalibrations[Id.WRIST].degreesA = 180;
        calibration.jointCalibrations[Id.WRIST].degreesA = -180;
        calibration.jointCalibrations[Id.CLAW].degreesA = 180;
        calibration.jointCalibrations[Id.CLAW].degreesA = -180;
        calibration.jointCalibrations[Id.CLAW].start
                = calibration.jointCalibrations[Id.CLAW].radiansToPosition(Math.toRadians(30));
        return calibration;
    }

    // Save the calibration data to a JSON file:
    public boolean saveToFile() {
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        try (FileWriter writer = new FileWriter(Specs.Arm.CALIBRATION_FILE)) {
            gson.toJson(this, writer);
        } catch (IOException e) {
            return false;
        }
        return true;
    }

    // Load the calibration data from a JSON file:
    public static Calibration loadFromFile() {
        Gson gson = new Gson();
        try (FileReader reader = new FileReader(Specs.Arm.CALIBRATION_FILE)) {
            return gson.fromJson(reader, Calibration.class);
        } catch (IOException e) {
            return null;
        }
    }
}

/**
 * A matrix for 2D affine transformations.
 */
class Xform {
    private final double[] m = new double[6]; // 2x3 matrix
    public Xform() {
        setIdentity();
    }
    public Xform(Xform other) {
        System.arraycopy(other.m, 0, m, 0, 6);
    }
    public void setIdentity() {
        m[0] = 1; m[1] = 0; m[2] = 0;
        m[3] = 0; m[4] = 1; m[5] = 0;
    }
    public void translate(double tx, double ty) {
        m[2] += m[0] * tx + m[1] * ty;
        m[5] += m[3] * tx + m[4] * ty;
    }
    public void rotate(double angle) { // Angle in radians
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double[] newM = new double[6];

        newM[0] = m[0] * cos + m[1] * sin;
        newM[1] = m[0] * -sin + m[1] * cos;
        newM[2] = m[2];

        newM[3] = m[3] * cos + m[4] * sin;
        newM[4] = m[3] * -sin + m[4] * cos;
        newM[5] = m[5];

        System.arraycopy(newM, 0, m, 0, 6);
    }
    public void scale(double sx, double sy) {
        m[0] *= sx;
        m[1] *= sy;
        m[3] *= sx;
        m[4] *= sy;
    }
    public Point xform(double x, double y) {
        return new Point(m[0] * x + m[1] * y + m[2],
                m[3] * x + m[4] * y + m[5]);
    }
    public double[] xform(double[] points) {
        double[] result = new double[points.length];
        for (int i = 0; i < points.length; i += 2) {
            result[i] = m[0] * points[i] + m[1] * points[i+1] + m[2];
            result[i+1] = m[3] * points[i] + m[4] * points[i+1] + m[5];
        }
        return result;
    }
    static public double[] getX(double[] xformedPoints) {
        double[] result = new double[xformedPoints.length / 2];
        for (int i = 0; i < xformedPoints.length / 2; i++) {
            result[i] = xformedPoints[2*i];
        }
        return result;
    }
    static public double[] getY(double[] xformedPoints) {
        double[] result = new double[xformedPoints.length / 2];
        for (int i = 0; i < xformedPoints.length / 2; i++) {
            result[i] = xformedPoints[2*i + 1];
        }
        return result;
    }
}

// Calculate the bounds of a set of points.
class Bounds {
    double xMin, xMax, yMin, yMax;
    Bounds() {
        this.xMin = Double.MAX_VALUE;
        this.xMax = -Double.MAX_VALUE;
        this.yMin = Double.MAX_VALUE;
        this.yMax = -Double.MAX_VALUE;
    }
    void update(double x, double y) {
        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;
        if (y < yMin) yMin = y;
        if (y > yMax) yMax = y;
    }
    void update(double[] points) {
        for (int i = 0; i < points.length; i += 2) {
            update(points[i], points[i+1]);
        }
    }
}

/**
 * Model the arm for fine bounds accumulation and visualization.
 */
class Model {
    Telemetry telemetry; // Driver station telemetry
    Bounds topBounds; // Accumulated top-view bounds for the arm
    Bounds sideBounds; // Accumulated side-view bounds for the arm
    double[] hardwareAngles = new double[Id.COUNT]; // In radians
    double xClawTarget, yClawTarget; // Target position for the claw
    Point computedPickupPoint; // Focal point computed from arm geometry, for verification
    double computedArmLength; // Length of the arm, for verification

    // Initialize the arm model.
    Model(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Set the target position of the specified joint.
    void setHardwarePosition(int id, Calibration.JointCalibration calibration, double position) {
        // Convert the position back to a radians angle:
        hardwareAngles[id] = calibration.positionToRadians(position);
    }

    // Set the pickup target visualization, 0 to disable
    void setPickupTarget(double x, double y) {
        xClawTarget = x;
        yClawTarget = y;
    }

    // Draw a basket at the given height.
    static void drawBasket(Canvas canvas, double height) {
        Xform basketXform = new Xform();
        basketXform.translate(18, height);
        basketXform.rotate(Math.atan(.75 / 5.75));
        double taper = (5.75 - 3.5) / 2;
        double[] highBasket = basketXform.xform(new double[]
                {0, 0,  5.75, 0,  5.75-taper, -6.25,   taper, -6.25,   0, 0});
        canvas.setFill("#ffffff");
        canvas.setStroke("#000000");
        canvas.setStrokeWidth(0);
        canvas.fillPolygon(Xform.getX(highBasket), Xform.getY(highBasket));
        canvas.strokePolyline(Xform.getX(highBasket), Xform.getY(highBasket));
    }

    // Draw the the field behind the robot.
    void drawBackground(Canvas topCanvas, Canvas sideCanvas) {
        // Draw the floor of the field for the side view:
        double[] floor = new double[]{-16, 0, 16, 0, 16, -2, -16, -2, -16, 0};
        sideCanvas.setFill("#ffffff");
        sideCanvas.setStroke("#000000");
        sideCanvas.setStrokeWidth(0);
        sideCanvas.fillPolygon(Xform.getX(floor), Xform.getY(floor));
        sideCanvas.strokePolyline(Xform.getX(floor), Xform.getY(floor));

        // Draw the two baskets:
        drawBasket(sideCanvas, 42.75); // 42.75 inches from floor
        drawBasket(sideCanvas, 25.5);  // 25.5 inches from floor

        // Draw the low and middle ascent bars:
        double x = 12;
        double[] lowBar = new double[]{x, 1.25, x, 2.25, x+1, 2.25, x+1, 1.25, x, 1.25};
        sideCanvas.fillPolygon(Xform.getX(lowBar), Xform.getY(lowBar));
        sideCanvas.strokePolyline(Xform.getX(lowBar), Xform.getY(lowBar));
        Point circle = new Point(x, 19.875);
        sideCanvas.fillCircle(circle.x, circle.y, 0.5);
        sideCanvas.strokeCircle(circle.x, circle.y, 0.5);

        // Draw the low specimen bar for comparison:
        double[] specimenBar = new double[]{x, 12.25, x, 13.25, x+1, 13.25, x+1, 12.25, x, 12.25};
        sideCanvas.setFill("#c0c0c0");
        sideCanvas.fillPolygon(Xform.getX(specimenBar), Xform.getY(specimenBar));
    }

    // Draw the bits that go on top of the arm
    void drawForeground(Canvas topCanvas, Canvas sideCanvas) {
        // The chassis side view:
        double halfLength = WilyConfig.ROBOT_LENGTH / 2;
        double[] chassis = new double[]{-halfLength, 2.8,  halfLength, 2.8,  halfLength, 0.9,  -halfLength, 0.9,  -halfLength, 2.8};
        sideCanvas.setFill("#c0c0c0");
        sideCanvas.fillPolygon(Xform.getX(chassis), Xform.getY(chassis));

        // Now the wheels:
        Point wheelCenter1 = new Point(-halfLength + 2.5, 3.75 / 2);
        Point wheelCenter2 = new Point(halfLength - 2.5, 3.75 / 2);
        sideCanvas.setFill("#000000");
        sideCanvas.fillCircle(wheelCenter1.x, wheelCenter1.y, 3.75 / 2);
        sideCanvas.fillCircle(wheelCenter2.x, wheelCenter2.y, 3.75 / 2);

        // Draw the claw target on top:
        if (xClawTarget != 0) {
            Point point = new Point(Specs.Arm.TURRET_OFFSET.x + xClawTarget, Specs.Arm.TURRET_OFFSET.y + yClawTarget); // @@@ Is this right?
            sideCanvas.setFill("#00ff00");
            sideCanvas.fillCircle(point.x, point.y, 1);
        }

        // Draw the accumulated bounds:
        sideCanvas.setStroke("#ff0000");
        if ((sideBounds.xMin <= sideBounds.xMax) && (sideBounds.yMin <= sideBounds.yMax)) {
            sideCanvas.strokeRect(sideBounds.xMin, sideBounds.yMin,
                    sideBounds.xMax - sideBounds.xMin,
                    sideBounds.yMax - sideBounds.yMin);
        }
    }

    // Draw an arm segment.
    void drawArmSegment(Canvas topCanvas, Xform topXform, Canvas sideCanvas, Xform sideXform, double length, int index) {
        String color = new String[]{"#a1b3be", "#79afa5", "#51aa8c", "#2aa673"}[index];
        topCanvas.setFill(color);
        sideCanvas.setFill(color);

        double x0 = -Specs.Arm.SEGMENT_OVERHANG;
        double x1 = length + Specs.Arm.SEGMENT_OVERHANG;
        double y = Specs.Arm.SEGMENT_WIDTH / 2;
        double[] geometry = new double[]{x0, y, x1, y, x1, -y, x0, -y, x0, y};

        double[] sidePoints = sideXform.xform(geometry);
        sideCanvas.fillPolygon(Xform.getX(sidePoints), Xform.getY(sidePoints));

        double[] topPoints = topXform.xform(geometry);
        topCanvas.fillPolygon(Xform.getX(topPoints), Xform.getY(topPoints));

        // Hack some end geometry for when segments are vertical:
        Point point = topXform.xform(0, 0);
        topCanvas.fillRect(point.x - Specs.Arm.SEGMENT_WIDTH /2, point.y - Specs.Arm.SEGMENT_WIDTH /2,
                Specs.Arm.SEGMENT_WIDTH, Specs.Arm.SEGMENT_WIDTH);
    }

    // Draw a clasp of the claw.
    void drawClasp(Canvas topCanvas, Xform topXformOriginal, Canvas sideCanvas, Xform sideXformOriginal, double yScale, double zAngle, double clawAngle, double wristAngle) {
        Xform topXform = new Xform(topXformOriginal);
        Xform sideXform = new Xform(sideXformOriginal);

        topXform.rotate(wristAngle);
        topXform.scale(Math.sin(clawAngle), yScale);
        sideXform.scale(1, yScale * Math.abs(Math.cos(wristAngle)));
        sideXform.rotate(clawAngle);

        double[] sidePoints = sideXform.xform(new double[]{0, 0.5, 2, 1.5, 4, 0.5});
        sideCanvas.setStroke("#000000");
        sideCanvas.setStrokeWidth(1);
        sideCanvas.strokePolyline(Xform.getX(sidePoints), Xform.getY(sidePoints));

        double[] topPoints = topXform.xform(new double[]{-4, 0, 4, 0});
        topCanvas.setStroke("#000000");
        topCanvas.setStrokeWidth(1);
        topCanvas.strokePolyline(Xform.getX(topPoints), Xform.getY(topPoints));
    }

    // Create a new transform with the x-axis scaled by the cosine of the Z angle.
    Xform xScale(Xform xform, double zAngle) {
        Xform result = new Xform(xform);
        result.scale(Math.cos(zAngle), 1);
        return result;
    }

    // Set a translation and rotation on the canvas, adjusting by 90 degrees to account for
    // Dashboard's goofy convention:
    void setTranslationRotation(Canvas canvas, double x, double y, double angle) {
        canvas.setTranslation(y, -x);
        canvas.setRotation(angle - Math.toRadians(90));
    }

    // Update the geometry of the model, get the bounds, and draw the arm.
    void update(Canvas canvas, Pose2d pose) {
        // The top view is a new canvas. It will be drawn on top of the side view.
        Canvas topCanvas = new Canvas();
        setTranslationRotation(topCanvas, pose.position.x, pose.position.y, pose.heading.log());

        // The side view is the regular canvas:
        Canvas sideCanvas = canvas;
        setTranslationRotation(sideCanvas, 0, 24, 0);

        Xform sideXform = new Xform();
        Xform topXform = new Xform();
        double zAngle = 0;

        topBounds = new Bounds();
        sideBounds = new Bounds();

        // Draw the field background:
        drawBackground(topCanvas, sideCanvas);

        // Shoulder base:
        Point shoulderBase = Specs.Arm.SHOULDER_OFFSET.rotate(hardwareAngles[Id.TURRET]).add(Specs.Arm.TURRET_OFFSET);
        sideXform.translate(shoulderBase.x, Specs.Arm.SHOULDER_HEIGHT);
        sideXform.scale(Math.cos(hardwareAngles[Id.TURRET]), 1);

        topXform.translate(shoulderBase.x, shoulderBase.y);
        topXform.rotate(hardwareAngles[Id.TURRET]);

        // Arm segment #1:
        zAngle += hardwareAngles[Id.SHOULDER];

        sideXform.rotate(hardwareAngles[Id.SHOULDER]);
        drawArmSegment(topCanvas, xScale(topXform, zAngle), sideCanvas, sideXform, Specs.Arm.SEGMENT_LENGTH, 0);
        sideXform.translate(Specs.Arm.SEGMENT_LENGTH, 0);
        topXform.translate(Specs.Arm.SEGMENT_LENGTH * Math.cos(zAngle), -Specs.Arm.SEGMENT_WIDTH);

        // Arm segment #2:
        zAngle += hardwareAngles[Id.ELBOW1];
        sideXform.rotate(hardwareAngles[Id.ELBOW1]);
        drawArmSegment(topCanvas, xScale(topXform, zAngle), sideCanvas, sideXform, Specs.Arm.SEGMENT_LENGTH, 1);
        sideXform.translate(Specs.Arm.SEGMENT_LENGTH, 0);
        topXform.translate(Specs.Arm.SEGMENT_LENGTH * Math.cos(zAngle), Specs.Arm.SEGMENT_WIDTH);

        // Arm segment #3:
        zAngle += hardwareAngles[Id.ELBOW2];
        sideXform.rotate(hardwareAngles[Id.ELBOW2]);
        drawArmSegment(topCanvas, xScale(topXform, zAngle), sideCanvas, sideXform, Specs.Arm.SEGMENT_LENGTH, 2);
        sideXform.translate(Specs.Arm.SEGMENT_LENGTH, 0);
        topXform.translate(Specs.Arm.SEGMENT_LENGTH * Math.cos(zAngle), -Specs.Arm.SEGMENT_WIDTH);

        // Arm segment #4 with the claw on the end:
        zAngle += hardwareAngles[Id.ELBOW3];
        sideXform.rotate(hardwareAngles[Id.ELBOW3]);
        drawArmSegment(topCanvas, xScale(topXform, zAngle), sideCanvas, sideXform, Specs.Arm.LAST_SEGMENT_LENGTH, 3);
        sideXform.translate(Specs.Arm.LAST_SEGMENT_LENGTH, 0);
        topXform.translate(Specs.Arm.LAST_SEGMENT_LENGTH * Math.cos(zAngle), 0); // Don't offset by width

        // The clasps of the claw:
        drawClasp(topCanvas, topXform, sideCanvas, sideXform, 1, zAngle, hardwareAngles[Id.CLAW], hardwareAngles[Id.WRIST]);
        drawClasp(topCanvas, topXform, sideCanvas, sideXform, -1, zAngle, hardwareAngles[Id.CLAW], hardwareAngles[Id.WRIST]);

        // Compute the pickup point in field space for verification:
        Point clawPoint = topXform.xform(0, 0);
        computedPickupPoint = clawPoint.rotate(pose.heading.log()).add(new Point(pose.position));
        computedArmLength = shoulderBase.distanceTo(clawPoint);

        // Draw the top bits:
        drawForeground(topCanvas, sideCanvas);

        // Undo the canvas transforms, remembering that Dashboard's transforms are not
        // cumulative:
        sideCanvas.setTranslation(0, 0);
        topCanvas.setRotation(0);
        topCanvas.setTranslation(0, 0);

        // Combine the side and top views into one canvas:
        canvas.getOperations().addAll(topCanvas.getOperations());
    }
}

/**
 * Encapsulation of arm control.
 */
class Arm {
    final Telemetry telemetry; // Driver station telemetry
    final Model model; // Model for visualization and bounds accumulation
    final Calibration calibration; // Calibration data

    Joint[] joints = new Joint[Id.COUNT]; // Joint states
    double previousUpdateTime; // Time of previous update call, in seconds
    boolean sloMo = false; // Slow motion mode

    // Per-joint state:
    static class Joint {
        int id; // Joint ID
        Calibration.JointCalibration calibration; // Calibration data for this joint
        Servo[] servos; // Servo controlling this joint
        double currentAngle; // Current angle, radians
        double targetAngle; // Target angle, radians
        double currentVelocity; // Current angular velocity, radians per second

        double radiansToPosition(double radians) {
            return calibration.radiansToPosition(radians);
        }
        double positionToRadians(double position) {
            return calibration.positionToRadians(position);
        }
        double homeInRadians() {
            return positionToRadians(calibration.start);
        }

        // Set the target position of the joint. Returns true if the joint is at the target.
        boolean setTarget(double target) {
            final double EPSILON = 0.01;

            // Clamp the target to the allowable range. Note that the min position may actually
            // be the maximum angle:
            double end1 = positionToRadians(calibration.min);
            double end2 = positionToRadians(calibration.max);
            double minAngle = Math.min(end1, end2);
            double maxAngle = Math.max(end1, end2);
            targetAngle = Math.min(maxAngle, Math.max(minAngle, target));
            return Math.abs(targetAngle - currentAngle) < EPSILON;
        }

        // Disable the joint while ensuring no divide-by-zeroes or other crashes from invalid
        // calibration data:
        void disable() {
            servos = new Servo[]{}; // Disable this joint by nulling the servos
            calibration.degreesA = -180;
            calibration.degreesB = 180;
            calibration.positionA = 0;
            calibration.positionB = 1;
            calibration.min = 0;
            calibration.max = 1;
        }

        // Return true if at the target angle:
        boolean atTarget() {
            final double EPSILON = 0.001;
            return Math.abs(targetAngle - currentAngle) < EPSILON;
        }

        // Set the angle of the joint. For motors, the units are ticks. For servos, the units are
        // normalized to the range [0, 1].
        void setHardwarePosition(double position) {
            // Crash if the position isn't within allowable range:
            if ((position < calibration.min) || (position > calibration.max)) {
                throw new IllegalArgumentException("Joint position out of range");
            }
            for (Servo servo: servos) {
                Stats.beginIo();
                servo.setPosition(position);
                Stats.endIo();
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.model = new Model(telemetry); // Model for visualization and bounds accumulation

        Calibration loadedCalibration = Calibration.loadFromFile();
        if (WilyWorks.isSimulating) {
            // calibration = Calibration.getDefaultCalibration();
            calibration = loadedCalibration; // @@@
        } else if (loadedCalibration != null) {
            calibration = loadedCalibration;
        } else {
            RobotLog.addGlobalWarningMessage("Arm is disabled: couldn't load calibration!");
            calibration = new Calibration();
        }

        // Ensure that the calibration allows the program to run without crashing.
        if (!calibration.isValidKinematics()) {
            RobotLog.addGlobalWarningMessage("Invalid arm kinematics");
            calibration.setDefaultKinematics();
        }

        for (int id = 0; id < Id.COUNT; id++) {
            int servoCount = Id.DEVICE_NAMES[id].length;

            Joint joint = new Joint();
            joint.id = id;
            joint.servos = new Servo[servoCount];
            for (int servoIndex = 0; servoIndex < servoCount; servoIndex++) {
                joint.servos[servoIndex] = hardwareMap.tryGet(Servo.class, Id.DEVICE_NAMES[id][servoIndex]);
                if (joint.servos[servoIndex] == null) {
                    if (!MecanumDrive.isDevBot)
                        RobotLog.addGlobalWarningMessage("Missing servo in RC configuration: " + Id.DEVICE_NAMES[id][servoIndex]);
                    joint.servos = new Servo[]{}; // Disable this joint by nulling the servos
                    break; // ====>
                }
            }
            if (calibration.jointCalibrations[id].isValid()) {
                joint.calibration = calibration.jointCalibrations[id];
            } else {
                if (!MecanumDrive.isDevBot)
                    // Disable the joint if the calibration is invalid and ensure no divide-by-zeroes:
                    RobotLog.addGlobalWarningMessage("Invalid servo calibration: " + Id.DEVICE_NAMES[joint.id][0]);

                joint.servos = new Servo[]{}; // Disable this joint by nulling the servos
                joint.calibration = Calibration.getDefaultCalibration().jointCalibrations[id];
            }
            joint.currentAngle = joint.homeInRadians();
            joint.targetAngle = joint.homeInRadians();
            joints[id] = joint;
        }
    }

    // Move all of the joints according to the target velocity and update the simulator.
    void update(Pose2d pose, Canvas canvas) {
        double time = nanoTime() / 1e9f; // Time in seconds
        double dt = 0;
        if (previousUpdateTime != 0) {
            dt = time - previousUpdateTime; // Delta-time
        }
        previousUpdateTime = time;

        for (Joint joint: joints) {
            double delta = joint.targetAngle - joint.currentAngle; // Do not normalize!
            double signum = Math.signum(delta);
            double distance = Math.abs(delta);
            double velocity = signum * joint.currentVelocity;
            double maxSpeed = calibration.maxSpeed;
            double acceleration = calibration.acceleration;
            double deceleration = calibration.deceleration;

            if (sloMo) {
                // Slow down the arm for testing
                maxSpeed *= 0.1;
                acceleration *= 0.1;
                deceleration *= 0.1;
            }
            if (joint.id == Id.SHOULDER) {
                maxSpeed *= 0.5; // Slow down the shoulder as a hack
            }

            double increasingVelocity = velocity + acceleration * dt; // Speed if we accelerate
            double decreasingVelocity = Math.sqrt(-2 * deceleration * distance); // Speed if we decelerate
            double newSpeed = Math.min(maxSpeed, Math.min(increasingVelocity, decreasingVelocity));

            double angleDelta = Math.min(newSpeed * dt, distance);
            joint.currentVelocity = signum * newSpeed;
            joint.currentAngle += signum * angleDelta;

            // LERP to convert to ticks/position:
            double position = joint.radiansToPosition(joint.currentAngle);

            // Saturate to the allowed range:
            if (position < joint.calibration.min)
                position = joint.calibration.min;
            if (position > joint.calibration.max)
                position = joint.calibration.max;

            joint.setHardwarePosition(position);
            model.setHardwarePosition(joint.id, joint.calibration, position);
        }
        model.update(canvas, pose);
    }

    // Compute the theoretical joint angles for a given reach. Returns null if the reach is out of
    // range.
    static double[] computeTheoreticalReach(double distance, double height) {
        // Within this class, height is considered relative to the base of the arm, but
        // user expects the height to be relative to the floor. Adjust the height accordingly:
        height -= Specs.Arm.SHOULDER_HEIGHT;

        double alpha = Math.asin((Specs.Arm.LAST_SEGMENT_LENGTH + Specs.Arm.CLAW_OFFSET + height) / Specs.Arm.SEGMENT_LENGTH);
        double a = Specs.Arm.SEGMENT_LENGTH * Math.cos(alpha);
        double b = distance - a;
        double beta = Math.asin(b / (2 * Specs.Arm.SEGMENT_LENGTH));

        // Failure if the request is out of range (which results in NaNs):
        if (Double.isNaN(alpha) || Double.isNaN(beta))
            return null; // ====>
        return new double[] { alpha, beta };
    }

    // Compute the joint angles for a given reach, taking measured fudge factors into account.
    double[] computeCorrectedReach(double distance, double height) {
        Calibration.Fudge lowerFudge = new Calibration.Fudge(0, 0, 0);
        Calibration.Fudge upperFudge = null;
        for (Calibration.Fudge fudge : calibration.fudges) {
            if (fudge.measuredDistance <= distance) {
                lowerFudge = fudge; // Keep the last fudge factor that is less than or equal to the distance
            } else {
                upperFudge = fudge; // The first fudge factor that is greater than the distance
                break; // Fudge factors are sorted by distance
            }
        }

        // The last fudge has to be measured at the full extent of the arm, so if this request
        // exceeds that, the request is not possible:
        if (upperFudge == null) {
            return null;
        }

        // LERP between the two fudge factors:
        double distanceDelta = upperFudge.measuredDistance - lowerFudge.measuredDistance;
        double correctedDistance = lowerFudge.computedDistance +
                (upperFudge.computedDistance - lowerFudge.computedDistance) *
                (distance - lowerFudge.measuredDistance) / distanceDelta;

        double heightCorrectionDelta = upperFudge.heightCorrection - lowerFudge.heightCorrection;
        double heightCorrection = lowerFudge.heightCorrection +
                (heightCorrectionDelta * (distance - lowerFudge.measuredDistance) / distanceDelta);
        double correctedHeight = height + heightCorrection;

        // Compute the angles based on the corrected distance and height:
        return computeTheoreticalReach(correctedDistance, correctedHeight);
    }

    void halt() {
        joints[Id.SHOULDER].targetAngle = joints[Id.SHOULDER].currentAngle;
        joints[Id.ELBOW1].targetAngle = joints[Id.ELBOW1].currentAngle;
        joints[Id.ELBOW2].targetAngle = joints[Id.ELBOW2].currentAngle;
        joints[Id.ELBOW3].targetAngle = joints[Id.ELBOW3].currentAngle;
    }
    boolean start() {
        return joints[Id.SHOULDER].setTarget(joints[Id.SHOULDER].homeInRadians()) &
                joints[Id.ELBOW1].setTarget(joints[Id.ELBOW1].homeInRadians()) &
                joints[Id.ELBOW2].setTarget(joints[Id.ELBOW2].homeInRadians()) &
                joints[Id.ELBOW3].setTarget(joints[Id.ELBOW3].homeInRadians());
    }
    boolean home() {
        return joints[Id.SHOULDER].setTarget(Math.toRadians(90)) &
                joints[Id.ELBOW1].setTarget(Math.toRadians(-170)) &
                joints[Id.ELBOW2].setTarget(Math.toRadians(170)) &
                joints[Id.ELBOW3].setTarget(Math.toRadians(-90));
    }
    boolean highBasket() {
        boolean done = joints[Id.SHOULDER].setTarget(Math.toRadians(70));
        // Only once the shoulder is in position do we move the elbows:
        if (done) {
            done = joints[Id.ELBOW1].setTarget(Math.toRadians(0)) &
                    joints[Id.ELBOW2].setTarget(Math.toRadians(0)) &
                    joints[Id.ELBOW3].setTarget(Math.toRadians(-70));
        }
        return done;
    }
    boolean pickup(double distance, double height) {
        double[] angles = computeCorrectedReach(distance, height);
        if (angles == null)
            return true; // An impossible configuration was requested so we're already done

        double alpha = angles[0];
        double beta = angles[1];
        return joints[Id.SHOULDER].setTarget(Math.PI / 2 - beta) &
                joints[Id.ELBOW1].setTarget(-Math.PI + 2 * beta) &
                joints[Id.ELBOW2].setTarget(Math.PI - beta - Math.PI / 2 + alpha) &
                joints[Id.ELBOW3].setTarget(-Math.PI + (Math.PI / 2 - alpha));
    }
    boolean wrist(double angle) { // Radians
        return joints[Id.WRIST].setTarget(angle);
    }
    boolean turret(double angle) { // Radians
        return joints[Id.TURRET].setTarget(angle);
    }
    boolean claw(boolean open) {
        return joints[Id.CLAW].setTarget(open ? Math.toRadians(90) : Math.toRadians(15));
    }
}

/**
 * Action to control the turret.
 */
class TurretAction extends RobotAction {
    Arm arm;
    double value;
    int thisInstance; // Identifier of this instance
    static int activeInstance = 0; // Identifier of the active action

    TurretAction(Arm arm, double value) {
        this.arm = arm;
        this.value = value;
    }

    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            thisInstance = ++activeInstance;
        }
        if (thisInstance != activeInstance)
            return false; // This action has been superseded by another
        return !arm.joints[Id.TURRET].setTarget(value);
    }
}

/**
 * Action to control the wrist joint.
 */
class WristAction extends RobotAction {
    Arm arm;
    double angle;
    int thisInstance; // Identifier of this instance
    static int activeInstance = 0; // Identifier of the active action

    WristAction(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
    }

    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            thisInstance = ++activeInstance;
        }
        if (thisInstance != activeInstance)
            return false; // This action has been superseded by another
        return !arm.joints[Id.WRIST].setTarget(Math.toRadians(angle));
    }
}


/**
 * Action to control the claw by opening or closing it.
 */
class ClawAction extends RobotAction {
    Arm arm;
    boolean open;
    int thisInstance; // Identifier of this instance
    static int activeInstance = 0; // Identifier of the active action

    ClawAction(Arm arm, boolean open) {
        this.arm = arm;
        this.open = open;
    }
    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            thisInstance = ++activeInstance;
        }
        if (thisInstance != activeInstance)
            return false; // This action has been superseded by another
        return !arm.joints[Id.CLAW].setTarget(open ? Math.toRadians(90) : Math.toRadians(15));
    }
}

/**
 * Action to control the arm's reach by operating the shoulder and elbow joints.
 */
class ReachAction extends RobotAction {
    enum State {
        PICKUP,
        HIGH_BASKET,
        HOME,
        START
    }
    enum Geometry {
        VERTICAL,
        HORIZONTAL,
        HOME
    }

    final Arm arm; // The arm to control
    final State state; // Requested reach action
    final double distance; // Distance to reach
    final double height; // Height to reach

    int thisInstance; // Identifier of this instance

    static int activeInstance = 0; // Identifier of the active action
    static Geometry geometry = Geometry.HOME; // The arm's current geometry

    ReachAction(Arm arm, State state) { this(arm, state, 0, 0); }
    ReachAction(Arm arm, State state, double distance, double height) {
        this.arm = arm;
        this.state = state;
        this.distance = distance;
        this.height = height;
    }

    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            thisInstance = ++activeInstance;
        }
        if (thisInstance != activeInstance)
            return false; // This action has been superseded by another

        // Note that bitwise AND is used to ensure all joints are set before returning.
        boolean done = false;
        if (state == State.HOME) {
            arm.model.setPickupTarget(0, 0);
            done = arm.home();
            if (done)
                geometry = Geometry.HOME;
        } else if (state == State.PICKUP) {
            if (geometry == Geometry.VERTICAL) {
                if (arm.home())
                    geometry = Geometry.HOME;
            } else {
                arm.model.setPickupTarget(distance, height);
                done = arm.pickup(distance, height);
                geometry = Geometry.HORIZONTAL;
            }
        } else if (state == State.HIGH_BASKET) {
            if (geometry == Geometry.HORIZONTAL) {
                if (arm.home())
                    geometry = Geometry.HOME;
            } else {
                arm.model.setPickupTarget(0, 0);
                done = arm.highBasket();
                geometry = Geometry.VERTICAL;
            }
        } else if (state == State.START) {
            if (geometry != Geometry.HOME) {
                if (arm.home())
                    geometry = Geometry.HOME;
            } else {
                done = arm.start();
            }
        }
        return !done; // Return true if more calls are needed
    }
}
