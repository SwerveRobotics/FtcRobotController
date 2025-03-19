package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamMentor.roadrunner.RobotAction;

import java.util.function.Supplier;

// Auto-only drive-to action.
class DriveToAction extends RobotAction {
    final Autopilot autoPilot;
    final Pose2d target;
    DriveToAction(Autopilot autoPilot, Pose2d target) {
        this.autoPilot = autoPilot;
        this.target = target;
    }

    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            autoPilot.setTarget(target, AutoPilotState.NONE, false);
        }
        return autoPilot.update(telemetryPacket.fieldOverlay());
    }
}

// Base class for auto-pilot actions.
abstract class AutoPilotAction extends RobotAction {
    Vector2d userVelocity = new Vector2d(0, 0);
    void setUserVelocity(Vector2d userVelocity) {
        this.userVelocity = userVelocity;
    }
}

// Operate the arm while driving to the basket.
class BasketAutoPilotAction extends AutoPilotAction {
    final Arm arm;
    final Autopilot autoPilot;
    boolean raised; // True if the arm has been raised

    BasketAutoPilotAction(Arm arm, Autopilot autoPilot) {
        this.arm = arm;
        this.autoPilot = autoPilot;
    }
    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            // Start the action:
            autoPilot.setTarget(new Pose2d(-54, -54, Math.toRadians(225)), AutoPilotState.BASKET, false);
            subActions.add(new ReachAction(arm, ReachAction.State.HOME));
            subActions.add(new TurretAction(arm, 0));
            raised = false;
        }
        if ((!raised) && (autoPilot.distanceToFinalTarget() < 24)) {
            subActions.add(new ReachAction(arm, ReachAction.State.HIGH_BASKET));
            raised = true;
        }
        return autoPilot.update(telemetryPacket.fieldOverlay(), userVelocity);
    }
}

// Operate the arm while driving to the submersible and picking stuff up.
class SubmersibleAutoPilotAction extends AutoPilotAction {
    final Telemetry telemetry;
    final Arm arm;
    final Autopilot autoPilot;
    final Supplier<Point> locationCallback; // Callback to get the latest focus point from the UI
    final double minTurretAngle; // Minimum possible turret angle
    final double maxTurretAngle; // Maximum possible turret angle

    SubmersibleAutoPilotAction(Telemetry telemetry, Arm arm, Autopilot autoPilot, Supplier<Point> locationCallback) {
        this.telemetry = telemetry;
        this.arm = arm;
        this.autoPilot = autoPilot;
        this.locationCallback = locationCallback;

        // Compute the turret's minimum and maximum angles:
        Calibration.JointCalibration calibration = arm.joints[Id.TURRET].calibration;
        double angle1 = calibration.positionToRadians(calibration.min);
        double angle2 = calibration.positionToRadians(calibration.max);
        minTurretAngle = Math.min(angle1, angle2);
        maxTurretAngle = Math.max(angle1, angle2);
    }

    // Set the target pose for the robot so that it can reach to the submersible's (x, y)
    // focus point specified by the user. Returns the arm length to the focus point.
    double[] setTargetPose() {
        final double X_ABUTTING = Specs.Field.SUBMERSIBLE_X - Specs.Robot.LENGTH/2; // Robot's x when abutting
        final double MIN_X_GAP = 3; // When not abutting, minimum x gap between the robot and the submersible
        final double MIN_Y_GAP = 3; // Minimum y gap between the robot and the submersible

        // Compute the position for the robot, assuming it's not too close to the upper or
        // lower bounds. Proportionally scale the x location so that it accounts for the arm's
        // minimum reach.
        Point focusPoint = locationCallback.get();

        // Set the common case for pickup where the robot abuts the submersible wall and no
        // rotation is needed:
        double robotHeading = 0;
        double turretAngle = 0;
        boolean abut = true;
        Point robotCenter = new Point(X_ABUTTING,
                focusPoint.y - Specs.Arm.TURRET_OFFSET.y - Specs.Arm.SHOULDER_OFFSET.y - Specs.Arm.CLAW_Y_OFFSET);

        // Compute the x coordinate of the shoulder:
        double xShoulder = robotCenter.x + Specs.Arm.TURRET_OFFSET.x + Specs.Arm.SHOULDER_OFFSET.x;
        if (focusPoint.x - xShoulder < arm.calibration.minReach()) {
            abut = false;
            xShoulder = focusPoint.x - arm.calibration.minReach() - MIN_X_GAP;
            robotCenter = new Point(xShoulder - Specs.Arm.TURRET_OFFSET.x - Specs.Arm.SHOULDER_OFFSET.x,
                    robotCenter.y);
        }

        // Use symmetry to handle the logic of being close to the upper or lower edges of the
        // submersible:
        double flipY = 1;
        double maxAngle = maxTurretAngle;
        if (robotCenter.y < 0) {
            robotHeading = -robotHeading;
            robotCenter = new Point(robotCenter.x, -robotCenter.y);
            focusPoint = new Point(focusPoint.x, -focusPoint.y);
            maxAngle = -minTurretAngle;
            flipY = -1;
        }

        // Check if the turret needs to rotate to reach the focus point:
        double rotateLine = Specs.Field.SUBMERSIBLE_Y - MIN_Y_GAP - Specs.Robot.WIDTH/2;
        if (robotCenter.y > rotateLine) {
            robotCenter = new Point(robotCenter.x, rotateLine);

            Point turretPoint = Specs.Arm.TURRET_OFFSET.rotate(robotHeading).add(robotCenter);
            double distanceFromTurret = focusPoint.subtract(turretPoint).length();
            double angleFromTurret = focusPoint.subtract(turretPoint).atan2();
            turretAngle = angleFromTurret - flipY * Math.asin((Specs.Arm.SHOULDER_OFFSET.y + Specs.Arm.CLAW_Y_OFFSET)
                    / distanceFromTurret);

            // If the turret angle is out of bounds, we'll have to rotate the robot:
            if (turretAngle > maxAngle) {
                turretAngle = maxAngle;
                abut = false; // The robot has to rotate so we can't abut

                // We have to turn, so make sure we're not too close to the submersible wall:
                double maxX = Specs.Field.SUBMERSIBLE_X - Specs.Robot.LENGTH/2 - MIN_X_GAP;
                if (robotCenter.x > maxX) {
                    robotCenter = new Point(maxX, robotCenter.y);
                }

                double distanceFromCenter = focusPoint.subtract(robotCenter).length();
                double angleFromCenter = focusPoint.subtract(robotCenter).atan2();
                Point shoulderOffset = Specs.Arm.SHOULDER_OFFSET.rotate(turretAngle).add(Specs.Arm.TURRET_OFFSET);
                double run = distanceFromCenter - shoulderOffset.x;
                double rise = shoulderOffset.y + Specs.Arm.CLAW_Y_OFFSET;
                robotHeading = angleFromCenter - turretAngle - flipY * Math.atan2(rise, run);
            }
        }

        if (flipY < 0) {
            robotHeading = -robotHeading;
            robotCenter = new Point(robotCenter.x, -robotCenter.y);
            turretAngle = -turretAngle;
            focusPoint = new Point(focusPoint.x, -focusPoint.y);
        }

        // Tell autopilot to drive to our target pose:
        autoPilot.setTarget(new Pose2d(robotCenter.x, robotCenter.y, robotHeading), AutoPilotState.PICKUP, abut);
        if (!autoPilot.update(telemetryPacket.fieldOverlay(), userVelocity)) {
            // The auto-pilot has decided to stop the robot. The robot's pose will not precisely
            // match the pose that we requested. Accommodate for such error by adjusting the
            // arm length and turret angle to compensate:
            Pose2d pose = autoPilot.poser.getPose(); // The robot's actual pose
            robotHeading = pose.heading.log();
            robotCenter = new Point(pose.position);

            Point turretPoint = Specs.Arm.TURRET_OFFSET.rotate(robotHeading).add(robotCenter);
            double distanceFromTurret = focusPoint.subtract(turretPoint).length();
            double angleFromTurret = focusPoint.subtract(turretPoint).atan2();
            turretAngle = angleFromTurret - Math.asin((Specs.Arm.SHOULDER_OFFSET.y + Specs.Arm.CLAW_Y_OFFSET)
                    / distanceFromTurret);
            turretAngle = Math.min(maxTurretAngle, Math.max(minTurretAngle, turretAngle));
        }

        Point turretPoint = Specs.Arm.TURRET_OFFSET.rotate(robotHeading).add(robotCenter);
        Point shoulderPoint = Specs.Arm.SHOULDER_OFFSET.rotate(robotHeading + turretAngle).add(turretPoint);
        double armLength = focusPoint.subtract(shoulderPoint).length();

{
    Canvas canvas = telemetryPacket.fieldOverlay();
    Point focus = Specs.Arm.SHOULDER_OFFSET.rotate(turretAngle).add(turretPoint).add(
            new Point(armLength * Math.cos(turretAngle + robotHeading),
                    armLength * Math.sin(turretAngle + robotHeading)));
    Point clawOffset = new Point(0, Specs.Arm.CLAW_Y_OFFSET).rotate(turretAngle);
    focus = focus.add(clawOffset);

    canvas.setStroke("#FF0000");
    canvas.strokeCircle(shoulderPoint.x, shoulderPoint.y, 2);
    canvas.strokeCircle(focus.x, focus.y, 2);
}

        // Tell the arm where to go:
        return new double[]{armLength, turretAngle};
    }

    // Set the arm control based on the robot's pose and the target focus point:
    void setArmControl(Canvas canvas, double targetArmLength, double targetTurretHeading) {
        // Check if the robot is getting close to the submersible:
        Pose2d pose = autoPilot.poser.getPose();
        Point robotCenter = new Point(pose.position);
        double robotHeading = pose.heading.log();
        double currentTurretHeading = arm.joints[Id.TURRET].currentAngle;
        double currentArmLength = arm.model.computedArmLength;

        // Compute a partial bound box of the arm at its target position:
        Point[] polygon = new Point[]{
                new Point(0, Specs.Arm.Y_BOUNDS),
                new Point(currentArmLength + Specs.Arm.X_CLAW_BOUNDS, Specs.Arm.Y_BOUNDS),
                new Point(currentArmLength + Specs.Arm.X_CLAW_BOUNDS, -Specs.Arm.Y_BOUNDS),
                new Point(0, -Specs.Arm.Y_BOUNDS)
        };
        Point turretPoint = Specs.Arm.TURRET_OFFSET.rotate(robotHeading).add(robotCenter);
        Point shoulderPoint = Specs.Arm.SHOULDER_OFFSET.rotate(robotHeading + currentTurretHeading).add(turretPoint);
        for (int i = 0; i < polygon.length; i++) {
            polygon[i] = polygon[i].rotate(robotHeading + currentTurretHeading).add(shoulderPoint);
        }

        // Determine if any of the arm's bounds segments intersect with the submersible walls:
        Segment[] boundsSegments = new Segment[]{
                new Segment(polygon[0], polygon[1]),
                new Segment(polygon[1], polygon[2]),
                new Segment(polygon[2], polygon[3]),
        };
        boolean retract = false;

        // @@@ Decide if I want to re-enable hit testing:

//        for (Segment boundSegment: boundsSegments) {
//            for (Segment wall : Specs.Field.COLLISION_WALLS) {
//                if (LineSegmentIntersection.doSegmentsIntersect(boundSegment, wall)) {
//                    // Woah, the arm would collide with the submersible! Retract it:
//                    retract = true;
//                }
//            }
//        }
//
//        canvas.setStroke("#a020f0");
//        for (Segment boundSegment: boundsSegments) {
//            canvas.strokeLine(boundSegment.p1.x, boundSegment.p1.y, boundSegment.p2.x, boundSegment.p2.y);
//        }
//        for (Segment wall : Specs.Field.COLLISION_WALLS) {
//            canvas.strokeLine(wall.p1.x, wall.p1.y, wall.p2.x, wall.p2.y);
//        }

        // Retract if the robot isn't in the pickup area:
        if ((pose.position.x < -48) || (pose.position.x > 0) ||
            (pose.position.y < -20) || (pose.position.y > 20)) {
            retract = true;
        }

        if (retract) {
            subActions.add(new ReachAction(arm, ReachAction.State.HOME));
            subActions.add(new TurretAction(arm, 0));
        } else {
            // It's safe to extend the arm:
            subActions.add(new ReachAction(arm, ReachAction.State.PICKUP, targetArmLength, Specs.Arm.SUBMERSIBLE_HEIGHT));
            subActions.add(new TurretAction(arm, targetTurretHeading));
        }
    }

    @Override
    public boolean run(double elapsedTime) {
        if (elapsedTime == 0) {
            // Start the action:
            subActions.add(new ReachAction(arm, ReachAction.State.HOME));
        }
        double[] armControl = setTargetPose();
        setArmControl(telemetryPacket.fieldOverlay(), armControl[0], armControl[1]);
        return true; // @@@ Loop endlessly
    }
}
