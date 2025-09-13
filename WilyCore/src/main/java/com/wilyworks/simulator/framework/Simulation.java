package com.wilyworks.simulator.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Globals;
import com.wilyworks.simulator.helpers.Point;

import java.util.LinkedList;
import java.util.Random;

/**
 * Fake wheel odometry localizer for the simulation.
 */
class WheelLocalizer {
    Simulation simulation;
    Pose2d previousPose;
    PoseVelocity2d previousVelocity;

    WheelLocalizer(Simulation simulation) {
        this.simulation = simulation;
        this.previousPose = simulation.getPose(0, false); // Use error-added pose
        this.previousVelocity = simulation.poseVelocity;
    }

    // Rotate a vector by 'theta' radians:
    static Vector2d transform(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) - y * Math.sin(theta),
                x * Math.sin(theta) + y * Math.cos(theta));
    }

    // Return a 'twist' that represents all movement since the last call:
    double[] update() {
        Pose2d simulationPose = simulation.getPose(0, false); // Use error-added pose
        double deltaAng = simulationPose.heading.log() - previousPose.heading.log();
        double deltaAngVel = simulation.poseVelocity.angVel - previousVelocity.angVel;

        // Transform from field-absolute position to robot-relative position:
        double robotAngle = simulationPose.heading.log();
        Vector2d deltaLinear = transform(
                simulationPose.position.x - previousPose.position.x,
                simulationPose.position.y - previousPose.position.y,
                -robotAngle);
        Vector2d deltaLinearVel = transform(
                simulation.poseVelocity.linearVel.x - previousVelocity.linearVel.x,
                simulation.poseVelocity.linearVel.y - previousVelocity.linearVel.y,
                -robotAngle);

        previousPose = simulationPose;
        previousVelocity = simulation.poseVelocity;

        return new double[]{deltaLinear.x, deltaLinear.y, deltaAng,
            deltaLinearVel.x, deltaLinearVel.y, deltaAngVel};
    }
}

/**
 * Structure for remembering poses.
 */
class PoseRecord {
    double time;
    Pose2d pose;

    public PoseRecord(double time, Pose2d pose) {
        this.time = time; this.pose = pose;
    }
}

/**
 * Kinematic simulation for the robot's movement.
 */
public class Simulation {
    // Size of the pose history, in seconds:
    final double POSE_HISTORY_SECONDS = 2.0;

    // Histories of the robot's poses, field-relative. The newest is first:
    public LinkedList<PoseRecord> truePoseHistory = new LinkedList<>(); // True pose
    public LinkedList<PoseRecord> errorPoseHistory = new LinkedList<>(); // Simulated error

    // The robot's current true pose velocity, field-relative:
    public PoseVelocity2d poseVelocity = new PoseVelocity2d(new Vector2d(0, 0), Math.toRadians(0));

    private WheelLocalizer wheelLocalizer; // Fake odometry localizer
    private WilyWorks.Config config; // Kinematic parameters for the simulation
    private PoseVelocity2d requestedVelocity; // Velocity requested by MecanumDrive

    // Random number generator for introducing error:
    private Random random = new Random();
    // Heading in which to bias the positional error, in radians:
    private double positionErrorAngle = Math.toRadians(Math.random() * 360);
    // Unit vector pointing in the direction of the positional bias, in robot-relative coordinates:
    private Point positionErrorVector = new Point(Math.cos(positionErrorAngle), Math.sin(positionErrorAngle));
    // Direction in which to bias the heading, either -1 or 1:
    private double headingErrorSign = random.nextInt(2) == 0 ? -1 : 1;
    // Distance traveled, in inches:
    public double totalDistance;

    public Simulation(WilyWorks.Config config) {
        this.config = config;
        truePoseHistory.add(new PoseRecord(WilyCore.time(), new Pose2d(0, 0, Math.toRadians(0))));
        errorPoseHistory.add(new PoseRecord(WilyCore.time(), new Pose2d(0, 0, Math.toRadians(0))));
        wheelLocalizer = new WheelLocalizer(this);
    }

    // Find the closest pose from the specified seconds ago. Zero retrieves the current pose.
    public Pose2d getPose(double secondsAgo, boolean truePose) {
        LinkedList<PoseRecord> poseHistory = (truePose) ? truePoseHistory : errorPoseHistory;
        if (secondsAgo == 0)
            return poseHistory.get(0).pose;

        double targetTime = WilyCore.time() - secondsAgo;
        double closestTimeGap = Double.MAX_VALUE;
        int closestIndex = 0;
        int i = 0;
        for (PoseRecord record: poseHistory) {
            double timeGap = Math.abs(targetTime - record.time);
            if (timeGap < closestTimeGap) {
                closestTimeGap = timeGap;
                closestIndex = i;
            }
            i++;
        }
        return poseHistory.get(closestIndex).pose;
    }

    // Record into the pose history:
    void recordPose(double time, Pose2d pose, boolean truePose) {
        LinkedList<PoseRecord> poseHistory = (truePose) ? truePoseHistory : errorPoseHistory;
        while (poseHistory.size() != 0) {
            PoseRecord oldPose = poseHistory.getLast();
            if (time - oldPose.time < POSE_HISTORY_SECONDS)
                break; // ====>
            poseHistory.removeLast();
        }
        poseHistory.addFirst(new PoseRecord(time, pose));
    }

    // Move the robot in the requested direction via kinematics:
    public void advance(double dt) {
        // Request a stop if no new velocity has been requested:
        if (requestedVelocity == null)
            requestedVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        boolean brake = requestedVelocity.linearVel.x == 0
                     && requestedVelocity.linearVel.y == 0
                     && requestedVelocity.angVel == 0;
        // ^^^ brake = false; // @@@@@@@@@@@@@@@@@@@@@@@@@

        // Brake if the requested velocity is zero, otherwise decelerate normally:
        double linearDeceleration = (brake) ? config.maxLinearBraking : config.maxLinearDeceleration;
        double angularAcceleration = (brake) ? config.maxAngularBraking : config.maxAngularAcceleration;

        // Handle the rotational velocity:
        double currentAngular = poseVelocity.angVel;
        double requestedAngular = requestedVelocity.angVel;
        double deltaAngular = requestedAngular - currentAngular;
        if (deltaAngular >= 0) {
            // Increase the angular velocity:
            currentAngular += angularAcceleration * dt;
            currentAngular = Math.min(currentAngular, config.maxAngularSpeed);
            currentAngular = Math.min(currentAngular, requestedAngular);
        } else {
            // Decrease the angular velocity:
            currentAngular -= angularAcceleration * dt; // maxAngAccel is positive
            currentAngular = Math.max(currentAngular, -config.maxAngularSpeed);
            currentAngular = Math.max(currentAngular, requestedAngular);
        }

        // Handle the linear velocity:
        double currentLinearX = poseVelocity.linearVel.x;
        double currentLinearY = poseVelocity.linearVel.y;
        double requestedLinearX = requestedVelocity.linearVel.x;
        double requestedLinearY = requestedVelocity.linearVel.y;

        double currentVelocity = Math.hypot(currentLinearX, currentLinearY);
        double requestedVelocity = Math.hypot(requestedLinearX, requestedLinearY);
        double currentAngle = Math.atan2(currentLinearY, currentLinearX); // Rise over run
        double requestedAngle = Math.atan2(requestedLinearY, requestedLinearX);

        // If the requested velocity is close to zero then its angle is rather undetermined.
        // Use the current angle in that case:
        if (Math.abs(requestedVelocity) == 0)
            requestedAngle = currentAngle;
        double theta = requestedAngle - currentAngle; // Angle from current to requested

        // Clamp to the maximum allowable velocities:
        currentVelocity = Math.min(currentVelocity, config.maxLinearSpeed);
        requestedVelocity = Math.min(requestedVelocity, config.maxLinearSpeed);

        // Perpendicular velocity is the current velocity component away from
        // the requested velocity vector. We reduce this by the deceleration:
        double perpVelocity = Math.sin(theta) * currentVelocity;
        if (perpVelocity >= 0) {
            perpVelocity += linearDeceleration * dt; // minProfileAccel is negative
            perpVelocity = Math.max(perpVelocity, 0);
        } else {
            perpVelocity -= linearDeceleration * dt;
            perpVelocity = Math.min(perpVelocity, 0);
        }

        // Parallel velocity is the current velocity component in the same direction
        // as the requested velocity. Accelerate or decelerate to match our parallel
        // velocity with the request:
        double parallelVelocity = Math.cos(theta) * currentVelocity;
        double parallelDelta = requestedVelocity - parallelVelocity;

        // We now know our perpendicular velocity and we know the maximum allowable
        // velocity so our maximum parallel velocity is remainder. Note that we're
        // guaranteed that won't try to do the square root of a negative:
        double maxParallelVelocity =
                Math.sqrt(Math.pow(config.maxLinearSpeed, 2) - Math.pow(perpVelocity, 2));

        if (parallelDelta >= 0) { // Increase the parallel velocity
            parallelVelocity += config.maxLinearAcceleration * dt;
            parallelVelocity = Math.min(parallelVelocity, maxParallelVelocity);
            parallelVelocity = Math.min(parallelVelocity, requestedVelocity);
        } else { // Decrease the parallel velocity:
            parallelVelocity += linearDeceleration * dt; // maxProfileAccel is positive
            parallelVelocity = Math.max(parallelVelocity, -maxParallelVelocity);
            parallelVelocity = Math.max(parallelVelocity, requestedVelocity);
        }
        currentLinearX = Math.cos(requestedAngle) * parallelVelocity
                       + Math.cos(requestedAngle - Math.PI / 2) * perpVelocity;
        currentLinearY = Math.sin(requestedAngle) * parallelVelocity
                       + Math.sin(requestedAngle - Math.PI / 2) * perpVelocity;

        //------------------------------------------------------------------------------------------
        // Update the true pose:
        Pose2d truePose = getPose(0, true);
        double x = truePose.position.x + dt * currentLinearX;
        double y = truePose.position.y + dt * currentLinearY;

        totalDistance += Math.hypot(dt * currentLinearX, dt * currentLinearY);

        // Keep the robot on the field. Zero the component velocity that made it leave
        // the field:
        if (x > 72.0) {
            x = 72.0;
            currentLinearX = 0;
        }
        if (x <= -72.0) {
            x = -72.0;
            currentLinearX = 0;
        }
        if (y > 72.0) {
            y = 72.0;
            currentLinearY = 0;
        }
        if (y <= -72.0) {
            y = -72.0;
            currentLinearY = 0;
        }

        // Update our official pose and velocity:
        truePose = new Pose2d(x, y, truePose.heading.log() + dt * currentAngular);
        poseVelocity = new PoseVelocity2d(new Vector2d(currentLinearX, currentLinearY), currentAngular);

        //------------------------------------------------------------------------------------------
        // Update the pose with error:
        Pose2d errorPose = truePose;
        if (WilyCore.enableSensorError) {
            Point fieldOffset = new Point(dt * currentLinearX, dt * currentLinearY);
            Point robotOffset = fieldOffset.rotate(-truePose.heading.log());

            // Calculate the new error pose as an offset from the old one:
            errorPose = addError(getPose(0, false), dt, robotOffset, dt * currentAngular);
        }

        recordPose(WilyCore.time(), truePose, true);
        recordPose(WilyCore.time(), errorPose, false);
    }

    // Add sensor error to the pose. The deltas are the differences from the new reference pose to
    // the old reference pose, where 'offset' is robot-relative coordinates:
    public Pose2d addError(Pose2d pose, double dt, Point offset, double dRadians) {
        // 95% of the heading error (two standard deviations) should be within a function
        // of time taken in this tick, times WilyConfig.headingError. The latter is expressed
        // as degrees per minute:
        double headingMeanError = headingErrorSign * dt * Math.toRadians(WilyCore.config.headingError) / 60;
        double heading = pose.heading.log() + dRadians + headingMeanError * (1 + random.nextGaussian() / 2);

        // 95% of the positional error (two standard deviations) should be within a function
        // of distance traveled in this tick, times WilyConfig.positionError. The latter is
        // expressed as a percentage of distance traveled:
        double positionMeanError = offset.length() * (WilyCore.config.positionError / 100.0);
        double errorMagnitude = positionMeanError * (1 + random.nextGaussian() / 2);
        Point positionError = positionErrorVector.multiply(errorMagnitude); // Field space

        Point positionDelta = offset.rotate(heading);

        double x = pose.position.x + positionDelta.x + positionError.x;
        double y = pose.position.y + positionDelta.y + positionError.y;
        return new Pose2d(x, y, heading);
    }

    // Entry point for Road Runner to set the current pose and velocity, both in field coordinates,
    // while running a trajectory.
    public void runTo(double dt, Pose2d errorPose, PoseVelocity2d poseVelocity) {
        Pose2d truePose = errorPose;
        if (WilyCore.enableSensorError) {
            // Perhaps intuitively, we add error to the 'true' pose rather than the error pose
            // in this case:
            Pose2d previousErrorPose = getPose(0, false);

            Point fieldOffset = new Point(errorPose.position.minus(previousErrorPose.position));
            Point robotOffset = fieldOffset.rotate(-errorPose.heading.log());
            double dRadians = errorPose.heading.log() - previousErrorPose.heading.log();

            // Calculate the new error pose as an offset from the old one:
            truePose = addError(getPose(0, true), dt, robotOffset, dRadians);

            totalDistance += Math.hypot(fieldOffset.x, fieldOffset.y);
        }

        recordPose(WilyCore.time(), truePose, true);
        recordPose(WilyCore.time(), errorPose, false);
        if (poseVelocity != null)
            this.poseVelocity = poseVelocity;
    }

    // Get the error label to show in the UI:
    public String getErrorLabel() {
        if (WilyCore.enableSensorError) {
            Pose2d truePose = getPose(0, true);
            Pose2d errorPose = getPose(0, false);

            double linearError = truePose.position.minus(errorPose.position).norm();
            double angularError = Math.abs(Globals.normalizeAngle(truePose.heading.log() - errorPose.heading.log()));
            // return String.format("Pose error: %.1f\", %.1f\u00b0, distance: %.1f\", time: %.0fs",
            //         linearError, Math.toDegrees(angularError), totalDistance, WilyCore.time());
            return "";
        } else {
            return "";
        }
    }

    // Power the motors according to the specified velocities. 'stickVelocity' is for controller
    // input and 'assistVelocity' is for computed driver assistance. The former is specified in
    // voltage values normalized from -1 to 1 (just like the regular DcMotor::SetPower() API)
    // whereas the latter is in inches/s or radians/s. Both types of velocities can be specified
    // at the same time in which case the velocities are added together (to allow assist and stick
    // control to blend together, for example).
    //
    // It's also possible to map the controller input to inches/s and radians/s instead of the
    // normalized -1 to 1 voltage range. You can reference MecanumDrive.PARAMS.maxWheelVel and
    // .maxAngVel to determine the range to specify. Note however that the robot can actually
    // go faster than Road Runner's PARAMS values so you would be unnecessarily slowing your
    // robot down.
    public void setDrivePowers(
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity)
    {
        PoseVelocity2d fieldVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (stickVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    stickVelocity.linearVel.x * config.maxLinearSpeed,
                    stickVelocity.linearVel.y * config.maxLinearSpeed),
                    stickVelocity.angVel * config.maxAngularSpeed);
            fieldVelocity = getPose(0, true).times(fieldVelocity); // Make it field-relative
        }
        if (assistVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    fieldVelocity.linearVel.x + assistVelocity.linearVel.x,
                    fieldVelocity.linearVel.y + assistVelocity.linearVel.y),
                    fieldVelocity.angVel + assistVelocity.angVel);
        }
        this.requestedVelocity = fieldVelocity;
    }

    // Entry point to get the current wheel-odometry localizer position:
    public double[] localizerUpdate() { return wheelLocalizer.update(); }

    // Entry point to set the pose and velocity, both in field coordinates and inches and radians:
    public void setStartPose(Pose2d pose, PoseVelocity2d poseVelocity) {
        recordPose(WilyCore.time(), pose, true);
        recordPose(WilyCore.time(), pose, false);
        if (poseVelocity != null)
            this.poseVelocity = poseVelocity;

        // Recreate the localizer so that it doesn't register a move:
        wheelLocalizer = new WheelLocalizer(this);
    }
}
