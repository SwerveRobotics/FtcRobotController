package com.wilyworks.simulator.framework;

import static java.lang.System.nanoTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.wilyworks.common.WilyWorks;

import java.awt.Dimension;

/**
 * Fake localizer for the simulation.
 */
class Localizer {
    Simulation simulation;
    Pose2d previousPose;
    PoseVelocity2d previousVelocity;

    Localizer(Simulation simulation) {
        this.simulation = simulation;
        this.previousPose = simulation.pose;
        this.previousVelocity = simulation.poseVelocity;
    }

    // Rotate a vector by 'theta' radians:
    static Vector2d transform(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) - y * Math.sin(theta),
                x * Math.sin(theta) + y * Math.cos(theta));
    }

    // Return an 'twist' that represents all movement since the last call:
    double[] update() {
        double deltaAng = simulation.pose.heading.log() - previousPose.heading.log();
        double deltaAngVel = simulation.poseVelocity.angVel - previousVelocity.angVel;

        // Transform from field-absolute position to robot-relative position:
        double robotAngle = simulation.pose.heading.log();
        Vector2d deltaLinear = transform(
                simulation.pose.position.x - previousPose.position.x,
                simulation.pose.position.y - previousPose.position.y,
                -robotAngle);
        Vector2d deltaLinearVel = transform(
                simulation.poseVelocity.linearVel.x - previousVelocity.linearVel.x,
                simulation.poseVelocity.linearVel.y - previousVelocity.linearVel.y,
                -robotAngle);

        previousPose = simulation.pose;
        previousVelocity = simulation.poseVelocity;

        return new double[]{deltaLinear.x, deltaLinear.y, deltaAng,
            deltaLinearVel.x, deltaLinearVel.y, deltaAngVel};
    }
}

/**
 * Kinematic simulation for the robot's movement.
 */
public class Simulation {
    public Pose2d pose = new Pose2d(-48, 0, Math.toRadians(90)); // Robot's true pose
    public PoseVelocity2d poseVelocity = new PoseVelocity2d(new Vector2d(0, 60), Math.toRadians(0)); // Robot's true pose velocity
    public Dimension robotSize = new Dimension(16, 18); // Size in inches of user's robot

    private Localizer localizer;
    private WilyWorks.Config config; // Kinematic parameters for the simulation
    private PoseVelocity2d requestedVelocity; // Velocity requested by MecanumDrive

    public Simulation(WilyWorks.Config config) {
        this.config = config;
        localizer = new Localizer(this);
    }

    // Get the current time, in seconds:
    static double time() {
        return nanoTime() * 1e-9;
    }

    // Move the robot in the requested direction via kinematics:
    public void advance(double dt) {
        // Request a stop if no new velocity has been requested:
        if (requestedVelocity == null)
            requestedVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Handle the rotational velocity:
        double currentAngular = poseVelocity.angVel;
        double requestedAngular = requestedVelocity.angVel;
        double deltaAngular = requestedAngular - currentAngular;
        if (deltaAngular >= 0) {
            // Increase the angular velocity:
            currentAngular += config.maxAngularAcceleration * dt;
            currentAngular = Math.min(currentAngular, config.maxAngularSpeed);
            currentAngular = Math.min(currentAngular, requestedAngular);
        } else {
            // Decrease the angular velocity:
            currentAngular -= config.maxAngularAcceleration * dt; // maxAngAccel is positive
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
        // the requested velocity. We reduce this by the deceleration:
        double perpVelocity = Math.sin(theta) * currentVelocity;
        if (perpVelocity >= 0) {
            perpVelocity += config.maxLinearDeceleration * dt; // minProfileAccel is negative
            perpVelocity = Math.max(perpVelocity, 0);
        } else {
            perpVelocity -= config.maxLinearDeceleration * dt;
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
            parallelVelocity -= config.maxLinearAcceleration * dt; // maxProfileAccel is positive
            parallelVelocity = Math.max(parallelVelocity, -maxParallelVelocity);
            parallelVelocity = Math.max(parallelVelocity, requestedVelocity);
        }
        currentLinearX = Math.cos(requestedAngle) * parallelVelocity
                       + Math.cos(requestedAngle - Math.PI / 2) * perpVelocity;
        currentLinearY = Math.sin(requestedAngle) * parallelVelocity
                       + Math.sin(requestedAngle - Math.PI / 2) * perpVelocity;

        double x = pose.position.x + dt * currentLinearX;
        double y = pose.position.y + dt * currentLinearY;

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
        pose = new Pose2d(x, y, pose.heading.log() + dt * currentAngular);
        poseVelocity = new PoseVelocity2d(new Vector2d(currentLinearX, currentLinearY), currentAngular);
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
            fieldVelocity = pose.times(fieldVelocity); // Make it field-relative
        }
        if (assistVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    fieldVelocity.linearVel.x + assistVelocity.linearVel.x,
                    fieldVelocity.linearVel.y + assistVelocity.linearVel.y),
                    fieldVelocity.angVel + assistVelocity.angVel);
        }
        this.requestedVelocity = fieldVelocity;
    }

    // Entry point to get the current localizer position:
    public double[] localizerUpdate() { return localizer.update(); }

    // Entry point to set the pose and velocity, both in field coordinates:
    public void setPose(Pose2d pose, PoseVelocity2d poseVelocity) {
        this.pose = pose;
        this.poseVelocity = poseVelocity;
        this.localizer = new Localizer(this);
    }
}
