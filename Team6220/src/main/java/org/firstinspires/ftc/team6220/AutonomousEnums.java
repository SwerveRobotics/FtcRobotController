package org.firstinspires.ftc.team6220;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;


public class AutonomousEnums {

    public enum AutoStartPosition {
        LEFT(DRIFTConstants.LEFT_STARTING_POSE),
        MIDDLE(DRIFTConstants.MIDDLE_STARTING_POSE),
        RIGHT(DRIFTConstants.RIGHT_STARTING_POSE);

        public final Pose2d startingPose;

        AutoStartPosition(Pose2d startingPose) {
            this.startingPose = startingPose;
        }
    }

    public enum AutoType {
        SCORING,
        PARK;
    }

    public enum ParkPosition {
        OBSERVATION(DRIFTConstants.OBSERVATION_PARK_POSITION),
        SUBMERSIBLE(DRIFTConstants.SUBMERSIBLE_PARK_POSITION);

        public final Vector2d parkingPosition;

        ParkPosition(Vector2d parkingPosition) {
            this.parkingPosition = parkingPosition;
        }

        public TrajectoryActionBuilder appendAutonomousSegment(TrajectoryActionBuilder builder, AutoType autoType) {
            switch(this) {
                case OBSERVATION: {
                    switch(autoType) {
                        case PARK: {
                            return builder.strafeTo(parkingPosition)
                                    .endTrajectory();
                        }
                        case SCORING: {
                            return builder.splineTo(parkingPosition, Math.PI)
                                    .endTrajectory();
                        }
                    }
                }
                case SUBMERSIBLE: {
                    switch (autoType) {
                        case PARK: {
                            // code for submersible park auto goes here
                        }
                        case SCORING: {
                            // code for submersible scoring-park auto goes here
                        }
                    }
                }
            }

            return builder;
        }

        @NonNull
        @Override
        public String toString() {
            // haven't implemented this yet, so ternary operator override go brrrt
            return this.equals(ParkPosition.SUBMERSIBLE) ? "DISABLED" : super.toString();
        }
    }

    public enum SpikeMarkSide {
        LEFT,
        RIGHT;

        public TrajectoryActionBuilder appendScoringRoute(TrajectoryActionBuilder actionBuilder) {
            switch (this) {
                case LEFT:
                    return actionBuilder
                            // strafe to scoring area
                            .strafeTo(new Vector2d(55, 60))
                            .endTrajectory()
                            .setTangent(Math.toRadians(-180))
                            // spline to prepare to collect first sample
                            .splineToLinearHeading(new Pose2d(45, 10, Math.toRadians(-90)),  Math.toRadians(-40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(90))
                            // spline to deposit first sample in scoring area
                            .splineToLinearHeading(new Pose2d(55, 55, Math.toRadians(-140)),  Math.toRadians(40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(-180))
                            // spline to prepare to collect second sample
                            .splineToLinearHeading(new Pose2d(55, 10, Math.toRadians(-90)),  Math.toRadians(-40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(90))
                            // spline to deposit second sample in scoring area
                            .splineToLinearHeading(new Pose2d(55, 61, Math.toRadians(-140)),  Math.toRadians(40))
                            .endTrajectory()
                            .setTangent(Math.toRadians(-180))
                            // spline to prepare to collect third sample
                            .splineToLinearHeading(new Pose2d(62, 10, Math.toRadians(-90)),  Math.toRadians(40))
                            .endTrajectory()
                            // strafe to deposit third sample
                            .strafeTo(new Vector2d(62, 55))
                            // prepare to move to park position
                            .strafeTo(new Vector2d(62, 50));
                case RIGHT:
            }
            return actionBuilder;
        }

        @NonNull
        @Override
        public String toString() {
            // haven't implemented this yet, so ternary operator override go brrrt
            return this.equals(SpikeMarkSide.RIGHT) ? "DISABLED" : super.toString();
        }
    }
}
