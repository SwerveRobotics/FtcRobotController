package org.firstinspires.ftc.team6220;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
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

        @NonNull
        @Override
        public String toString() {
            // haven't implemented this yet, so ternary operator override go brrrt
            return this.equals(ParkPosition.SUBMERSIBLE) ? "DISABLED" : super.toString();
        }

        ParkPosition(Vector2d parkingPosition) {
            this.parkingPosition = parkingPosition;
        }
    }

    public enum SpikeMarkSide {
        LEFT,
        RIGHT;

        @NonNull
        @Override
        public String toString() {
            // haven't implemented this yet, so ternary operator override go brrrt
            return this.equals(SpikeMarkSide.RIGHT) ? "DISABLED" : super.toString();
        }
    }
}
