package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;

public class AutonomousEnums {
    public enum AllianceColor {
        BLUE,
        RED
    }

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
        PARK,
        SCORING;
    }

    public enum SpikeMarkPickupAmount {
        THREE,
        TWO,
        ONE,
        ZERO
    }

    public enum ParkPosition {
        OBSERVATION,
        SUBMERSIBLE
    }

    public enum SpikeMarkSide {
        LEFT,
        RIGHT
    }
}
