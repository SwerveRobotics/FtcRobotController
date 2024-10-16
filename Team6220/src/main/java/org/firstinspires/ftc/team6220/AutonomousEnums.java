package org.firstinspires.ftc.team6220;

import com.acmerobotics.roadrunner.Pose2d;

public class AutonomousEnums {
    public enum AllianceColor {
        BLUE,
        RED
    }

    public enum AutoStartPosition {
        LEFT(new Pose2d(-20, 60, (3*Math.PI)/2)),
        MIDDLE(new Pose2d(0, 60, (3*Math.PI)/2)),
        RIGHT(new Pose2d(20, 60, (3*Math.PI)/2));

        public final Pose2d startingPose;

        AutoStartPosition(Pose2d startingPose) {
            this.startingPose = startingPose;
        }
    }

    public enum AutoType {
        PARK,
        BASKET;
    }

    public enum SpikeMarkPickupAmount {
        ZERO,
        ONE,
        TWO,
        THREE
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
