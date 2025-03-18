package org.firstinspires.ftc.teamMentor;

public class Specs {
    /**
     * Arm constants.
     */
    static class Arm {
        static final String CALIBRATION_FILE = "/sdcard/arm_calibration.json";

        static final double MAX_SERVO_SPEED = Math.toRadians(60) / 0.17; // Marcel's specs: 60 degrees in 0.17 seconds
        static final Point TURRET_OFFSET = new Point(-4, 1); // Relative to robot center
        static final Point SHOULDER_OFFSET = new Point(0, 0.5); // Offset of shoulder relative to turret base
        static final double SEGMENT_WIDTH = 2.75; // Width of the arm segments
        static final double CLAW_Y_OFFSET = -SEGMENT_WIDTH; // Offset of the claw in y, relative to the shoulder's center

        static final double SHOULDER_HEIGHT = 8; // Height of the shoulder joint relative to floor
        static final double SEGMENT_OVERHANG = 2; // Overhang of the arm segment past the joint
        static final double SEGMENT_LENGTH = 13.0; // Length of an arm segment from joint to joint
        static final double LAST_SEGMENT_LENGTH = 3.0; // Length of the last arm segment
        static final double CLAW_OFFSET = 2; // Offset from the end of the last segment to the claw
        static final double Y_BOUNDS = 4; // Positive Y value that bounds the arm, on either side of the turret base
        static final double X_CLAW_BOUNDS = 2; // Additional X bounds to account for the claw
        static final double SUBMERSIBLE_HEIGHT = 4; // Target distance from claw tip to floor when in submersible
    }

    /**
     * Constants describing the field.
     */
    static class Field {
        static final double ABUTMENT_X = -15.5; // X coordinate of the submersible abutment, measure using Calibrator
        static final double SUBMERSIBLE_WIDTH = 27.5; // Inches
        static final double SUBMERSIBLE_LENGTH = 44.5; // Inches
        static final double HALF_SUBMERSIBLE_WIDTH = SUBMERSIBLE_WIDTH / 2;
        static final double HALF_SUBMERSIBLE_LENGTH = SUBMERSIBLE_LENGTH / 2;
        static final double SUBMERSIBLE_X = -15; // Robot has to stay to the left of this, in inches
        static final double SUBMERSIBLE_Y = 21; // Robot has to stay below this y value, in inches

        static final Segment[] COLLISION_WALLS = {
                new Segment(18, 24, 24, 24),
                new Segment(18, -24, 24, -24),
                new Segment(-18, 24, -24, 24),
                new Segment(-18, -24, -24, -24),
        };
    }

}
