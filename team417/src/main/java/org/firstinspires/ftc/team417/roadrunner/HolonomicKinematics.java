package org.firstinspires.ftc.team417.roadrunner;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.Arrays;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.OptionalDouble;

public class HolonomicKinematics {
    public KinematicType kinematicType;
    public final double trackWidth;
    public final double lateralMultiplier;

    /**
     * Constructor for HolonomicKinematics.
     *
     * @param kinematicType      Type of kinematic mechanism (Mecanum or X).
     * @param trackWidth         Distance between wheels on opposite sides.
     */
    public HolonomicKinematics(KinematicType kinematicType, double trackWidth) {
        this(kinematicType, trackWidth, 1);
    }

    /**
     * Constructor for HolonomicKinematics.
     *
     * @param trackWidth         Distance between wheels on opposite sides.
     * @param lateralMultiplier  Factor to adjust strafe velocity, ignored for X kinematic type.
     */
    public HolonomicKinematics(double trackWidth, double lateralMultiplier) {
        this.trackWidth = trackWidth;
        this.lateralMultiplier = lateralMultiplier;
    }

    /**
     * Constructor for HolonomicKinematics.
     *
     * @param kinematicType      Type of kinematic mechanism (Mecanum or X).
     * @param trackWidth         Distance between wheels on opposite sides.
     * @param lateralMultiplier  Factor to adjust strafe velocity, ignored for X kinematic type.
     */
    public HolonomicKinematics(KinematicType kinematicType, double trackWidth, double lateralMultiplier) {
        this.kinematicType = kinematicType;
        this.trackWidth = trackWidth;
        this.lateralMultiplier = lateralMultiplier;
    }

    /**
     * Overloaded constructor.
     *
     * @param kinematicType      Type of kinematic mechanism (Mecanum or X).
     * @param trackWidth         Distance between wheels on opposite sides.
     * @param wheelbase          Distance between wheels on the same side.
     * @param lateralMultiplier  Factor to adjust strafe velocity, ignored for X kinematic type.
     */
    public HolonomicKinematics(KinematicType kinematicType, double trackWidth, double wheelbase, double lateralMultiplier) {
        this(kinematicType, (trackWidth + wheelbase) / 2, lateralMultiplier);
    }

    public static class WheelIncrements<Param> {
        public final DualNum<Param> leftFront;
        public final DualNum<Param> leftBack;
        public final DualNum<Param> rightBack;
        public final DualNum<Param> rightFront;

        public WheelIncrements(DualNum<Param> leftFront, DualNum<Param> leftBack, DualNum<Param> rightBack, DualNum<Param> rightFront) {
            this.leftFront = leftFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;
            this.rightFront = rightFront;
        }
    }

    public <Param> Twist2dDual<Param> forward(WheelIncrements<Param> w) {
        switch (kinematicType) {
            case MECANUM:
                return new Twist2dDual<>(
                        new Vector2dDual<>(
                                (w.leftFront.plus(w.leftBack).plus(w.rightBack).plus(w.rightFront)).times(0.25),
                                (w.leftFront.unaryMinus().plus(w.leftBack).minus(w.rightBack).plus(w.rightFront)).times(0.25 / lateralMultiplier)
                        ),
                        (w.leftFront.unaryMinus().minus(w.leftBack).plus(w.rightBack).plus(w.rightFront)).times(0.25 / trackWidth)
                );
            case X:
                return new Twist2dDual<>(
                        new Vector2dDual<>(
                                (w.leftFront.plus(w.leftBack).plus(w.rightBack).plus(w.rightFront)).times(0.25 / sqrt(2.0)),
                                (w.leftFront.unaryMinus().plus(w.leftBack).minus(w.rightBack).plus(w.rightFront)).times(0.25 / sqrt(2.0))
                        ),
                        (w.leftFront.unaryMinus().minus(w.leftBack).plus(w.rightBack).plus(w.rightFront)).times(0.25 / trackWidth)
                );
            default:
                throw new IllegalStateException("Unexpected kinematicType: " + kinematicType);
        }
    }

    public static class WheelVelocities<Param> {
        public final DualNum<Param> leftFront;
        public final DualNum<Param> leftBack;
        public final DualNum<Param> rightBack;
        public final DualNum<Param> rightFront;

        public WheelVelocities(DualNum<Param> leftFront, DualNum<Param> leftBack, DualNum<Param> rightBack, DualNum<Param> rightFront) {
            this.leftFront = leftFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;
            this.rightFront = rightFront;
        }

        public List<DualNum<Param>> all() {
            return Arrays.asList(leftFront, leftBack, rightBack, rightFront);
        }
    }

    public <Param> WheelVelocities<Param> inverse(PoseVelocity2dDual<Param> t) {
        switch (kinematicType) {
            case MECANUM:
                return new WheelVelocities<>(
                        t.linearVel.x.minus(t.linearVel.y.times(lateralMultiplier)).minus(t.angVel.times(trackWidth)),
                        t.linearVel.x.plus(t.linearVel.y.times(lateralMultiplier)).minus(t.angVel.times(trackWidth)),
                        t.linearVel.x.minus(t.linearVel.y.times(lateralMultiplier)).plus(t.angVel.times(trackWidth)),
                        t.linearVel.x.plus(t.linearVel.y.times(lateralMultiplier)).plus(t.angVel.times(trackWidth))
                );
            case X:
                return new WheelVelocities<>(
                        t.angVel.times(trackWidth).unaryMinus().times(1/sqrt(2.0)).plus(t.linearVel.x.minus(t.linearVel.y)),
                        t.angVel.times(trackWidth).unaryMinus().times(1/sqrt(2.0)).plus(t.linearVel.x.plus(t.linearVel.y)),
                        t.angVel.times(trackWidth).times(1/sqrt(2.0)).plus(t.linearVel.x.minus(t.linearVel.y)),
                        t.angVel.times(trackWidth).times(1/sqrt(2.0)).plus(t.linearVel.x.plus(t.linearVel.y))


//                        (t.linearVel.x.minus(t.linearVel.y).minus(t.angVel.times(trackWidth))),
//                        (t.linearVel.x.plus(t.linearVel.y).minus(t.angVel.times(trackWidth))),
//                        (t.linearVel.x.minus(t.linearVel.y).plus(t.angVel.times(trackWidth))),
//                        (t.linearVel.x.plus(t.linearVel.y).plus(t.angVel.times(trackWidth)))
                );
            default:
                throw new IllegalStateException("Unexpected kinematicType: " + kinematicType);
        }
    }

    public class WheelVelConstraint implements VelConstraint {
        public final double maxWheelVel;

        public WheelVelConstraint(double maxWheelVel) {
            this.maxWheelVel = maxWheelVel;
        }

        @Override
        public double maxRobotVel(Pose2dDual<Arclength> robotPose, @NonNull PosePath path, double s) {
            Pose2d txRobotWorld = robotPose.value().inverse();
            PoseVelocity2d robotVelWorld = robotPose.velocity().value();
            PoseVelocity2d robotVelRobot = txRobotWorld.times(robotVelWorld);
            OptionalDouble result = inverse(PoseVelocity2dDual.constant(robotVelRobot, 1))
                    .all()
                    .stream()
                    .mapToDouble(wheelVel -> abs(maxWheelVel / wheelVel.value())).min();
            if (result.isPresent()) {
                return result.getAsDouble();
            } else {
                throw new NoSuchElementException("OptionalDouble result had no double present");
            }
        }
    }
}