package org.firstinspires.ftc.team417.roadrunner

import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import kotlin.math.abs
import kotlin.math.sqrt

enum class KinematicType {
    MECANUM,
    X,
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction (ignore this if [kinematicType] is X)
 */
data class HolonomicKinematics @JvmOverloads constructor(
    @JvmField
    val kinematicType: KinematicType,
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [HolonomicKinematics]
     */
    constructor(
        kinematicType: KinematicType,
        trackWidth: Double,
        wheelbase: Double,
        lateralMultiplier: Double = 1.0
    ) : this(kinematicType, (trackWidth + wheelbase) / 2, lateralMultiplier)

    data class WheelIncrements<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    )

    fun <Param> forward(w: WheelIncrements<Param>) = Twist2dDual(
        when (kinematicType) {
            KinematicType.MECANUM -> Vector2dDual(
                (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * 0.25,
                (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / lateralMultiplier),
            )

            KinematicType.X -> Vector2dDual(
                // X-Drive's forward kinematics needs to account for the 45-degree angle of wheels, hence the sqrt(2)/2 multiplier
                (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * (0.25 / sqrt(2.0)),
                (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / sqrt(2.0))
            )
        },
        (-w.leftFront - w.leftBack + w.rightBack + w.rightFront) * (0.25 / trackWidth),
    )

    data class WheelVelocities<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) {
        fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }


    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = when (kinematicType) {
        KinematicType.MECANUM -> WheelVelocities(
            t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
            t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
            t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
            t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
        )

        KinematicType.X -> WheelVelocities(
            // The inverse kinematics of an X-Drive must consider the 45-degree orientation of the wheels
            (t.linearVel.x - t.linearVel.y - t.angVel * trackWidth) * sqrt(2.0),
            (t.linearVel.x + t.linearVel.y - t.angVel * trackWidth) * sqrt(2.0),
            (t.linearVel.x - t.linearVel.y + t.angVel * trackWidth) * sqrt(2.0),
            (t.linearVel.x + t.linearVel.y + t.angVel * trackWidth) * sqrt(2.0),
        )
    }

    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double
        ): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }
}