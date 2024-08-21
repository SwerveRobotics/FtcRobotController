package org.firstinspires.ftc.team417.roadrunner

import com.acmerobotics.roadrunner.*
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class XKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
) {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [XKinematics]
     */
    constructor(
        trackWidth: Double,
        wheelbase: Double
    ) : this((trackWidth + wheelbase) / 2)

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
        Vector2dDual(
            // X-Drive's forward kinematics needs to account for the 45-degree angle of wheels, hence the sqrt(2)/2 multiplier
            (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * (0.25 / sqrt(2.0)),
            (-w.leftFront + w.leftBack -w.rightBack + w.rightFront) * (0.25 / sqrt(2.0))
        ),
        (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / trackWidth),
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

    fun <Param> inverse(t: PoseVelocity2dDual<Param>) = WheelVelocities(
        // The inverse kinematics of an X-Drive must consider the 45-degree orientation of the wheels
        (t.linearVel.x - t.linearVel.y - t.angVel * trackWidth) * sqrt(2.0),
        (t.linearVel.x + t.linearVel.y - t.angVel * trackWidth) * sqrt(2.0),
        (t.linearVel.x - t.linearVel.y + t.angVel * trackWidth) * sqrt(2.0),
        (t.linearVel.x + t.linearVel.y + t.angVel * trackWidth) * sqrt(2.0),
    )

    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
            val txRobotWorld = robotPose.value().inverse()
            val robotVelWorld = robotPose.velocity().value()
            val robotVelRobot = txRobotWorld * robotVelWorld
            return inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
                .all()
                .minOf { abs(maxWheelVel / it.value()) }
        }
    }
}
