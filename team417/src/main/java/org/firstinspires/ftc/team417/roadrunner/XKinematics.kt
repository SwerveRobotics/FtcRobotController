data class XKinematics @JvmOverloads constructor(
        @JvmField
        val trackWidth: Double,
        @JvmField
        val lateralMultiplier: Double = 1.0
) {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [XKinematics]
     */
    constructor(
            trackWidth: Double,
            wheelbase: Double,
            lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)

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
                    // TODO: difference explanation: X-Drive's forward kinematics needs to account for the 45-degree angle of wheels, hence the sqrt(2)/2 multiplier
                    (w.leftFront + w.rightBack) * 0.5 * (Math.sqrt(2.0) / 2) +
                            (w.leftBack + w.rightFront) * 0.5 * (Math.sqrt(2.0) / 2),
                    (-w.leftFront + w.rightBack) * (0.5 / lateralMultiplier) * (Math.sqrt(2.0) / 2) +
                            (w.leftBack - w.rightFront) * (0.5 / lateralMultiplier) * (Math.sqrt(2.0) / 2)
            ),
            (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.5 / trackWidth),
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
            // TODO: difference explanation: The inverse kinematics of an X-Drive must consider the 45-degree orientation of the wheels
            (t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * trackWidth) * (Math.sqrt(2.0) / 2),
            (t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * trackWidth) * (Math.sqrt(2.0) / 2),
            (t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * trackWidth) * (Math.sqrt(2.0) / 2),
            (t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * trackWidth) * (Math.sqrt(2.0) / 2),
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
