package org.firstinspires.ftc.team417.roadrunner

import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.VelConstraint

enum class KinematicType {
    MECANUM,
    X
}

data class Kinematics(
    val mecanumKinematics: MecanumKinematics,
    val xKinematics: XKinematics,
    val kinematicType: KinematicType
) {
    inner class WheelVelConstraint(@JvmField val maxWheelVel: Double) : VelConstraint {
        override fun maxRobotVel(
            robotPose: Pose2dDual<Arclength>,
            path: PosePath,
            s: Double
        ): Double {
            return when (kinematicType) {
                KinematicType.MECANUM -> mecanumKinematics.WheelVelConstraint(maxWheelVel)
                    .maxRobotVel(robotPose, path, s)
                KinematicType.X -> xKinematics.WheelVelConstraint(maxWheelVel)
                    .maxRobotVel(robotPose, path, s)
            }
        }
    }
}