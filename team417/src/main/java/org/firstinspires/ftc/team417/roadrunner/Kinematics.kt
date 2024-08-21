package org.firstinspires.ftc.team417.roadrunner

sealed class Kinematics {
        data class Mecanum(val value: Mecanum) : Kinematics()
        data class X(val value: XKinematics) : Kinematics()
}