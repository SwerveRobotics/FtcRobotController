package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.IMU;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Wily Works simulated IMU implementation.
 */
public class WilyIMU extends WilyHardwareDevice implements IMU {
    double startYaw;

    @Override
    public boolean initialize(Parameters parameters) {
        resetYaw();
        return true;
    }

    @Override
    public void resetYaw() {
        startYaw = WilyWorks.getPose().heading.log();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(
                AngleUnit.RADIANS,
                WilyWorks.getPose().heading.log() - startYaw,
                0,
                0,
                0);
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return new Orientation();
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return new Quaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return new AngularVelocity(
                angleUnit,
                (float) WilyWorks.getPoseVelocity().angVel, // ### transformedAngularVelocityVector.get(0),
                0, // ### transformedAngularVelocityVector.get(1),
                0, // ### transformedAngularVelocityVector.get(2),
                0); // ### rawAngularVelocity.acquisitionTime);
    }
}
