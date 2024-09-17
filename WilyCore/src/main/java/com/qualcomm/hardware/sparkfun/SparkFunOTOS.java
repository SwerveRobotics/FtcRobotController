/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package com.qualcomm.hardware.sparkfun;

import java.nio.ByteBuffer;
import java.util.Arrays;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        name = "SparkFun OTOS",
        xmlTag = "SparkFunOTOS",
        description = "SparkFun Qwiic Optical Tracking Odometry Sensor"
)
public class SparkFunOTOS extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public SparkFunOTOS(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
    }

    // Default I2C addresses of the Qwiic OTOS
    public static final byte DEFAULT_ADDRESS = 0x17;
    // Minimum scalar value for the linear and angular scalars
    public static final double MIN_SCALAR = 0.872;

    // Maximum scalar value for the linear and angular scalars
    public static final double MAX_SCALAR = 1.127;

    // OTOS register map
    protected static final byte REG_PRODUCT_ID = 0x00;
    protected static final byte REG_HW_VERSION = 0x01;
    protected static final byte REG_FW_VERSION = 0x02;
    protected static final byte REG_SCALAR_LINEAR = 0x04;
    protected static final byte REG_SCALAR_ANGULAR = 0x05;
    protected static final byte REG_IMU_CALIB = 0x06;
    protected static final byte REG_RESET = 0x07;
    protected static final byte REG_SIGNAL_PROCESS = 0x0E;
    protected static final byte REG_SELF_TEST = 0x0F;
    protected static final byte REG_OFF_XL = 0x10;
    protected static final byte REG_OFF_XH = 0x11;
    protected static final byte REG_OFF_YL = 0x12;
    protected static final byte REG_OFF_YH = 0x13;
    protected static final byte REG_OFF_HL = 0x14;
    protected static final byte REG_OFF_HH = 0x15;
    protected static final byte REG_STATUS = 0x1F;
    protected static final byte REG_POS_XL = 0x20;
    protected static final byte REG_POS_XH = 0x21;
    protected static final byte REG_POS_YL = 0x22;
    protected static final byte REG_POS_YH = 0x23;
    protected static final byte REG_POS_HL = 0x24;
    protected static final byte REG_POS_HH = 0x25;
    protected static final byte REG_VEL_XL = 0x26;
    protected static final byte REG_VEL_XH = 0x27;
    protected static final byte REG_VEL_YL = 0x28;
    protected static final byte REG_VEL_YH = 0x29;
    protected static final byte REG_VEL_HL = 0x2A;
    protected static final byte REG_VEL_HH = 0x2B;
    protected static final byte REG_ACC_XL = 0x2C;
    protected static final byte REG_ACC_XH = 0x2D;
    protected static final byte REG_ACC_YL = 0x2E;
    protected static final byte REG_ACC_YH = 0x2F;
    protected static final byte REG_ACC_HL = 0x30;
    protected static final byte REG_ACC_HH = 0x31;
    protected static final byte REG_POS_STD_XL = 0x32;
    protected static final byte REG_POS_STD_XH = 0x33;
    protected static final byte REG_POS_STD_YL = 0x34;
    protected static final byte REG_POS_STD_YH = 0x35;
    protected static final byte REG_POS_STD_HL = 0x36;
    protected static final byte REG_POS_STD_HH = 0x37;
    protected static final byte REG_VEL_STD_XL = 0x38;
    protected static final byte REG_VEL_STD_XH = 0x39;
    protected static final byte REG_VEL_STD_YL = 0x3A;
    protected static final byte REG_VEL_STD_YH = 0x3B;
    protected static final byte REG_VEL_STD_HL = 0x3C;
    protected static final byte REG_VEL_STD_HH = 0x3D;
    protected static final byte REG_ACC_STD_XL = 0x3E;
    protected static final byte REG_ACC_STD_XH = 0x3F;
    protected static final byte REG_ACC_STD_YL = 0x40;
    protected static final byte REG_ACC_STD_YH = 0x41;
    protected static final byte REG_ACC_STD_HL = 0x42;
    protected static final byte REG_ACC_STD_HH = 0x43;

    // Product ID register value
    protected static final byte PRODUCT_ID = 0x5F;

    // Conversion factors
    protected static final double RADIAN_TO_DEGREE = 180.0 / Math.PI;
    protected static final double DEGREE_TO_RADIAN = Math.PI / 180.0;

    // Conversion factor for the linear position registers. 16-bit signed
    // registers with a max value of 10 meters (394 inches) gives a resolution
    // of about 0.0003 mps (0.012 ips)
    protected static final double METER_TO_INT16 = 32768.0 / 10.0;
    protected static final double INT16_TO_METER = 1.0 / METER_TO_INT16;

    // Conversion factor for the linear velocity registers. 16-bit signed
    // registers with a max value of 5 mps (197 ips) gives a resolution of about
    // 0.00015 mps (0.006 ips)
    protected static final double MPS_TO_INT16 = 32768.0 / 5.0;
    protected static final double INT16_TO_MPS = 1.0 / MPS_TO_INT16;

    // Conversion factor for the linear acceleration registers. 16-bit signed
    // registers with a max value of 157 mps^2 (16 g) gives a resolution of
    // about 0.0048 mps^2 (0.49 mg)
    protected static final double MPSS_TO_INT16 = 32768.0 / (16.0 * 9.80665);
    protected static final double INT16_TO_MPSS = 1.0 / MPSS_TO_INT16;

    // Conversion factor for the angular position registers. 16-bit signed
    // registers with a max value of pi radians (180 degrees) gives a resolution
    // of about 0.00096 radians (0.0055 degrees)
    protected static final double RAD_TO_INT16 = 32768.0 / Math.PI;
    protected static final double INT16_TO_RAD = 1.0 / RAD_TO_INT16;

    // Conversion factor for the angular velocity registers. 16-bit signed
    // registers with a max value of 34.9 rps (2000 dps) gives a resolution of
    // about 0.0011 rps (0.061 degrees per second)
    protected static final double RPS_TO_INT16 = 32768.0 / (2000.0 * DEGREE_TO_RADIAN);
    protected static final double INT16_TO_RPS = 1.0 / RPS_TO_INT16;

    // Conversion factor for the angular acceleration registers. 16-bit signed
    // registers with a max value of 3141 rps^2 (180000 dps^2) gives a
    // resolution of about 0.096 rps^2 (5.5 dps^2)
    protected static final double RPSS_TO_INT16 = 32768.0 / (Math.PI * 1000.0);
    protected static final double INT16_TO_RPSS = 1.0 / RPSS_TO_INT16;


    // 2D pose structure, including x and y coordinates and heading angle.
    // Although pose is traditionally used for position and orientation, this
    // structure is also used for velocity and accleration by the OTOS driver
    public static class Pose2D {
        public double x;
        public double y;
        public double h;

        public Pose2D() {
            x = 0.0;
            y = 0.0;
            h = 0.0;
        }

        public Pose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public void set(Pose2D pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.h = pose.h;
        }
    }

    // Version register structure
    public static class Version {
        public byte minor;
        public byte major;

        public Version() {
            set((byte) 0);
        }

        public Version(byte value) {
            set(value);
        }

        public void set(byte value) {
            minor = (byte) (value & 0x0F);
            major = (byte) ((value >> 4) & 0x0F);
        }

        public byte get() {
            return (byte) ((major << 4) | minor);
        }
    }

    // Signal process config register structure
    public static class SignalProcessConfig {
        public boolean enLut;
        public boolean enAcc;
        public boolean enRot;
        public boolean enVar;

        public SignalProcessConfig() {
            set((byte) 0);
        }

        public SignalProcessConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            enLut = (value & 0x01) != 0;
            enAcc = (value & 0x02) != 0;
            enRot = (value & 0x04) != 0;
            enVar = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((enLut ? 0x01 : 0) | (enAcc ? 0x02 : 0) | (enRot ? 0x04 : 0) | (enVar ? 0x08 : 0));
        }
    }

    // Self-test config register structure
    public static class SelfTestConfig {
        public boolean start;
        public boolean inProgress;
        public boolean pass;
        public boolean fail;

        public SelfTestConfig() {
            set((byte) 0);
        }

        public SelfTestConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            start = (value & 0x01) != 0;
            inProgress = (value & 0x02) != 0;
            pass = (value & 0x04) != 0;
            fail = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((start ? 0x01 : 0) | (inProgress ? 0x02 : 0) | (pass ? 0x04 : 0) | (fail ? 0x08 : 0));
        }
    }

    // Status register structure
    public static class Status {
        public boolean warnTiltAngle;
        public boolean warnOpticalTracking;
        public boolean errorPaa;
        public boolean errorLsm;

        public Status() {
            set((byte) 0);
        }

        public Status(byte value) {
            set(value);
        }

        public void set(byte value) {
            warnTiltAngle = (value & 0x01) != 0;
            warnOpticalTracking = (value & 0x02) != 0;
            errorPaa = (value & 0x40) != 0;
            errorLsm = (value & 0x80) != 0;
        }

        public byte get() {
            return (byte) ((warnTiltAngle ? 0x01 : 0) | (warnOpticalTracking ? 0x02 : 0) | (errorPaa ? 0x40 : 0) | (errorLsm ? 0x80 : 0));
        }
    }

    protected DistanceUnit _distanceUnit = DistanceUnit.INCH;
    protected AngleUnit _angularUnit = AngleUnit.DEGREES;
    protected double _linearScalar = 0;
    protected double _angularScalar = 0;
    protected Pose2D _offset = new Pose2D(0, 0, 0);

    // Helpers to convert the OTOS notion of pose to-and-from the simulation's notion of pose:
    protected Pose2d simulationPose(Pose2D otosPose) {
        return new Pose2d(
                _distanceUnit.toInches(otosPose.x),
                _distanceUnit.toInches(otosPose.y),
                _angularUnit.toRadians(otosPose.h));
    }
    protected Pose2D otosPose(Pose2d simulationPose) {
        return new Pose2D(
                _distanceUnit.fromInches(simulationPose.position.x),
                _distanceUnit.fromInches(simulationPose.position.y),
                _angularUnit.fromRadians(simulationPose.heading.log()));
    }

    @Override
    protected boolean doInitialize() { return true; }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() { return HardwareDevice.Manufacturer.SparkFun; }

    @Override
    public String getDeviceName()
    {
        return "SparkFun Qwiic Optical Tracking Odometry Sensor";
    }

    public boolean begin() { return isConnected(); }
    public boolean isConnected() { return true; }
    public void getVersionInfo(Version hwVersion, Version fwVersion) {
        hwVersion.set((byte) 0);
        fwVersion.set((byte) 0);
    }
    public boolean selfTest() { return true; }
    public boolean calibrateImu() {
        return calibrateImu(255, true);
    }
    public boolean calibrateImu(int numSamples, boolean waitUntilDone) { return true; }
    public int getImuCalibrationProgress() { return 0; }
    public DistanceUnit getLinearUnit() {
        return _distanceUnit;
    }
    public void setLinearUnit(DistanceUnit unit) { _distanceUnit = unit; }
    public AngleUnit getAngularUnit() {
        return _angularUnit;
    }
    public void setAngularUnit(AngleUnit unit) { _angularUnit = unit; }
    public double getLinearScalar() { return _linearScalar; }
    public boolean setLinearScalar(double scalar) {
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;
        _linearScalar = scalar;
        return true;
    }
    public double getAngularScalar() { return _angularScalar; }
    public boolean setAngularScalar(double scalar) {
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;
        _angularScalar = scalar;
        return true;
    }
    public void resetTracking() {}
    public SignalProcessConfig getSignalProcessConfig() { return new SignalProcessConfig((byte) 0); };
    public void setSignalProcessConfig(SignalProcessConfig config) { }
    public Status getStatus() { return new Status((byte) 0); }
    public Pose2D getOffset() { return new Pose2D(_offset.x, _offset.y, _offset.h); }
    public void setOffset(Pose2D pose) { _offset = new Pose2D(_offset.x, _offset.y, _offset.h); }
    public Pose2D getPosition() { return otosPose(WilyCore.getPose()); }
    public void setPosition(Pose2D pose) { WilyCore.setStartPose(simulationPose(pose), null); }
    public Pose2D getVelocity() { return new Pose2D(0, 0, 0); }
    public Pose2D getAcceleration() { return new Pose2D(0, 0, 0); }
    public Pose2D getPositionStdDev() { return new Pose2D(0, 0, 0); }
    public Pose2D getVelocityStdDev() { return new Pose2D(0, 0, 0); }
    public Pose2D getAccelerationStdDev() { return new Pose2D(0, 0, 0); }
    public void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPosition());
        vel.set(getVelocity());
        acc.set(getAcceleration());
    }
    public void getPosVelAccStdDev(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPositionStdDev());
        vel.set(getVelocityStdDev());
        acc.set(getAccelerationStdDev());
    }
    public void getPosVelAccAndStdDev(Pose2D pos, Pose2D vel, Pose2D acc,
                                      Pose2D posStdDev, Pose2D velStdDev, Pose2D accStdDev) {
        getPosVelAcc(pos, vel, acc);
        getPosVelAccStdDev(posStdDev, velStdDev, accStdDev);
    }

    protected Pose2D regsToPose(byte[] rawData, double rawToXY, double rawToH) {
        return new Pose2D(0, 0, 0);
    }

    protected void poseToRegs(byte[] rawData, Pose2D pose, double xyToRaw, double hToRaw) { }
}
