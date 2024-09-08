/**
 * Extended version of the SparkFun OTOS driver. This provides additional functionality over
 * the stock driver.
 */

package org.firstinspires.ftc.team417;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.Arrays;

@I2cDeviceType
@DeviceProperties(
        name = "SparkFun OTOS Extended",
        xmlTag = "SparkFunOTOS2",
        description = "SparkFun Qwiic Optical Tracking Odometry Sensor Extended"
)
public class SparkFunOTOSExtended extends SparkFunOTOS {
    public SparkFunOTOSExtended(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }

    // Modified version of poseToRegs to fix pose setting issue
    // see https://discord.com/channels/225450307654647808/1246977443030368349/1271702497659977760
    @Override
    protected void poseToRegs(byte[] rawData, Pose2D pose, double xyToRaw, double hToRaw) {
        // Convert pose units to raw data
        short rawX = (short) (_distanceUnit.toMeters(pose.x) * xyToRaw);
        short rawY = (short) (_distanceUnit.toMeters(pose.y) * xyToRaw);
        short rawH = (short) (_angularUnit.toRadians(pose.h) * hToRaw);

        // Store raw data in buffer
        rawData[0] = (byte) (rawX & 0xFF);
        rawData[1] = (byte) ((rawX >> 8) & 0xFF);
        rawData[2] = (byte) (rawY & 0xFF);
        rawData[3] = (byte) ((rawY >> 8) & 0xFF);
        rawData[4] = (byte) (rawH & 0xFF);
        rawData[5] = (byte) ((rawH >> 8) & 0xFF);
    }

    /**
     * Gets the position, and velocity measured by the OTOS in a single burst read
     * @param pos Position measured by the OTOS
     * @param vel Velocity measured by the OTOS
     */
    public void getPosVel(Pose2D pos, Pose2D vel) {
        // Read all pose registers
        byte[] rawData = deviceClient.read(REG_POS_XL, 12);

        // Convert raw data to pose units
        pos.set(regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD));
        vel.set(regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS));
    }
}