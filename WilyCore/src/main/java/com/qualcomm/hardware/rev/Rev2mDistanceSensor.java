package com.qualcomm.hardware.rev;

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "@string/rev_laser_sensor_name", description = "@string/rev_laser_sensor_name", xmlTag = "REV_VL53L0X_RANGE_SENSOR", compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class Rev2mDistanceSensor extends VL53L0X {
    public Rev2mDistanceSensor(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }

    @Override
    public String getDeviceName() {
        return "REV 2M ToF Distance Sensor";
    }
}
