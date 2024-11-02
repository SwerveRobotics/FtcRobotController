package org.swerverobotics.ftc;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType()
@DeviceProperties(name = "SparkFun Ultrasonic Distance Sensor", description = "QWIIC Ultrasonic Distance Sensor TCT40", xmlTag = "QWIIC_ULTRASONIC_DISTANCE_SENSOR")
public class UltrasonicDistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    final int TRIGGER_AND_READ_REGISTER = 0x01;

    public double getDistance(DistanceUnit distanceUnit) {
        int distance = 0;
        return distanceUnit.fromUnit(DistanceUnit.MM, distance);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.SparkFun;
    }

    @Override
    public String getDeviceName() {
        return "SparkFun Ultrasonic Distance Sensor";
    }

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2F);

    public UltrasonicDistanceSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }
}
