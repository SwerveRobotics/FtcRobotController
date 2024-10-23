package org.firstinspires.ftc.team417.distance;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType()
@DeviceProperties(name = "QWIIC Ultrasonic Distance Sensor", description = "Sparkfun Ultrasonic Distance Sensor", xmlTag = "QWIIC_ULTRASONIC_DISTANCE_SENSOR")
public class UltrasoundDistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    final int TRIGGER_AND_READ_REGISTER = 0x01;

    public int getDistance() {
        byte[] rawData = deviceClient.read(TRIGGER_AND_READ_REGISTER, 2);
        return ((rawData[0] & 0xff) << 8) | (rawData[1] & 0xff);
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "QWIIC Ultrasonic Distance Sensor";
    }

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2F);

    public UltrasoundDistanceSensor(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
    }
}
