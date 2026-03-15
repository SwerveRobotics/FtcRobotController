package com.qualcomm.hardware.lynx;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class LynxModule implements HardwareDevice {

    BulkCachingMode bulkCachingMode = BulkCachingMode.OFF;

    protected final Object bulkCachingLock;
    protected BulkData lastBulkData = new BulkData(); // guarded by bulkCachingLock

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    public enum BulkCachingMode
    {
        OFF,
        MANUAL,
        AUTO
    }

    public LynxModule() {
        this.bulkCachingLock = new Object();
    }

    public static class BulkData {
        private final LynxGetBulkInputDataResponse resp = new LynxGetBulkInputDataResponse();
    }

    public void setBulkCachingMode(BulkCachingMode mode) {
        this.bulkCachingMode = mode;
    }

    public BulkCachingMode getBulkCachingMode() {
        return bulkCachingMode;
    }

    public SerialNumber getSerialNumber()
    {
        return new SerialNumber("") {
            @Override
            public boolean isUsb() {
                return true;
            }
        };
    }

    public boolean isParent()
    {
        return true;
    }

    public int getModuleAddress()
    {
        return 173; // Magic number for HomeBot
    }

    /**
     * Returns the current consumption of the whole module.
     * @param unit current units
     * @return module current consumption
     */
    public double getCurrent(CurrentUnit unit)
    {
        return 0;
    }

    /**
     * Returns the current consumption of the GPIO bus.
     * @param unit current units
     * @return GPIO bus current consumption
     */
    public double getGpioBusCurrent(CurrentUnit unit)
    {
        return 0;
    }

    /**
     * Returns the current consumption of the I2C bus.
     * @param unit current units
     * @return I2C bus current consumption
     */
    public double getI2cBusCurrent(CurrentUnit unit)
    {
        return 0;
    }

    /**
     * Returns the input (battery) voltage.
     * @param unit voltage units
     * @return input voltage
     */
    public double getInputVoltage(VoltageUnit unit)
    {
        return 0;
    }

    /**
     * Returns the auxiliary (5V) voltage.
     * @param unit voltage units
     * @return auxiliary voltage
     */
    public double getAuxiliaryVoltage(VoltageUnit unit)
    {
        return 0;
    }

    /**
     * Returns the module temperature.
     * @param unit temperature units
     * @return module temperature
     */
    public double getTemperature(TempUnit unit)
    {
        return 0;
    }

}
