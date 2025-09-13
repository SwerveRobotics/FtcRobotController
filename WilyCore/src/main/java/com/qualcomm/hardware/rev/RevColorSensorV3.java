package com.qualcomm.hardware.rev;

import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteOrder;
import java.util.Locale;

@I2cDeviceType
@DeviceProperties(name = "@string/rev_color_sensor_v3_name",
        description = "@string/rev_color_sensor_v3_description",
        xmlTag = "RevColorSensorV3",
        compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class RevColorSensorV3 extends BroadcomColorSensorImpl implements DistanceSensor, OpticalDistanceSensor, ColorRangeSensor
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected static final double apiLevelMin = 0.0;
    protected static final double apiLevelMax = 1.0;

    /**
     * Experimentally determined constants for converting optical measurements to distance.
     */
    double aParam = 325.961;
    double binvParam = -0.75934;
    double cParam = 26.980;
    double maxDist = 6.0; // inches

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public RevColorSensorV3(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(RevColorSensorV3.Parameters.createForAPDS9151(), deviceClient, deviceClientIsOwned);
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public String getDeviceName()
    {
        return "Rev Color Sensor v3";
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

    @Override
    public HardwareDevice.Manufacturer getManufacturer()
    {
        return Manufacturer.Broadcom;
    }

    //----------------------------------------------------------------------------------------------
    // OpticalDistanceSensor / LightSensor
    //----------------------------------------------------------------------------------------------

    @Override public double getLightDetected()
    {
        return Range.clip(
                Range.scale(getRawLightDetected(), 0, getRawLightDetectedMax(), apiLevelMin, apiLevelMax),
                apiLevelMin, apiLevelMax);
    }

    @Override public double getRawLightDetected()
    {
        return rawOptical();
    }

    @Override public double getRawLightDetectedMax()
    {
        return parameters.proximitySaturation;
    }

    @Override
    public void enableLed(boolean enable) {

    }

    @Override public String status()
    {
        return String.format(Locale.getDefault(), "%s on %s", getDeviceName(), getConnectionInfo());
    }

    //----------------------------------------------------------------------------------------------
    // DistanceSensor
    //----------------------------------------------------------------------------------------------

    /**
     * Returns a calibrated, linear sense of distance as read by the infrared proximity
     * part of the sensor. Distance is measured to the plastic housing at the front of the
     * sensor.
     *
     * Natively, the raw optical signal follows an inverse square law. Here, parameters have
     * been fitted to turn that into a <em>linear</em> measure of distance. The function fitted
     * was of the form:
     *
     *      RawOptical = a * distance^b + c
     *
     * The calibration was performed with proximity sensor pulses set to 32, LED driver current set
     * to 125ma, and measurement rate set to 100ms. If the end user chooses to use different
     * settings, the device will need to be recalibrated.
     *
     * Additionally, calibration was performed using card stock measured head on. Actual raw values
     * measured will depend on reflectivity and angle of surface. End user should experimentally
     * verify calibration is appropriate for their own application.
     *
     * @param unit  the unit of distance in which the result should be returned
     * @return      the currently measured distance in the indicated units. will always be between
     *              0.25 and 6.0 inches.
     */
    @Override public double getDistance(DistanceUnit unit)
    {
        int rawOptical = rawOptical();
        double inOptical = inFromOptical(rawOptical);
        return unit.fromUnit(DistanceUnit.INCH, inOptical);
    }

    /**
     * Converts a raw optical inverse-square reading into a fitted, calibrated linear reading in
     * INCHES.
     */
    protected double inFromOptical(int rawOptical)
    {
        // can't have a negative number raised to a fractional power. In this situation need to
        // return max value
        if(rawOptical <= cParam) return maxDist;

        // compute the distance based on an inverse power law, i.e.
        //  distance = ((RawOptical - c)/a)^(1/b)
        double dist = Math.pow((rawOptical - cParam)/aParam, binvParam);

        // distance values change very rapidly with small values of RawOptical and become
        // impractical to fit to a distribution. Returns are capped to an experimentally determined
        // max value.
        return Math.min(dist, maxDist);
    }

    //----------------------------------------------------------------------------------------------
    // Raw sensor data
    //----------------------------------------------------------------------------------------------

    public int rawOptical()
    {
        return 0; // return (readUnsignedShort(Register.PS_DATA, ByteOrder.LITTLE_ENDIAN) & 0x7FF);
    }
}
