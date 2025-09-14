package com.qualcomm.robotcore.hardware;

public interface LightSensor extends HardwareDevice {

    /**
     * Get the amount of light detected by the sensor, scaled and cliped to a range
     * which is a pragmatically useful sensitivity. Note that returned values INCREASE as
     * the light energy INCREASES.
     * @return amount of light, on a scale of 0.0 to 1.0
     */
    double getLightDetected();

    /**
     * Returns a signal whose strength is proportional to the intensity of the light measured.
     * Note that returned values INCREASE as the light energy INCREASES. The units in which
     * this signal is returned are unspecified.
     * @return a value proportional to the amount of light detected, in unspecified units
     */
    double getRawLightDetected();

    /**
     * Returns the maximum value that can be returned by {@link #getRawLightDetected}.
     * @return the maximum value that can be returned by getRawLightDetected
     * @see #getRawLightDetected
     */
    double getRawLightDetectedMax();

    /**
     * Enable the LED light
     * @param enable true to enable; false to disable
     */
    void enableLed(boolean enable);

    /**
     * Status of this sensor, in string form
     * @return status
     */
    String status();

}
