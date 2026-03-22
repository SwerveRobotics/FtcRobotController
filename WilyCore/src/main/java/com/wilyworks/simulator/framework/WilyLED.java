package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.LED;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;

/**
 * Wily Works LED implementation.
 */
public class WilyLED extends LED {
    // Assume that every digital channels is a REV LED indicator. Doesn't hurt if that's not
    // the case:
    public boolean enable = true; // They're always on by default
    public double x;
    public double y;
    public boolean isRed;
    public WilyLED(String deviceName) {
        WilyWorks.Config.LEDIndicator wilyLed = null;
        for (WilyWorks.Config.LEDIndicator led: WilyCore.config.ledIndicators) {
            if (led.name.equals(deviceName)) {
                wilyLed = led;
            }
        }
        if (wilyLed != null) {
            x = wilyLed.x;
            y = wilyLed.y;
            isRed = wilyLed.isRed;
        } else {
            isRed = !(deviceName.toLowerCase().contains("green"));
        }
    }

    @Override
    public void enable(boolean enableLed) { this.enable = enableLed; }

    @Override
    public void enableLight(boolean enable) { enable(enable); }

    @Override
    public boolean isLightOn() {
        return enable;
    }
}

