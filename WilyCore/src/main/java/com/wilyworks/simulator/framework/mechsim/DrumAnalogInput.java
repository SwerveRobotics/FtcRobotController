package com.wilyworks.simulator.framework.mechsim;

import com.wilyworks.simulator.framework.WilyAnalogInput;

// Hooked class for measuring the position of the drum:
public class DrumAnalogInput extends WilyAnalogInput {
    DecodeSlowBotMechSim mechSim;

    DrumAnalogInput(DecodeSlowBotMechSim mechSim) {
        this.mechSim = mechSim;
    }

    // Return a voltage that is proportional to the drum location, with some variation:
    @Override
    public double getVoltage() {
        double variation = -0.1 + Math.random() * 0.2; // random() generates numbers between 0 and 1
        return 3.5 * mechSim.actualDrumPosition + variation;
    }
}
