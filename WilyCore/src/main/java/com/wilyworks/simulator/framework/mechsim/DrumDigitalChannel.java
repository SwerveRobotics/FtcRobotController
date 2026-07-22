package com.wilyworks.simulator.framework.mechsim;

import com.wilyworks.simulator.framework.WilyDigitalChannel;

public class DrumDigitalChannel extends WilyDigitalChannel {
    DecodeSlowBotMechSim mechSim;
    int index;

    DrumDigitalChannel(DecodeSlowBotMechSim mechSim, int index) {
        this.mechSim = mechSim;
        this.index = index;
    }

    @Override
    public boolean getState() {
        return super.getState();
    }
}
