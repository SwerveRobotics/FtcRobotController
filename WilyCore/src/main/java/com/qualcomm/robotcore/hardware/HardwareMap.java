package com.qualcomm.robotcore.hardware;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.wilyworks.simulator.framework.WilyHardwareMap;

public class HardwareMap extends WilyHardwareMap {
    public HardwareMap() {
        super(null, null);
    }
    public HardwareMap(Context appContext, OpModeManagerNotifier notifier) {
        super(appContext, notifier);
    }
}
