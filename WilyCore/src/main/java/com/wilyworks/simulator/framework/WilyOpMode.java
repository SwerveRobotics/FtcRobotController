package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.wilyworks.simulator.WilyCore;

public abstract class WilyOpMode extends OpMode {

    abstract public void runOpMode() throws InterruptedException;

    public void waitForStart() {
        while (!isStarted())
            sleep(30);
    }

    public final boolean opModeIsActive() { return WilyCore.status.state != WilyCore.State.STOPPED; }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final boolean isStopRequested() { return WilyCore.status.state == WilyCore.State.STOPPED; }

    public final boolean isStarted() { return WilyCore.status.state == WilyCore.State.STARTED; }
}
