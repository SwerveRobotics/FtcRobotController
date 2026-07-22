package com.qualcomm.ftccommon;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

public class FtcEventLoop {
    public OpModeManagerImpl getOpModeManager() {
        return new OpModeManagerImpl();
    }
}
