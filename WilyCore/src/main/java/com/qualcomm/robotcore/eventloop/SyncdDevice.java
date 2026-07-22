package com.qualcomm.robotcore.eventloop;

public interface SyncdDevice {

    interface Manager {
        void registerSyncdDevice(SyncdDevice device);
        void unregisterSyncdDevice(SyncdDevice device);
    }
}
