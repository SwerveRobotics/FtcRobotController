package com.qualcomm.robotcore.eventloop.opmode;

public interface OpModeManagerNotifier
{
    interface Notifications
    {
        /** The indicated OpMode is just about to be initialized. */
        void onOpModePreInit(OpMode opMode);

        /** The indicated OpMode is just about to be started. */
        void onOpModePreStart(OpMode opMode);

        /** The indicated OpMode has just been stopped. */
        void onOpModePostStop(OpMode opMode);
    }

    /**
     * Registers an object as explicitly interested in receiving notifications as
     * to the coming and going of OpModes.
     * @param listener the object which is to receive notifications
     * @return the currently active OpMode at the instant of registration
     * @see #unregisterListener(Notifications)
     */
    OpMode registerListener(OpModeManagerNotifier.Notifications listener);

    /**
     * Unregisters a previously registered listener. If the provided listener is in
     * fact not currently registered, the call has no effect.
     * @param listener the listener to be unregistered.
     */
    void unregisterListener(OpModeManagerNotifier.Notifications listener);
}
