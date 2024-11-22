package org.firstinspires.ftc.teamMentor.roadrunner;

import android.annotation.SuppressLint;
import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This class wraps Road Runner actions to implement easier-to-use support for state-machine
 * cooperative multitasking, with additional error checking to prevent robot destruction.
 * @noinspection unused
 */
@SuppressLint("DefaultLocale")
abstract public class RobotAction implements Action {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Private implementation

    // Description of the action:
    private String name = "";

    // Every Run() call should take 10ms or less. If a run() exceeds the following amount of time,
    // a warning is generated on the Driver Station:
    /** @noinspection FieldCanBeLocal*/
    private final double WARNING_TIMEOUT = 0.060; // 60 milliseconds

    // If a run() exceeds the following amount of time, an exception is throw and the robot is
    // stopped:
    /** @noinspection FieldCanBeLocal*/
    private final double WATCHDOG_TIMEOUT = 0.300; // 300 milliseconds

    // Start time of the action, zero if it's not currently active:
    private double actionStartTime;

    // True if the user has been warning about a run() taking too long:
    private boolean warned;

    // Current time, in seconds:
    private double time() { return System.nanoTime() * 1e-9; }

    // The following globals should only be accessed when holding 'runLock'. They are used to
    // communicate between the watchdog thread and the working threads.
    static private final Object runLock = new Object();
    static private volatile double runStartTime;
    static private volatile RobotAction runAction;

    // Global watchdog thread:
    static private volatile Thread watchDogThread;

    // Watchdog thread:
    class Watchdog extends Thread {
        Thread parentThread;
        Watchdog(Thread parentThread) {
            this.parentThread = parentThread;
        }
        public void run() {
            setName("RobotAction watchdog thread");

            // Loop forever:
            while (true) {
                // Grab the lock
                synchronized(runLock) {
                    if (runStartTime != 0) {
                        double elapsedTime = time() - runStartTime;
                        if (elapsedTime > WATCHDOG_TIMEOUT) {
                            // Raising an exception is not useful here because it would provide the
                            // stacktrace of this watchdog thread and not the culprit thread.
                            // Instead, spew an error message to the Driver Station and terminate the
                            // main thread:
                            String message;
                            if (runAction.name.isEmpty()) {
                                message = String.format("ERROR: Aborted, unidentified RobotAction is taking too long "
                                                + "(%.0fms and counting). Call SetName() to identify actions.",
                                        elapsedTime * 1000.0);
                            } else {
                                message = String.format("ERROR: Aborted, RobotAction '%s' is taking too long (%.0fms "
                                        + "and counting).", runAction.name, elapsedTime * 1000.0);
                            }
                            runAction.warned = true;
                            RobotLog.addGlobalWarningMessage(message);
                            Log.e("RobotAction", message);
                            runStartTime = 0;

                            // Terminate the original thread to prevent the robot from running into
                            // something and breaking:
                            parentThread.interrupt();
                        }
                    }
                }

                // Now that we're no longer holding the lock, sleep:
                try {
                    //noinspection BusyWait
                    sleep(100);
                } catch (InterruptedException e) {
                    // Exit our loop and terminate this thread when requested:
                    watchDogThread = null;
                    return;
                }
            }
        }
    }

    // This function is called by Road Runner. We, in turn, call the user's RobotAction.
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Start the watchdog thread if it hasn't already started:
        if (watchDogThread == null) {
            runStartTime = 0;
            runAction = null;
            watchDogThread = new Watchdog(Thread.currentThread());
            watchDogThread.start();
        }

        // Update the member field with the current telemetry packet:
        this.telemetryPacket = telemetryPacket;
        boolean callAgain;

        // Use globals to tell the watchdog thread that a run() has started:
        synchronized(runLock) {
            runAction = this;
            runStartTime = time();
        }

        // Invoke the run!
        if (actionStartTime == 0) {
            actionStartTime = time();
            callAgain = run(0.0);
        } else {
            callAgain = run(time() - actionStartTime);
        }
        double runTime = time() - runStartTime;

        // Let the watchdog thread know that the run() has completed:
        synchronized(runLock) {
            runAction = null;
            runStartTime = 0;
        }

        if ((runTime > WARNING_TIMEOUT) && (!warned)) {
            String message;
            if (name.isEmpty()) {
                message = String.format("WARNING: Unidentified RobotAction took %.0fms in its run() call, "
                        + "should take no more than 10ms. Call SetName() to identify actions.",
                        runTime * 1000.0);
            } else {
                message = String.format("WARNING: RobotAction '%s' took %.0fms in its run() call, "
                                + "should take no more than 10ms.",
                        name, runTime * 1000.0);
            }
            RobotLog.addGlobalWarningMessage(message);
            Log.w("RobotAction", message);
            warned = true;
        }
        if (!callAgain) {
            // The action is complete. Reset the start time in preparation for next time the
            // action is invoked:
            actionStartTime = 0;
        }
        return callAgain;
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        Action.super.preview(fieldOverlay);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Public interfaces.

    // This Dashboard telemetry packet will always be non-null when run() is called. It can be
    // used to send Field View drawing to the FTC Dashboard or other telemetry:
    public TelemetryPacket telemetryPacket;

    // Set the name of this action, primarily so it can be identified if it takes too long
    // to run:
    public void setName(String name) {
        this.name = name;
    }

    // Implement this run() method for your action. Return true if the action should run again,
    // false if it's complete. 'elapsedTime' is the time, in seconds, since the action began.
    //
    // Your run() implementation should be fast, no more than 10ms. Anything longer and
    // the driving loop can't be updated in a timely fashion. Your code should *never* loop
    // in the run() call waiting on the hardware. Instead, poll the hardware and return 'true'
    // if the hardware's not ready yet, and you'll be called again:
    public abstract boolean run(double elapsedTime);

    // Returns true if the action is still actively running:
    public boolean isRunning() {
        return actionStartTime != 0;
    }
}