package org.firstinspires.ftc.robotcore.internal.network;

import android.annotation.Nullable;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.RobocolDatagram;

import java.util.concurrent.CopyOnWriteArrayList;

public class NetworkConnectionHandler {
    private static final NetworkConnectionHandler theInstance = new NetworkConnectionHandler();

    public static NetworkConnectionHandler getInstance() {
        return theInstance;
    }

    protected final RecvLoopCallbackChainer theRecvLoopCallback = new RecvLoopCallbackChainer();

    // Sidekick's Wily Works loop callback:
    public static RecvLoopRunnable.RecvLoopCallback wilyRecvLoopCallback;

    protected class RecvLoopCallbackChainer implements RecvLoopRunnable.RecvLoopCallback {

        protected final CopyOnWriteArrayList<RecvLoopRunnable.RecvLoopCallback> callbacks = new CopyOnWriteArrayList<RecvLoopRunnable.RecvLoopCallback>();

        void push(@Nullable RecvLoopRunnable.RecvLoopCallback callback) {
            wilyRecvLoopCallback = callback;
        }

        void remove(@Nullable RecvLoopRunnable.RecvLoopCallback callback) {
        }

        @Override public CallbackResult packetReceived(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override public CallbackResult peerDiscoveryEvent(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override public CallbackResult heartbeatEvent(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override
        public CallbackResult commandEvent(Command command) throws RobotCoreException {
            return null;
        }

        @Override public CallbackResult telemetryEvent(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override public CallbackResult gamepadEvent(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override public CallbackResult emptyEvent(RobocolDatagram packet) throws RobotCoreException {
            return CallbackResult.NOT_HANDLED;
        }

        @Override public CallbackResult reportGlobalError(String error, boolean recoverable) {
            return CallbackResult.NOT_HANDLED;
        }
    }
}
