package org.firstinspires.ftc.robotcore.internal.network;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robocol.RobocolDatagram;

public class RecvLoopRunnable {
    public interface RecvLoopCallback {
        CallbackResult packetReceived(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult peerDiscoveryEvent(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult heartbeatEvent(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult commandEvent(Command command) throws RobotCoreException;
        CallbackResult telemetryEvent(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult gamepadEvent(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult emptyEvent(RobocolDatagram packet) throws RobotCoreException;
        CallbackResult reportGlobalError(String error, boolean recoverable);
    }
}
