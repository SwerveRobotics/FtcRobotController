package com.qualcomm.robotcore.robocol;

import java.net.DatagramPacket;

public class RobocolDatagram {

    /** the system-level packet over which we are a wrapper */
    private DatagramPacket packet;

    public RobocolDatagram(DatagramPacket packet) {
        this.packet = packet;
    }

    /**
     * Gets the payload of this datagram packet
     * @return byte[] data
     */
    public byte[] getData() {
        return packet.getData();
    }
}
