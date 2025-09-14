package fi.iki.elonen;

/*
 * #%L
 * NanoHttpd-Websocket
 * %%
 * Copyright (C) 2012 - 2015 nanohttpd
 * %%
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the nanohttpd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * #L%
 */

import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.Charset;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;

public abstract class NanoWSD extends NanoHTTPD {

    public NanoWSD(int port) {
        super(port);
    }

    public NanoWSD(String hostname, int port) {
        super(hostname, port);
    }

    public static enum State {
        UNCONNECTED,
        CONNECTING,
        OPEN,
        CLOSING,
        CLOSED
    }

    public static class WebSocketFrame {

        public String getTextPayload() {
            return "";
        }

        public byte[] getBinaryPayload() { return new byte[0]; }

        public static enum CloseCode {
            NormalClosure(1000),
            GoingAway(1001),
            ProtocolError(1002),
            UnsupportedData(1003),
            NoStatusRcvd(1005),
            AbnormalClosure(1006),
            InvalidFramePayloadData(1007),
            PolicyViolation(1008),
            MessageTooBig(1009),
            MandatoryExt(1010),
            InternalServerError(1011),
            TLSHandshake(1015);

            public static CloseCode find(int value) {
                for (CloseCode code : values()) {
                    if (code.getValue() == value) {
                        return code;
                    }
                }
                return null;
            }

            private final int code;

            private CloseCode(int code) {
                this.code = code;
            }

            public int getValue() {
                return this.code;
            }
        }
    }

    protected abstract WebSocket openWebSocket(IHTTPSession handshake);

    public static abstract class WebSocket {

        public WebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
        }

        public boolean isOpen() { return false; }

        protected abstract void onOpen();

        protected abstract void onClose(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote);

        protected abstract void onMessage(WebSocketFrame message);

        protected abstract void onPong(WebSocketFrame pong);

        protected abstract void onException(IOException exception);

        /**
         * Debug method. <b>Do not Override unless for debug purposes!</b>
         *
         * @param frame
         *            The received WebSocket Frame.
         */
        protected void debugFrameReceived(WebSocketFrame frame) {
        }

        /**
         * Debug method. <b>Do not Override unless for debug purposes!</b><br>
         * This method is called before actually sending the frame.
         *
         * @param frame
         *            The sent WebSocket Frame.
         */
        protected void debugFrameSent(WebSocketFrame frame) {
        }

        public void close(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) throws IOException {
        }

        private void doClose(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
        }

        // --------------------------------IO--------------------------------------

//        public NanoHTTPD.IHTTPSession getHandshakeRequest() {
//            return this.handshakeRequest;
//        }
//
//        public NanoHTTPD.Response getHandshakeResponse() {
//            return this.handshakeResponse;
//        }

        private void handleCloseFrame(WebSocketFrame frame) throws IOException {
        }

        private void handleFrameFragment(WebSocketFrame frame) throws IOException {
        }

        private void handleWebsocketFrame(WebSocketFrame frame) throws IOException {
        }

        public void ping(byte[] payload) throws IOException {
        }

        private void readWebsocket() {
        }

        public void send(byte[] payload) throws IOException {
        }

        public void send(String payload) throws IOException {
        }

        public synchronized void sendFrame(WebSocketFrame frame) throws IOException {
        }
    }
}
