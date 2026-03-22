/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.robotcore.util;

/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import androidx.annotation.Nullable;

import com.google.gson.TypeAdapter;
import com.google.gson.annotations.JsonAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

import java.io.IOException;
import java.io.Serializable;
import java.util.HashMap;
import java.util.UUID;

/**
 */
@SuppressWarnings("WeakerAccess")
@JsonAdapter(SerialNumber.GsonTypeAdapter.class)
public abstract class SerialNumber implements Serializable {

    protected SerialNumber(String serialNumberString) {
    }

    public static SerialNumber createFake() {
        return null;
    }

    public static SerialNumber createEmbedded() { return null; }

    public static SerialNumber fromString(@Nullable String serialNumberString) {
        return null;
    }

    public static @Nullable SerialNumber fromStringOrNull(@Nullable String serialNumberString) {
        return null;
    }

    public static @Nullable SerialNumber fromUsbOrNull(@Nullable String serialNumberString) {
        return null;
    }

    /** Makes up a serial-number-like-thing for USB devices that internally lack a serial number. */
    public static SerialNumber fromVidPid(int vid, int pid, String connectionPath) {
        return null;
    }

    //------------------------------------------------------------------------------------------------
    // Gson
    //------------------------------------------------------------------------------------------------

    static class GsonTypeAdapter extends TypeAdapter<SerialNumber> {
        @Override public void write(JsonWriter writer, SerialNumber serialNumber) throws IOException {
            if (serialNumber==null) {
                writer.nullValue();
            } else {
                writer.value(serialNumber.getString());
            }
        }

        @Override public SerialNumber read(JsonReader reader) throws IOException {
            return SerialNumber.fromStringOrNull(reader.nextString());
        }
    }

    //------------------------------------------------------------------------------------------------
    // Accessing
    //------------------------------------------------------------------------------------------------

    public boolean isVendorProduct() {
        return false;
    }

    /**
     * Returns whether the indicated serial number is one of the legacy
     * fake serial number forms or not.
     * @return whether the the serial number is a legacy fake form of serial number
     */
    public boolean isFake() {
        return false;
    }

    /**
     * Returns whether the serial number is one of an actual USB device.
     */
    public boolean isUsb() {
        return false;
    }

    /**
     * Returns whether the serial number is the one used for the embedded
     * Expansion Hub inside a Rev Control Hub.
     */
    public boolean isEmbedded() {
        return false;
    }

    /**
     * Returns the string contents of the serial number. Result is not intended to be
     * displayed to humans.
     * @see #toString()
     */
    public String getString() { return ""; }


    /**
     */
    public SerialNumber getScannableDeviceSerialNumber() {
        return this;
    }

    //------------------------------------------------------------------------------------------------
    // Comparison
    //------------------------------------------------------------------------------------------------

    public boolean matches(Object pattern) {
        return this.equals(pattern);
    }

    @Override
    public boolean equals(Object object) {
        if (object == null) return false;
        if (object == this) return true;

        if (object instanceof SerialNumber) {
            return false;
        }

        if (object instanceof String) {
            return this.equals((String)object);
        }

        return false;
    }

    // separate method to avoid annoying Android Studio inspection warnings when comparing SerialNumber against String
    public boolean equals(String string) {
        return false;
    }

    @Override
    public int hashCode() {
        return 0;
    }

    //------------------------------------------------------------------------------------------------
    // Serial number display name management
    //------------------------------------------------------------------------------------------------

    protected static final HashMap<String,String> deviceDisplayNames = new HashMap<String, String>();

    public static void noteSerialNumberType(SerialNumber serialNumber, String typeName) {
        synchronized (deviceDisplayNames) {
            deviceDisplayNames.put(serialNumber.getString(), Misc.formatForUser("%s [%s]", typeName, serialNumber));
        }
    }

    public static String getDeviceDisplayName(SerialNumber serialNumber) {
        synchronized (deviceDisplayNames) {
            String result = deviceDisplayNames.get(serialNumber.getString());
            return result;
        }
    }

}
