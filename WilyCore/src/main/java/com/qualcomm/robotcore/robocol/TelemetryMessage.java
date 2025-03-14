package com.qualcomm.robotcore.robocol;

import java.nio.charset.Charset;
import java.util.LinkedHashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class TelemetryMessage  {

    public static final String DEFAULT_TAG = "TELEMETRY_DATA";
    private static final Charset CHARSET = Charset.forName("UTF-8");

    private final Map<String, String> dataStrings = new LinkedHashMap<String, String>();  // linked so as to preserve addition order as iteration order
    private final Map<String, Float> dataNumbers = new LinkedHashMap<String, Float>();

    protected static String getKey(int iLine)
    {
        // Keys must be unique. If they start with nul, then they're not shown on the driver display.
        // Historically, they were always shown, and sorted, so we used an *increasing sequence*
        // of unrenderable strings.
        return String.format("\0%c", 0x180 + iLine);
    }

    public synchronized Map<String, String> getDataStrings() {
        return dataStrings;
    }

    public TelemetryMessage() {
        dataStrings.put(getKey(0), "Line zero");
        dataStrings.put(getKey(1), "Line one");
    }
}