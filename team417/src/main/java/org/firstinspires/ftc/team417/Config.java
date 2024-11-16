package org.firstinspires.ftc.team417;

public class Config {
    public static boolean used = false;

    public enum Robots {
        FAST_BOT,
        SLOW_BOT
    }

    public static Robots robot;

    public static boolean useDistance;

    public static boolean useReliableAuto;

    public enum Alliances {
        BLUE,
        RED
    }

    public static Alliances alliance;

    public enum Locations {
        OBSERVATION,
        NET
    }

    public static Locations location;

    public static double minWaitTime = 0.0;
    public static double maxWaitTime = 15.0;
    public static double scaleWaitTime = 1.0;

    public static double waitTime;
}
