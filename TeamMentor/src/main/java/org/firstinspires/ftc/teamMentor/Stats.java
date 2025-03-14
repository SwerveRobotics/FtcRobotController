package org.firstinspires.ftc.teamMentor;

public class Stats {
    static Stats loop = new Stats();
    class Mode {
        public static final int REGULAR = 0;
        public static final int FINE = 1;
        //--------------------------------
        public static final int COUNT = 2;
    }
    String[] modeNames = {"Regular", "Fine"};

    class Counters {
        public int loopCount = 0; // Number of loops
        public double loopTime = 0; // Total time spent in loops, seconds
        public int ioCount = 0; // Number of I/O operations
        public double ioTime = 0; // Total time spent in I/O operations, seconds
    }

    int mode = Mode.REGULAR; // Mode of the current loop
    Counters[] stats = new Counters[Mode.COUNT]; // Statistics for each mode
    double loopStartTime = 0; // Time when the loop started, seconds
    double ioStartTime = 0; // Time when the I/O operations started, seconds

    Stats() {
        for (int i = 0; i < Mode.COUNT; i++) {
            stats[i] = new Counters();
        }
    }

    // Call at the beginning of the loop:
    void update(int newMode) {
        // Complete the previous loop, if any:
        double time = System.nanoTime() * 1e-9;
        if (loopStartTime != 0) {
            double loopTime = time - loopStartTime;
            stats[mode].loopTime += loopTime;
            stats[mode].loopCount++;
        }

        mode = newMode;
        loopStartTime = time;
    }

    // Query the statistics:
    String getStats() {
        String result = "Loop times:\n";
        for (int i = 0; i < Mode.COUNT; i++) {
            Counters s = stats[i];
            if (s.loopCount > 0) {
                result += String.format("&emsp;%s: %.1fms (%.1fms I/O, %.1fms other)\n",
                        modeNames[i], s.loopTime * 1000 / s.loopCount, s.ioTime * 1000 / s.loopCount,
                        (s.loopTime - s.ioTime) * 1000 / s.loopCount);
            }
        }
        return result;
    }

    //----------------------------------------------------------------------------------------------
    // Public APIs
    public static void beginLoop(int mode) {
        loop.update(mode);
    }

    // Suspend loop tracking, as when waiting for the Start button to be pressed:
    public static void suspend() {
        loop.loopStartTime = 0;
    }

    // Call this before doing an I/O operation.
    public static void beginIo() {
        loop.ioStartTime = System.nanoTime() * 1e-9;
    }

    // Call this after an I/O operation.
    public static void endIo() {
        double time = System.nanoTime() * 1e-9;
        loop.stats[loop.mode].ioCount++;
        loop.stats[loop.mode].ioTime += time - loop.ioStartTime;
        loop.ioStartTime = 0;
    }

    public static String getString() {
        return loop.getStats();
    }
}
