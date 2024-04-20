package org.firstinspires.ftc.team417_CENTERSTAGE;

import java.util.ArrayList;

public class Filters {

    private ArrayList<Double> values = new ArrayList<>();

    private double ave() {
        double total = 0;
        for (double v : values)
            total += v;
        return total / values.size();
    }

    public double slidingWindow(int range, double value) {
        values.add(0, value);
        if (values.size() > range)
            values.remove(values.size() - 1);

        return ave();
    }

    public double slidingWindow(int range, double value, double highEnd, double lowEnd) {
        if (value > lowEnd && value < highEnd)
            return slidingWindow(range, value);
        return ave();
    }
}
