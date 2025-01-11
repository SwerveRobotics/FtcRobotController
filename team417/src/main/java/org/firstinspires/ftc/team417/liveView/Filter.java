package org.firstinspires.ftc.team417.liveView;


import java.util.ArrayList;

public class Filter {

    ArrayList<Double> values = new ArrayList<>();

    public double slidingWindow(double value, int size) {
        values.add(value);

        if (values.size() > size) {
            values.remove(0);
        }

        double sum = 0;

        for (int i = 0; i < values.size(); i++) {
            sum += values.get(i);
        }

        return sum / values.size();
    }
}
