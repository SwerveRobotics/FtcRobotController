package org.firstinspires.ftc.team417.color;

public class Lab {
    public double L;
    public double a;
    public double b;

    public Lab(double L, double a, double b) {
        this.L = L;
        this.a = a;
        this.b = b;
    }

    public boolean equals(Lab color, double epsilon) {
        return Math.abs(color.L - L) < epsilon &&
                Math.abs(color.a - a) < epsilon &&
                Math.abs(color.b - b) < epsilon;
    }

    @Override
    public String toString() {
        return "Lab{" +
                "L=" + L +
                ", a=" + a +
                ", b=" + b +
                '}';
    }
}
