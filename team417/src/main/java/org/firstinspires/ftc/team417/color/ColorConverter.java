package org.firstinspires.ftc.team417.color;

public class ColorConverter {
    private static final double REF_X = 95.047;
    private static final double REF_Y = 100.000;
    private static final double REF_Z = 108.883;

    public static Lab rgbToLab(double r, double g, double b) {
        double[] xyz = rgbToXyz(r, g, b);
        return xyzToLab(xyz[0], xyz[1], xyz[2]);
    }

    private static double[] rgbToXyz(double r, double g, double b) {
        r = r / 255.0;
        g = g / 255.0;
        b = b / 255.0;

        r = (r > 0.04045) ? Math.pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
        g = (g > 0.04045) ? Math.pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
        b = (b > 0.04045) ? Math.pow((b + 0.055) / 1.055, 2.4) : b / 12.92;

        r *= 100;
        g *= 100;
        b *= 100;

        double x = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
        double y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
        double z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;

        return new double[]{x, y, z};
    }

    private static Lab xyzToLab(double x, double y, double z) {
        x /= REF_X;
        y /= REF_Y;
        z /= REF_Z;

        x = transformXyz(x);
        y = transformXyz(y);
        z = transformXyz(z);

        double l = 116 * y - 16;
        double a = 500 * (x - y);
        double b = 200 * (y - z);

        return new Lab(l, a, b);
    }

    private static double transformXyz(double t) {
        final double delta = 6.0 / 29.0;
        final double deltaCube = delta * delta * delta;
        return t > deltaCube ? Math.pow(t, 1.0/3.0) : (t / (3 * delta * delta)) + 4.0/29.0;
    }
}