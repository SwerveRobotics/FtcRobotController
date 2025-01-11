package org.firstinspires.ftc.team417.color;

public class ColorConverter {
    // Reference values for D65 standard illuminant
    private static final double REF_X = 95.047;
    private static final double REF_Y = 100.000;
    private static final double REF_Z = 108.883;

    // Gamma correction threshold
    private static final double GAMMA_THRESHOLD = 0.04045;
    private static final double XYZ_THRESHOLD = 0.008856;

    /**
     * Converts normalized RGB (0-1) to LAB color space
     */
    public static double[] rgbToLab(double r, double g, double b) {
        // First convert RGB to XYZ
        double[] xyz = rgbToXyz(r, g, b);

        // Then convert XYZ to LAB
        return xyzToLab(xyz[0], xyz[1], xyz[2]);
    }

    /**
     * Converts normalized RGB (0-1) to XYZ color space
     */
    private static double[] rgbToXyz(double r, double g, double b) {
        // Convert RGB to linear RGB (remove gamma correction)
        r = (r > GAMMA_THRESHOLD) ? Math.pow((r + 0.055) / 1.055, 2.4) : r / 12.92;
        g = (g > GAMMA_THRESHOLD) ? Math.pow((g + 0.055) / 1.055, 2.4) : g / 12.92;
        b = (b > GAMMA_THRESHOLD) ? Math.pow((b + 0.055) / 1.055, 2.4) : b / 12.92;

        // Scale RGB values
        r *= 100;
        g *= 100;
        b *= 100;

        // Convert to XYZ using D65 matrix (more precise values)
        double x = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
        double y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
        double z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;

        return new double[]{x, y, z};
    }

    /**
     * Converts XYZ color space to LAB
     */
    private static double[] xyzToLab(double x, double y, double z) {
        // Normalize XYZ values with reference white point
        x = x / REF_X;
        y = y / REF_Y;
        z = z / REF_Z;

        // Apply nonlinear transformation
        x = transformXyz(x);
        y = transformXyz(y);
        z = transformXyz(z);

        // Calculate LAB values
        double l = Math.max(0, 116 * y - 16);
        double a = 500 * (x - y);
        double b = 200 * (y - z);

        return new double[]{l, a, b};
    }

    /**
     * Helper function for XYZ to LAB conversion
     */
    private static double transformXyz(double t) {
        if (t > XYZ_THRESHOLD) {
            return Math.pow(t, 1.0/3.0);
        } else {
            return (7.787 * t) + (16.0/116.0);
        }
    }

    /**
     * Helper method to check if two LAB values are approximately equal
     */
    private static boolean isLabEqual(double[] lab1, double[] lab2, double tolerance) {
        return Math.abs(lab1[0] - lab2[0]) < tolerance &&
                Math.abs(lab1[1] - lab2[1]) < tolerance &&
                Math.abs(lab1[2] - lab2[2]) < tolerance;
    }

    public static void main(String[] args) {
        // Test case structure: RGB input -> expected LAB output (approximately)
        Object[][] testCases = {
                // Primary colors
                {new double[]{1.0, 0.0, 0.0}, new double[]{53.23, 80.11, 67.22}, "Pure Red"},
                {new double[]{0.0, 1.0, 0.0}, new double[]{87.74, -86.18, 83.18}, "Pure Green"},
                {new double[]{0.0, 0.0, 1.0}, new double[]{32.30, 79.20, -107.86}, "Pure Blue"},

                // Secondary colors
                {new double[]{1.0, 1.0, 0.0}, new double[]{97.14, -21.56, 94.48}, "Yellow"},
                {new double[]{1.0, 0.0, 1.0}, new double[]{60.32, 98.25, -60.84}, "Magenta"},
                {new double[]{0.0, 1.0, 1.0}, new double[]{91.12, -48.08, -14.14}, "Cyan"},

                // Grayscale
                {new double[]{0.0, 0.0, 0.0}, new double[]{0.00, 0.00, 0.00}, "Black"},
                {new double[]{1.0, 1.0, 1.0}, new double[]{100.00, 0.01, -0.01}, "White"},
                {new double[]{0.5, 0.5, 0.5}, new double[]{53.39, 0.00, -0.01}, "Mid Gray"},

                // Pastels
                {new double[]{1.0, 0.8, 0.8}, new double[]{86.41, 18.01, 6.86}, "Light Pink"},
                {new double[]{0.8, 1.0, 0.8}, new double[]{95.46, -25.57, 19.17}, "Light Green"},

                // Edge cases
                {new double[]{0.04045, 0.04045, 0.04045}, new double[]{2.83, 0.00, 0.00}, "Gamma correction threshold"},
                {new double[]{0.0088, 0.0088, 0.0088}, new double[]{0.62, 0.00, 0.00}, "XYZ transformation threshold"}
        };

        // Run all test cases
        int passed = 0;
        double tolerance = 0.1; // Reduced tolerance for more precise testing

        System.out.println("Running ColorConverter test cases...\n");

        for (Object[] test : testCases) {
            double[] rgb = (double[]) test[0];
            double[] expectedLab = (double[]) test[1];
            String testName = (String) test[2];

            double[] resultLab = rgbToLab(rgb[0], rgb[1], rgb[2]);

            boolean passed_test = isLabEqual(resultLab, expectedLab, tolerance);
            passed += passed_test ? 1 : 0;

            System.out.printf("Test: %s%n", testName);
            System.out.printf("Input RGB: (%.3f, %.3f, %.3f)%n", rgb[0], rgb[1], rgb[2]);
            System.out.printf("Expected LAB: (%.2f, %.2f, %.2f)%n", expectedLab[0], expectedLab[1], expectedLab[2]);
            System.out.printf("Got LAB: (%.2f, %.2f, %.2f)%n", resultLab[0], resultLab[1], resultLab[2]);
            System.out.printf("Status: %s%n%n", passed_test ? "PASSED" : "FAILED");
        }

        System.out.printf("Test summary: %d/%d tests passed%n", passed, testCases.length);
    }
}