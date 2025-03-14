package android.graphics;

public class Color {
    public static final int BLACK = -16777216;
    public static final int BLUE = -16776961;
    public static final int CYAN = -16711681;
    public static final int DKGRAY = -12303292;
    public static final int GRAY = -7829368;
    public static final int GREEN = -16711936;
    public static final int LTGRAY = -3355444;
    public static final int MAGENTA = -65281;
    public static final int RED = -65536;
    public static final int TRANSPARENT = 0;
    public static final int WHITE = -1;
    public static final int YELLOW = -256;

    public static int argb(int alpha, int red, int green, int blue) {
        return (alpha << 24) | (red << 16) | (green << 8) | blue;
    }

    public static void colorToHSV(int color, float[] hsv) {
        float rNorm = ((color >> 16) & 0xff) / 255.0f;
        float gNorm = ((color >> 8) & 0xff) / 255.0f;
        float bNorm = ((color) & 0xff) / 255.0f;

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;

        float h = 0, s, v = max;

        if (delta != 0) {
            if (max == rNorm) {
                h = ((gNorm - bNorm) / delta) % 6;
            } else if (max == gNorm) {
                h = ((bNorm - rNorm) / delta) + 2;
            } else {
                h = ((rNorm - gNorm) / delta) + 4;
            }

            h *= 60;
            if (h < 0) {
                h += 360;
            }
        }

        s = (max == 0) ? 0 : delta / max;
        hsv[0] = h;
        hsv[1] = s;
        hsv[2] = v;
    }

    public static int HSVToColor(float[] hsv) {
        float h = hsv[0], s = hsv[1], v = hsv[2];
        float r = 0, g = 0, b = 0;

        int i = (int) (h / 60) % 6;
        float f = (h / 60) - i;
        float p = v * (1 - s);
        float q = v * (1 - f * s);
        float t = v * (1 - (1 - f) * s);

        switch (i) {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = v;
                b = p;
                break;
            case 2:
                r = p;
                g = v;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = v;
                break;
            case 4:
                r = t;
                g = p;
                b = v;
                break;
            case 5:
                r = v;
                g = p;
                b = q;
                break;
        }

        int red = Math.round(r * 255);
        int green = Math.round(g * 255);
        int blue = Math.round(b * 255);

        return Color.argb(0xff, red, green, blue);
    }

    public static int parseColor(String colorString) {
        if ((colorString.length() < 7) || (colorString.charAt(0) != '#'))
            return 0;
        int value = (int) Long.parseLong(colorString.substring(1), 16);
        if (colorString.length() == 7)
            value |= 0xff000000;
        return value;
    }
}
