package android.graphics;

import android.annotation.NonNull;

import java.io.OutputStream;
import java.nio.Buffer;

public class Bitmap {
    @NonNull
    public static final int DENSITY_NONE = 0;

    Bitmap() {
        throw new RuntimeException("Stub!");
    }

    public int getDensity() {
        throw new RuntimeException("Stub!");
    }

    public void setDensity(int density) {
        throw new RuntimeException("Stub!");
    }

    public void reconfigure(int width, int height, Config config) {
        throw new RuntimeException("Stub!");
    }

    public void setWidth(int width) {
        throw new RuntimeException("Stub!");
    }

    public void setHeight(int height) {
        throw new RuntimeException("Stub!");
    }

    public void setConfig(Config config) {
        throw new RuntimeException("Stub!");
    }

    public void recycle() {
        throw new RuntimeException("Stub!");
    }

    public boolean isRecycled() {
        throw new RuntimeException("Stub!");
    }

    public int getGenerationId() {
        throw new RuntimeException("Stub!");
    }

    public void copyPixelsToBuffer(Buffer dst) {
        throw new RuntimeException("Stub!");
    }

    public void copyPixelsFromBuffer(Buffer src) {
        throw new RuntimeException("Stub!");
    }

    public Bitmap copy(Config config, boolean isMutable) {
        throw new RuntimeException("Stub!");
    }


    public static Bitmap createScaledBitmap(@NonNull Bitmap src, int dstWidth, int dstHeight, boolean filter) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(@NonNull Bitmap src) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(@NonNull Bitmap source, int x, int y, int width, int height) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(int width, int height, @NonNull Config config) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(int width, int height, @NonNull Config config, boolean hasAlpha) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(@NonNull int[] colors, int offset, int stride, int width, int height, @NonNull Config config) {
        throw new RuntimeException("Stub!");
    }

    public static Bitmap createBitmap(@NonNull int[] colors, int width, int height, Config config) {
        throw new RuntimeException("Stub!");
    }

    public byte[] getNinePatchChunk() {
        throw new RuntimeException("Stub!");
    }

    public boolean compress(CompressFormat format, int quality, OutputStream stream) {
        throw new RuntimeException("Stub!");
    }

    public boolean isMutable() {
        throw new RuntimeException("Stub!");
    }

    public boolean isPremultiplied() {
        throw new RuntimeException("Stub!");
    }

    public void setPremultiplied(boolean premultiplied) {
        throw new RuntimeException("Stub!");
    }

    public int getWidth() {
        throw new RuntimeException("Stub!");
    }

    public int getHeight() {
        throw new RuntimeException("Stub!");
    }

    public int getScaledWidth(Canvas canvas) {
        throw new RuntimeException("Stub!");
    }

    public int getScaledHeight(Canvas canvas) {
        throw new RuntimeException("Stub!");
    }

    public int getScaledWidth(int targetDensity) {
        throw new RuntimeException("Stub!");
    }

    public int getScaledHeight(int targetDensity) {
        throw new RuntimeException("Stub!");
    }

    public int getRowBytes() {
        throw new RuntimeException("Stub!");
    }

    public int getByteCount() {
        throw new RuntimeException("Stub!");
    }

    public int getAllocationByteCount() {
        throw new RuntimeException("Stub!");
    }

    public Config getConfig() {
        throw new RuntimeException("Stub!");
    }

    public boolean hasAlpha() {
        throw new RuntimeException("Stub!");
    }

    public void setHasAlpha(boolean hasAlpha) {
        throw new RuntimeException("Stub!");
    }

    public boolean hasMipMap() {
        throw new RuntimeException("Stub!");
    }

    public void setHasMipMap(boolean hasMipMap) {
        throw new RuntimeException("Stub!");
    }

    public void eraseColor(int c) {
        throw new RuntimeException("Stub!");
    }

    public void eraseColor(long color) {
        throw new RuntimeException("Stub!");
    }

    public int getPixel(int x, int y) {
        throw new RuntimeException("Stub!");
    }

    @NonNull
    public Color getColor(int x, int y) {
        throw new RuntimeException("Stub!");
    }

    public void getPixels(int[] pixels, int offset, int stride, int x, int y, int width, int height) {
        throw new RuntimeException("Stub!");
    }

    public void setPixel(int x, int y, int color) {
        throw new RuntimeException("Stub!");
    }

    public void setPixels(int[] pixels, int offset, int stride, int x, int y, int width, int height) {
        throw new RuntimeException("Stub!");
    }

    public int describeContents() {
        throw new RuntimeException("Stub!");
    }

    public Bitmap extractAlpha() {
        throw new RuntimeException("Stub!");
    }

    public Bitmap extractAlpha(Paint paint, int[] offsetXY) {
        throw new RuntimeException("Stub!");
    }

    public boolean sameAs(Bitmap other) {
        throw new RuntimeException("Stub!");
    }

    public void prepareToDraw() {
        throw new RuntimeException("Stub!");
    }

    public static enum CompressFormat {
        JPEG,
        PNG,
        /** @deprecated */
        @Deprecated
        WEBP,
        WEBP_LOSSY,
        WEBP_LOSSLESS;
    }

    public static enum Config {
        ALPHA_8,
        RGB_565,
        /** @deprecated */
        @Deprecated
        ARGB_4444,
        ARGB_8888,
        RGBA_F16,
        HARDWARE;
    }

}
