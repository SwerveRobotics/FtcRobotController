package android.graphics;

import android.annotation.NonNull;
import android.annotation.Nullable;

public class Canvas {
    public Canvas(@NonNull Bitmap bitmap) {
        throw new RuntimeException("Wily Works Stub!");
    }

    public static final int ALL_SAVE_FLAG = 31;

    public Canvas() {
        throw new RuntimeException("Stub!");
    }

    public boolean isHardwareAccelerated() {
        throw new RuntimeException("Stub!");
    }

    public void setBitmap(@Nullable Bitmap bitmap) {
        throw new RuntimeException("Stub!");
    }

    public void enableZ() {
        throw new RuntimeException("Stub!");
    }

    public void disableZ() {
        throw new RuntimeException("Stub!");
    }

    public boolean isOpaque() {
        throw new RuntimeException("Stub!");
    }

    public int getWidth() {
        throw new RuntimeException("Stub!");
    }

    public int getHeight() {
        throw new RuntimeException("Stub!");
    }

    public int getDensity() {
        throw new RuntimeException("Stub!");
    }

    public void setDensity(int density) {
        throw new RuntimeException("Stub!");
    }

    public int getMaximumBitmapWidth() {
        throw new RuntimeException("Stub!");
    }

    public int getMaximumBitmapHeight() {
        throw new RuntimeException("Stub!");
    }

    public int save() {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public int saveLayer(float left, float top, float right, float bottom, @Nullable Paint paint, int saveFlags) {
        throw new RuntimeException("Stub!");
    }

    public int saveLayer(float left, float top, float right, float bottom, @Nullable Paint paint) {
        throw new RuntimeException("Stub!");
    }
    /** @deprecated */
    @Deprecated
    public int saveLayerAlpha(float left, float top, float right, float bottom, int alpha, int saveFlags) {
        throw new RuntimeException("Stub!");
    }

    public int saveLayerAlpha(float left, float top, float right, float bottom, int alpha) {
        throw new RuntimeException("Stub!");
    }

    public void restore() {
        throw new RuntimeException("Stub!");
    }

    public int getSaveCount() {
        throw new RuntimeException("Stub!");
    }

    public void restoreToCount(int saveCount) {
        throw new RuntimeException("Stub!");
    }

    public void translate(float dx, float dy) {
        throw new RuntimeException("Stub!");
    }

    public void scale(float sx, float sy) {
        throw new RuntimeException("Stub!");
    }

    public final void scale(float sx, float sy, float px, float py) {
        throw new RuntimeException("Stub!");
    }

    public void rotate(float degrees) {
        throw new RuntimeException("Stub!");
    }

    public final void rotate(float degrees, float px, float py) {
        throw new RuntimeException("Stub!");
    }

    public void skew(float sx, float sy) {
        throw new RuntimeException("Stub!");
    }

    public boolean clipRect(float left, float top, float right, float bottom) {
        throw new RuntimeException("Stub!");
    }

    public boolean clipOutRect(float left, float top, float right, float bottom) {
        throw new RuntimeException("Stub!");
    }

    public boolean clipRect(int left, int top, int right, int bottom) {
        throw new RuntimeException("Stub!");
    }

    public boolean clipOutRect(int left, int top, int right, int bottom) {
        throw new RuntimeException("Stub!");
    }


    /** @deprecated */
    @Deprecated
    public void drawBitmap(@NonNull int[] colors, int offset, int stride, float x, float y, int width, int height, boolean hasAlpha, @Nullable Paint paint) {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public void drawBitmap(@NonNull int[] colors, int offset, int stride, int x, int y, int width, int height, boolean hasAlpha, @Nullable Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawBitmapMesh(@NonNull Bitmap bitmap, int meshWidth, int meshHeight, @NonNull float[] verts, int vertOffset, @Nullable int[] colors, int colorOffset, @Nullable Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawCircle(float cx, float cy, float radius, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawColor(int color) {
        throw new RuntimeException("Stub!");
    }

    public void drawColor(long color) {
        throw new RuntimeException("Stub!");
    }

    public void drawLine(float startX, float startY, float stopX, float stopY, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawLines(@NonNull float[] pts, int offset, int count, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawLines(@NonNull float[] pts, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }
    public void drawOval(float left, float top, float right, float bottom, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawPaint(@NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawPoint(float x, float y, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawPoints(float[] pts, int offset, int count, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawPoints(@NonNull float[] pts, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public void drawPosText(@NonNull char[] text, int index, int count, @NonNull float[] pos, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    /** @deprecated */
    @Deprecated
    public void drawPosText(@NonNull String text, @NonNull float[] pos, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawRect(float left, float top, float right, float bottom, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawRGB(int r, int g, int b) {
        throw new RuntimeException("Stub!");
    }
    public void drawRoundRect(float left, float top, float right, float bottom, float rx, float ry, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawText(@NonNull char[] text, int index, int count, float x, float y, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawText(@NonNull String text, float x, float y, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawText(@NonNull String text, int start, int end, float x, float y, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawText(@NonNull CharSequence text, int start, int end, float x, float y, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawTextRun(@NonNull char[] text, int index, int count, int contextIndex, int contextCount, float x, float y, boolean isRtl, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

    public void drawTextRun(@NonNull CharSequence text, int start, int end, int contextStart, int contextEnd, float x, float y, boolean isRtl, @NonNull Paint paint) {
        throw new RuntimeException("Stub!");
    }

}
