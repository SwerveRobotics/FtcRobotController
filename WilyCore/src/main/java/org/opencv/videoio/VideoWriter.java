package org.opencv.videoio;


import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Size;

/**
 * Video writer class.
 *
 * The class provides C++ API for writing video files or image sequences.
 */
public class VideoWriter {

    protected VideoWriter(long addr) {  }

    public long getNativeObjAddr() { return 0; }

    // internal usage only
    public static VideoWriter __fromPtr__(long addr) { return new VideoWriter(addr); }

    public VideoWriter() { }
    public VideoWriter(String filename, int fourcc, double fps, Size frameSize, boolean isColor) {
    }
    public VideoWriter(String filename, int fourcc, double fps, Size frameSize) {
    }
    public VideoWriter(String filename, int apiPreference, int fourcc, double fps, Size frameSize, boolean isColor) {
    }
    public VideoWriter(String filename, int apiPreference, int fourcc, double fps, Size frameSize) {
    }
    public VideoWriter(String filename, int fourcc, double fps, Size frameSize, MatOfInt params) {
    }
    public VideoWriter(String filename, int apiPreference, int fourcc, double fps, Size frameSize, MatOfInt params) {
    }

    public boolean isOpened() {
        return true;
    }

    public void release() {
    }

    public void write(Mat image) {
    }


    /**
     * Sets a property in the VideoWriter.
     *
     *      @param propId Property identifier from cv::VideoWriterProperties (eg. cv::VIDEOWRITER_PROP_QUALITY)
     *      or one of REF: videoio_flags_others
     *
     *      @param value Value of the property.
     *      @return  {@code true} if the property is supported by the backend used by the VideoWriter instance.
     */
    public boolean set(int propId, double value) {
        return true;
    }
    public double get(int propId) {
        return 0;
    }
    public static int fourcc(char c1, char c2, char c3, char c4) {
        return 0;
    }
    public String getBackendName() {
        return "";
    }
    @Override
    protected void finalize() throws Throwable {
    }
}
