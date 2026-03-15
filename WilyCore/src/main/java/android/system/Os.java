package android.system;

import java.io.FileDescriptor;

public final class Os {
    public static FileDescriptor open(String path, int flags, int mode) throws ErrnoException {
        return null;
    }
    public static void close(FileDescriptor fd) throws ErrnoException {
    }
    public static void fsync(FileDescriptor fd) throws ErrnoException {
    }
}
