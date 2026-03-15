package android.system;

import android.annotation.NonNull;

import java.io.IOException;
import java.net.SocketException;

public final class ErrnoException extends Exception {
    public final int errno = 0;

    public ErrnoException(String functionName, int errno) {
        throw new RuntimeException("Stub!");
    }

    public ErrnoException(String functionName, int errno, Throwable cause) {
        throw new RuntimeException("Stub!");
    }

    public String getMessage() {
        throw new RuntimeException("Stub!");
    }

    @NonNull
    public IOException rethrowAsIOException() throws IOException {
        throw new RuntimeException("Stub!");
    }

    @NonNull
    public SocketException rethrowAsSocketException() throws SocketException {
        throw new RuntimeException("Stub!");
    }
}
