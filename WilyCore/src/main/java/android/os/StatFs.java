package android.os;

import java.io.File;

public class StatFs {
    final File root;
    public StatFs(String path) {
        root = new File(path);
    }
    public long getAvailableBytes() {
        return root.getTotalSpace();
    }
}
