package android.content;

import android.app.Activity;

import java.io.File;

public class Context extends Activity {
    public Resources getResources() {
        return new Resources();
    }
    public String getPackageName() {
        return "WilyWorks";
    }
    public static final int MODE_PRIVATE = 0;
    public File getDir(String var1, int var2) { return null; }
}
