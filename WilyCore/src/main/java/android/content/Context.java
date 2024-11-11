package android.content;

import android.app.Activity;

public class Context extends Activity {
    public Resources getResources() {
        return new Resources();
    }
    public String getPackageName() {
        return "WilyWorks";
    }
}
