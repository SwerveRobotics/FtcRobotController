package android.app;

import android.view.View;

public class Activity {
    public <T extends View> T findViewById(int id) {
        return (T) new View();
    }
}
