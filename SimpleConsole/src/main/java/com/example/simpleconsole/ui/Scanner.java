package com.example.simpleconsole.ui;

import java.io.InputStream;

/**
 * Similarly to the Gamepad, a lightweight "Scanner".
 */

public class Scanner {
    public Scanner(InputStream input) {
    }

    public String nextLine() {
        while (Robotics.gamepad.inputLine == null);
        String line = Robotics.gamepad.inputLine;
        Robotics.gamepad.inputLine = null;
        return line;
    }
}
