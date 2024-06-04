package com.example.simpleconsole.ui;

import java.io.InputStream;

/**
 * Similarly to the Gamepad, a lightweight "Scanner".
 */

public class Scanner {
    private boolean closed;

    public Scanner(InputStream input) {
        if (input != System.in) {
            System.err.println("You must use System.in for your scanner for this tutorial!");
        }
        closed = false;
    }

    public String nextLine() {
        Robotics.gamepad.inputting = true;
        if (closed) {
            System.err.println("Scanner is closed, so you can't use it!");
            return null;
        }
        while (Robotics.gamepad.inputLine == null);
        String line = Robotics.gamepad.inputLine;
        Robotics.gamepad.inputLine = null;
        Robotics.gamepad.inputting = false;
        return line;
    }

    public void close() {
        closed = true;
    }
}
