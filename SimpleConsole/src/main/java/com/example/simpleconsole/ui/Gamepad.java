package com.example.simpleconsole.ui;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

/**
 * Windows hook for key presses.
 */
class KeyDispatcher implements KeyEventDispatcher {
    private Gamepad gamepad;

    public KeyDispatcher(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        char code = keyEvent.getKeyChar();
        boolean isPressed = (keyEvent.getID() == KeyEvent.KEY_PRESSED);

        switch (code) {
            case KeyEvent.VK_UP:
                gamepad.dpad_up = isPressed;
                break;
            case KeyEvent.VK_DOWN:
                gamepad.dpad_down = isPressed;
                break;
            case KeyEvent.VK_LEFT:
                gamepad.dpad_left = isPressed;
                break;
            case KeyEvent.VK_RIGHT:
                gamepad.dpad_right = isPressed;
                break;
            case KeyEvent.VK_A:
                gamepad.a = isPressed;
                break;
            case KeyEvent.VK_B:
                gamepad.b = isPressed;
                break;
            case KeyEvent.VK_X:
                gamepad.x = isPressed;
                break;
            case KeyEvent.VK_Y:
                gamepad.y = isPressed;
                break;
            case KeyEvent.VK_ENTER:
                gamepad.start = isPressed;
                break;
        }

        if (isPressed) {
            gamepad.addChar(code);
        }

        return true;
    }
}

/**
 * This class implements a lightweight emulation of FTC Gamepad that can run on the PC.
 */
public class Gamepad {
    volatile String buffer = "";
    volatile String inputLine;

    volatile public boolean dpad_down;
    volatile public boolean dpad_up;
    volatile public boolean dpad_left;
    volatile public boolean dpad_right;
    volatile public boolean a;
    volatile public boolean b;
    volatile public boolean x;
    volatile public boolean y;
    volatile public boolean start;

    public Gamepad() {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyDispatcher(this));
    }

    public synchronized void addChar(int keyChar) {
        switch (keyChar) {
            case 65535:
                break;
            case 8:
                int length = buffer.length();
                if (length > 0) {
                    buffer = buffer.substring(0, length - 1);
                    System.out.write(keyChar);
                }
                break;
            case 10:
                inputLine = buffer;
                buffer = "";
                System.out.write(keyChar);
                break;
            default:
                buffer += (char) keyChar;
                System.out.write(keyChar);
        }
    }
}