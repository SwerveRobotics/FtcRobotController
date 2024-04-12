package com.wilyworks.simulator.framework;

import com.badlogic.gdx.controllers.Controller;
import com.wilyworks.simulator.WilyCore;

import org.libsdl.SDL;

import uk.co.electronstudio.sdl2gdx.SDL2ControllerManager;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

/**
 * Windows hook for key presses.
 */
class KeyDispatcher implements KeyEventDispatcher {
    // Consecutive clicks must be this many seconds to activate double-click:
    final double DOUBLE_CLICK_DURATION = 0.5;

    private boolean altActivated; // True if Alt-mode was activating by double-tapping the Alt key
    private double altPressTime; // Time when the Alt key was last pressed for a double-tap; 0 if none
    private boolean ctrlPressed; // True if the Control key is currently being pressed
    private boolean shiftPressed; // True if the Shift key is currently being pressed

    public boolean[] button = new boolean[SDL.SDL_CONTROLLER_BUTTON_MAX];
    public float[] axis = new float[SDL.SDL_CONTROLLER_AXIS_MAX];
    public float axisMultiplier; // When an axis is activated, use this for its speed

    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        boolean pressed = (keyEvent.getID() != KeyEvent.KEY_RELEASED);
        float axisValue = (pressed) ? 1.0f : 0.0f;

        switch (code) {
            case KeyEvent.VK_ALT:
                if (pressed) {
                    double time = WilyCore.time();
                    if ((altPressTime == 0) || (time - altPressTime > DOUBLE_CLICK_DURATION)) {
                        altPressTime = WilyCore.time();
                    } else {
                        // We detected an Alt double-click! Toggle the state:
                        altActivated = !altActivated;
                        altPressTime = 0;
                    }
                }
                break;

            case KeyEvent.VK_CONTROL: ctrlPressed = pressed; break;
            case KeyEvent.VK_SHIFT: shiftPressed = pressed; break;

            case KeyEvent.VK_A: axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = -axisValue; break;
            case KeyEvent.VK_D: axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = axisValue; break;
            case KeyEvent.VK_W: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = -axisValue; break;
            case KeyEvent.VK_S: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = axisValue; break;
            case KeyEvent.VK_COMMA: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT] = axisValue; break;
            case KeyEvent.VK_PERIOD: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT] = axisValue; break;

            case KeyEvent.VK_LEFT:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = -axisValue;
                }
                break;
            case KeyEvent.VK_RIGHT:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = axisValue;
                }
                break;
            case KeyEvent.VK_UP:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = -axisValue;
                }
                break;
            case KeyEvent.VK_DOWN:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = axisValue;
                }
                break;

            case KeyEvent.VK_E:
            case KeyEvent.VK_SPACE: button[SDL.SDL_CONTROLLER_BUTTON_A] = pressed; break;
            case KeyEvent.VK_B: button[SDL.SDL_CONTROLLER_BUTTON_B] = pressed; break;
            case KeyEvent.VK_X: button[SDL.SDL_CONTROLLER_BUTTON_X] = pressed; break;
            case KeyEvent.VK_Y: button[SDL.SDL_CONTROLLER_BUTTON_Y] = pressed; break;
            case KeyEvent.VK_DEAD_TILDE: button[SDL.SDL_CONTROLLER_BUTTON_GUIDE] = pressed; break;
            case KeyEvent.VK_TAB: button[SDL.SDL_CONTROLLER_BUTTON_START] = pressed; break;
            case KeyEvent.VK_BACK_SPACE: button[SDL.SDL_CONTROLLER_BUTTON_BACK] = pressed; break;
            case KeyEvent.VK_SEMICOLON: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER] = pressed; break;
            case KeyEvent.VK_QUOTE: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER] = pressed; break;
            case KeyEvent.VK_BRACELEFT: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK] = pressed; break;
            case KeyEvent.VK_BRACERIGHT: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK] = pressed; break;
        }

        // Speed is 20% of max when control is pressed, 100% when shift is pressed, 40% otherwise:
        axisMultiplier = (ctrlPressed) ? 0.20f : ((shiftPressed) ? 1.0f : 0.4f);
        return true;
    }
}

/**
 * Wily Works Gamepad implementation that takes input either from a connected gamepad or
 * from the keyboard.
 */
public class WilyGamepad {

    public volatile float left_stick_x = 0f;
    public volatile float left_stick_y = 0f;
    public volatile float right_stick_x = 0f;
    public volatile float right_stick_y = 0f;
    public volatile boolean dpad_up = false;
    public volatile boolean dpad_down = false;
    public volatile boolean dpad_left = false;
    public volatile boolean dpad_right = false;
    public volatile boolean a = false;
    public volatile boolean b = false;
    public volatile boolean x = false;
    public volatile boolean y = false;
    public volatile boolean guide = false;
    public volatile boolean start = false;
    public volatile boolean back = false;
    public volatile boolean left_bumper = false;
    public volatile boolean right_bumper = false;
    public volatile boolean left_stick_button = false;
    public volatile boolean right_stick_button = false;
    public volatile float left_trigger = 0f;
    public volatile float right_trigger = 0f;
    public volatile boolean circle = false;
    public volatile boolean cross = false;
    public volatile boolean triangle = false;
    public volatile boolean square = false;
    public volatile boolean share = false;
    public volatile boolean options = false;
    public volatile boolean ps = false;

    SDL2ControllerManager controllerManager;
    KeyDispatcher keyDispatcher;
    Controller controller;

    public WilyGamepad() {
        controllerManager = new SDL2ControllerManager();
        keyDispatcher = new KeyDispatcher();
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(keyDispatcher);
    }

    void updateButtonAliases(){
        // There is no assignment for touchpad because there is no equivalent on XBOX controllers.
        circle = b;
        cross = a;
        triangle = y;
        square = x;
        share = back;
        options = start;
        ps = guide;
    }

    // FTC automatically implements a dead-zone but we have to do it manually on PC:
    private float deadZone(float value) {
        final double EPSILON = 0.05f;
        if (Math.abs(value) <= EPSILON)
            value = 0;
        return value;
    }

    // Get button state from either the controller or the keyboard:
    boolean getButton(int sdlButton) {
        if (controller != null)
            return controller.getButton(sdlButton) || keyDispatcher.button[sdlButton];
        else
            return keyDispatcher.button[sdlButton];
    }

    // Get axis state from either the controller or the keyboard, with the latter winning ties:
    float getAxis(int sdlAxis) {
        if (keyDispatcher.axis[sdlAxis] != 0)
            return keyDispatcher.axis[sdlAxis] * keyDispatcher.axisMultiplier;
        else if (controller != null)
            return deadZone(controller.getAxis(sdlAxis));
        else
            return 0;
    }

    // Poll the attached game controller to update the button and axis states
    public void update() {
        // Set some state so 'getButton' and 'getAxis' work:
        int count = controllerManager.getControllers().size;
        controller = (count == 0) ? null : controllerManager.getControllers().get(0);

        a = getButton(SDL.SDL_CONTROLLER_BUTTON_A);
        b = getButton(SDL.SDL_CONTROLLER_BUTTON_B);
        x = getButton(SDL.SDL_CONTROLLER_BUTTON_X);
        y = getButton(SDL.SDL_CONTROLLER_BUTTON_Y);
        back = getButton(SDL.SDL_CONTROLLER_BUTTON_BACK);
        guide = getButton(SDL.SDL_CONTROLLER_BUTTON_GUIDE);
        start = getButton(SDL.SDL_CONTROLLER_BUTTON_START);
        dpad_up = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_UP);
        dpad_down = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        dpad_left = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        dpad_right = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        left_bumper = getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
        right_bumper = getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        left_stick_button = getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK);
        right_stick_button = getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK);

        left_stick_x = getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTX);
        left_stick_y = getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTY);
        right_stick_x = getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTX);
        right_stick_y = getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTY);
        left_trigger = getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        right_trigger = getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

        updateButtonAliases();
    }
}
