package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.wilyworks.simulator.WilyCore;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWGamepadState;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

// import uk.co.electronstudio.sdl2gdx.SDL2ControllerManager;

/**
 * Fake do-nothing gamepad classes while the gamepad library is offline.
 */
class SDL {
    public static final int SDL_CONTROLLER_BUTTON_INVALID = -1,
            SDL_CONTROLLER_BUTTON_A = 0,
            SDL_CONTROLLER_BUTTON_B = 1,
            SDL_CONTROLLER_BUTTON_X = 2,
            SDL_CONTROLLER_BUTTON_Y = 3,
            SDL_CONTROLLER_BUTTON_BACK = 4,
            SDL_CONTROLLER_BUTTON_GUIDE = 5,
            SDL_CONTROLLER_BUTTON_START = 6,
            SDL_CONTROLLER_BUTTON_LEFTSTICK = 7,
            SDL_CONTROLLER_BUTTON_RIGHTSTICK = 8,
            SDL_CONTROLLER_BUTTON_LEFTSHOULDER = 9,
            SDL_CONTROLLER_BUTTON_RIGHTSHOULDER = 10,
            SDL_CONTROLLER_BUTTON_DPAD_UP = 11,
            SDL_CONTROLLER_BUTTON_DPAD_DOWN = 12,
            SDL_CONTROLLER_BUTTON_DPAD_LEFT = 13,
            SDL_CONTROLLER_BUTTON_DPAD_RIGHT = 14,
            SDL_CONTROLLER_BUTTON_MAX = 15;
    public static final int SDL_CONTROLLER_AXIS_INVALID = -1,
            SDL_CONTROLLER_AXIS_LEFTX = 0,
            SDL_CONTROLLER_AXIS_LEFTY = 1,
            SDL_CONTROLLER_AXIS_RIGHTX = 2,
            SDL_CONTROLLER_AXIS_RIGHTY = 3,
            SDL_CONTROLLER_AXIS_TRIGGERLEFT = 4,
            SDL_CONTROLLER_AXIS_TRIGGERRIGHT = 5,
            SDL_CONTROLLER_AXIS_MAX = 6;
}

/**
 * Window manager hook for key presses.
 */
class KeyDispatcher implements KeyEventDispatcher {
    // Speed modifiers:
    final float FAST_SPEED = 1.0f;
    final float NORMAL_SPEED = 0.5f;
    final float SLOW_SPEED = 0.2f;

    // Consecutive clicks must be this many seconds to activate double-click:
    final double DOUBLE_CLICK_DURATION = 0.5;

    boolean altActivated; // True if Alt-mode was activating by double-tapping the Alt key
    double altPressTime; // Time when the Alt key was last pressed for a double-tap; 0 if none
    boolean altPressed; // True if the Alt key is currently being pressed
    boolean ctrlPressed; // True if the Control key is currently being pressed
    boolean shiftPressed; // True if the Shift key is currently being pressed

    boolean[] button = new boolean[SDL.SDL_CONTROLLER_BUTTON_MAX];
    float[] axis = new float[SDL.SDL_CONTROLLER_AXIS_MAX];
    float axisMultiplier; // When an axis is activated, use this for its speed

    int associatedGamepad = 1; // Which gamepad this input goes to, 1 or 2

    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        boolean pressed = (keyEvent.getID() != KeyEvent.KEY_RELEASED);
        float axisValue = (pressed) ? 1.0f : 0.0f;

        switch (code) {
            case KeyEvent.VK_ALT:
                altPressed = pressed;
                if (pressed) {
                    double time = WilyCore.wallClockTime();
                    if ((altPressTime == 0) || (time - altPressTime > DOUBLE_CLICK_DURATION)) {
                        altPressTime = WilyCore.wallClockTime();
                    } else {
                        // We detected an Alt double-click! Toggle the state:
                        altActivated = !altActivated;
                        altPressTime = 0;
                    }
                }
                break;

            case KeyEvent.VK_1: associatedGamepad = 1; break;
            case KeyEvent.VK_F1: associatedGamepad = 1; break;
            case KeyEvent.VK_2: associatedGamepad = 2; break;
            case KeyEvent.VK_F2: associatedGamepad = 2; break;
            case KeyEvent.VK_CONTROL: ctrlPressed = pressed; break;
            case KeyEvent.VK_SHIFT: shiftPressed = pressed; break;

            case KeyEvent.VK_A:
                if (altActivated || altPressed) {
                    button[SDL.SDL_CONTROLLER_BUTTON_A] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = 0;
                } else {
                    axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = -axisValue;
                    button[SDL.SDL_CONTROLLER_BUTTON_A] = false;
                }
                break;

            case KeyEvent.VK_D: axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = axisValue; break;
            case KeyEvent.VK_W: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = -axisValue; break;
            case KeyEvent.VK_S: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = axisValue; break;
            case KeyEvent.VK_COMMA: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT] = axisValue; break;
            case KeyEvent.VK_PERIOD: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT] = axisValue; break;

            case KeyEvent.VK_LEFT:
            case KeyEvent.VK_KP_LEFT:
                if (altActivated || altPressed) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = -axisValue;
                }
                break;
            case KeyEvent.VK_RIGHT:
            case KeyEvent.VK_KP_RIGHT:
                if (altActivated || altPressed) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = axisValue;
                }
                break;
            case KeyEvent.VK_UP:
            case KeyEvent.VK_KP_UP:
                if (altActivated || altPressed) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = -axisValue;
                }
                break;
            case KeyEvent.VK_DOWN:
            case KeyEvent.VK_KP_DOWN:
                if (altActivated || altPressed) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = axisValue;
                }
                break;

            case KeyEvent.VK_E:
            case KeyEvent.VK_SPACE:
            case KeyEvent.VK_ENTER:
                button[SDL.SDL_CONTROLLER_BUTTON_A] = pressed;
                break;
            case KeyEvent.VK_B: button[SDL.SDL_CONTROLLER_BUTTON_B] = pressed; break;
            case KeyEvent.VK_X: button[SDL.SDL_CONTROLLER_BUTTON_X] = pressed; break;
            case KeyEvent.VK_Y: button[SDL.SDL_CONTROLLER_BUTTON_Y] = pressed; break;
            case KeyEvent.VK_BACK_QUOTE: button[SDL.SDL_CONTROLLER_BUTTON_GUIDE] = pressed; break;
            case KeyEvent.VK_TAB: button[SDL.SDL_CONTROLLER_BUTTON_START] = pressed; break;
            case KeyEvent.VK_BACK_SPACE: button[SDL.SDL_CONTROLLER_BUTTON_BACK] = pressed; break;
            case KeyEvent.VK_SEMICOLON: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER] = pressed; break;
            case KeyEvent.VK_QUOTE: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER] = pressed; break;
            case KeyEvent.VK_BRACELEFT: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK] = pressed; break;
            case KeyEvent.VK_BRACERIGHT: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK] = pressed; break;

            // Let the default dispatcher handle everything else so that basics like Alt-F4
            // work to close the application:
            default: return false;
        }

        // Speed is 20% of max when control is pressed, 100% when shift is pressed, 40% otherwise:
        axisMultiplier = (ctrlPressed) ? SLOW_SPEED : ((shiftPressed) ? FAST_SPEED : NORMAL_SPEED);
        return true;
    }

    float getAxis(int gamepadId, int sdlAxis) {
        return (gamepadId == associatedGamepad) ? axis[sdlAxis] * axisMultiplier : 0;
    }

    boolean getButton(int gamepadId, int sdlButton) {
        return (gamepadId == associatedGamepad) ? button[sdlButton] : false;
    }
}

/**
 * Abstraction for true gamepad input on the PC via GLFW.
 */
class GamepadInput {
    int associatedGamepad = 1; // Gamepad that this input is directed to, 1 or 2
    GLFWGamepadState gamepadState; // Gamepad input object, null if no controller is plugged in

    GamepadInput() {
        if (!GLFW.glfwInit()) {
            System.out.println("Failed to initialize GLFW!");
            return;
        }
        if (GLFW.glfwJoystickIsGamepad(GLFW.GLFW_JOYSTICK_1)) {
            gamepadState = GLFWGamepadState.create();
        }
    }

    // FTC automatically implements a dead-zone but we have to do it manually on PC:
    private float deadZone(float value) {
        final double EPSILON = 0.05f;
        if (Math.abs(value) <= EPSILON)
            value = 0;
        return value;
    }

    boolean getButton(int gamepadId, int button) {
        if (gamepadId != associatedGamepad)
            return false;
        if (gamepadState == null)
            return false;

        final int[] mapping = {
            GLFW.GLFW_GAMEPAD_BUTTON_A,             // SDL_CONTROLLER_BUTTON_A = 0,
            GLFW.GLFW_GAMEPAD_BUTTON_B,             // SDL_CONTROLLER_BUTTON_B = 1,
            GLFW.GLFW_GAMEPAD_BUTTON_X,             // SDL_CONTROLLER_BUTTON_X = 2,
            GLFW.GLFW_GAMEPAD_BUTTON_Y,             // SDL_CONTROLLER_BUTTON_Y = 3,
            GLFW.GLFW_GAMEPAD_BUTTON_BACK,          // SDL_CONTROLLER_BUTTON_BACK = 4,
            GLFW.GLFW_GAMEPAD_BUTTON_GUIDE,         // SDL_CONTROLLER_BUTTON_GUIDE = 5,
            GLFW.GLFW_GAMEPAD_BUTTON_START,         // SDL_CONTROLLER_BUTTON_START = 6,
            GLFW.GLFW_GAMEPAD_BUTTON_LEFT_THUMB,    // SDL_CONTROLLER_BUTTON_LEFTSTICK = 7,
            GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_THUMB,   // SDL_CONTROLLER_BUTTON_RIGHTSTICK = 8,
            GLFW.GLFW_GAMEPAD_BUTTON_LEFT_BUMPER,   // SDL_CONTROLLER_BUTTON_LEFTSHOULDER = 9,
            GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER,  // SDL_CONTROLLER_BUTTON_RIGHTSHOULDER = 10,
            GLFW.GLFW_GAMEPAD_BUTTON_DPAD_UP,       // SDL_CONTROLLER_BUTTON_DPAD_UP = 11,
            GLFW.GLFW_GAMEPAD_BUTTON_DPAD_DOWN,     // SDL_CONTROLLER_BUTTON_DPAD_DOWN = 12,
            GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT,     // SDL_CONTROLLER_BUTTON_DPAD_LEFT = 13,
            GLFW.GLFW_GAMEPAD_BUTTON_DPAD_RIGHT     // SDL_CONTROLLER_BUTTON_DPAD_RIGHT = 14
        };
        return gamepadState.buttons(mapping[button]) != 0;
    }

    float getAxis(int gamepadId, int axis) {
        if (gamepadId != associatedGamepad)
            return 0;
        if (gamepadState == null)
            return 0;
        float result = gamepadState.axes(axis);

        // FTC returns trigger results in the range [0, 1] but LWJGL returns [-1, 1]:
        if ((axis == SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT) ||
            (axis == SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT))
            result = (result + 1) / 2;

        return deadZone(result);
    }

    void poll() {
        if (gamepadState != null) {
            GLFW.glfwGetGamepadState(GLFW.GLFW_JOYSTICK_1, gamepadState);

            // Handle gamepad changes:
            if (gamepadState.buttons(GLFW.GLFW_GAMEPAD_BUTTON_START) != 0) {
                if (gamepadState.buttons(GLFW.GLFW_GAMEPAD_BUTTON_A) != 0)
                    associatedGamepad = 1;
                else if (gamepadState.buttons(GLFW.GLFW_GAMEPAD_BUTTON_B) != 0)
                    associatedGamepad = 2;
            }
        }
    }
}

/**
 * This class is tasked with regularly updating the state of the Gamepad objects.
 */
public class InputManager extends Thread {
    Gamepad gamepad1;
    Gamepad gamepad2;
    GamepadInput gamepadInput = new GamepadInput();
    KeyDispatcher keyDispatcher = new KeyDispatcher();

    // Wrap the two gamepad objects:
    public InputManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(keyDispatcher);

        // Start our thread:
        setName("Wily gamepad thread");
        start();
    }

    // The input worker thread runs this loop forever:
    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
        while (true) {
            // Update the gamepad state every 10 milliseconds:
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            update(gamepad1, 1);
            update(gamepad2, 2);
        }
    }

    // Get button state from either the controller or the keyboard:
    boolean getButton(int gamepadId, int sdlButton) {
        return gamepadInput.getButton(gamepadId, sdlButton)
            || keyDispatcher.getButton(gamepadId, sdlButton);
    }

    // Get axis state from either the controller or the keyboard, with the latter winning ties:
    float getAxis(int gamepadId, int sdlAxis) {
        float result = keyDispatcher.getAxis(gamepadId, sdlAxis);
        if (result == 0)
            result = gamepadInput.getAxis(gamepadId, sdlAxis);
        return result;
    }

    // Poll the attached game controller to update the button and axis states
    void update(Gamepad gamepad, int gamepadId) {
        gamepadInput.poll();

        // Now set the state:
        gamepad.a = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_A);
        gamepad.b = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_B);
        gamepad.x = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_X);
        gamepad.y = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_Y);
        gamepad.back = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_BACK);
        gamepad.guide = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_GUIDE);
        gamepad.start = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_START);
        gamepad.dpad_up = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_DPAD_UP);
        gamepad.dpad_down = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        gamepad.dpad_left = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        gamepad.dpad_right = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        gamepad.left_bumper = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
        gamepad.right_bumper = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        gamepad.left_stick_button = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK);
        gamepad.right_stick_button = getButton(gamepadId, SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK);

        gamepad.left_stick_x = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_LEFTX);
        gamepad.left_stick_y = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_LEFTY);
        gamepad.right_stick_x = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_RIGHTX);
        gamepad.right_stick_y = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_RIGHTY);
        gamepad.left_trigger = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        gamepad.right_trigger = getAxis(gamepadId, SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

        gamepad.updateButtonAliases();
    }

    // Get a string describing which gamepads the inputs correspond to.
    public String getMappings() {
        String result = "Keyboard: gamepad" + keyDispatcher.associatedGamepad;
        if (gamepadInput.gamepadState != null) {
            result += ", Controller: gamepad" + gamepadInput.associatedGamepad;
        }
        return result;
    }
}