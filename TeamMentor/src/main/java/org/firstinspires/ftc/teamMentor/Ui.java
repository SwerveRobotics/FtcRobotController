package org.firstinspires.ftc.teamMentor;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to handle user interface screen output and gamepad input.
 */
class Ui {
    final Telemetry telemetry;
    final Gamepad[] gamepads; // [0] is unused, [1] is gamepad1, [2] is gamepad2

    int defaultGamepad = 1; // 1 or 2
    StringBuilder stringBuilder = new StringBuilder(); // For incremental telemetry output

    static final String HIGHLIGHT_COLOR = "#9090c0"; // Contrasting highlight color
    static final String CRITICAL_ICON = "<big>\u274c</big> ";
    static final String WARNING_ICON = "<big>\u26a0\ufe0f</big> ";
    static final String QUESTION_ICON = "<big>\u2753</big> ";
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed

    static String buttonString(String button) {
        return String.format("<span style='background:#a0a0a0'>%s</span>", button);
    }
    static final String A = "\ud83c\udd50"; // Symbol for the gamepad A button
    static final String B = "\ud83c\udd51"; // Symbol for the gamepad B button
    static final String X = "\ud83c\udd67"; // Symbol for the gamepad X button
    static final String Y = "\ud83c\udd68"; // Symbol for the gamepad Y button
    static final String TRIGGERS = buttonString("triggers");
    static final String LEFT_TRIGGER = buttonString("LT");
    static final String RIGHT_TRIGGER = buttonString("RT");
    static final String LEFT_BUMPER = buttonString("LB");
    static final String RIGHT_BUMPER = buttonString("RB");
    static final String DPAD_LEFT_RIGHT = buttonString("&nbsp;DPAD \u2194&nbsp;");
    static final String DPAD_UP_DOWN = buttonString("&nbsp;DPAD \u2195&nbsp;");
    static final String GUIDE = buttonString("<small>HOME \u2302</small>");

    Ui(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepads = new Gamepad[]{null, gamepad1, gamepad2};
    }

    // Button press state:
    private final boolean[][] buttonPressed = new boolean[3][15];
    private boolean buttonPress(boolean pressed, int gamepadIndex, int buttonIndex) {
        boolean press = pressed && !buttonPressed[gamepadIndex][buttonIndex];
        buttonPressed[gamepadIndex][buttonIndex] = pressed;
        return press;
    }

    double nextAdvanceTime;
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double REPEAT_DELAY = 0.1; // Seconds after any repeat to repeat again
    private boolean repeatableButtonPress(boolean pressed, int gamepadIndex, int index) {
        boolean press = pressed && !buttonPressed[gamepadIndex][index];
        buttonPressed[gamepadIndex][index] = pressed;
        if (press) {
            nextAdvanceTime = time() + INITIAL_DELAY;
        } else if ((pressed) && (time() > nextAdvanceTime)) {
            nextAdvanceTime = time() + REPEAT_DELAY;
            press = true;
        }
        return press;
    }

    double time() {
        return System.nanoTime() * 1e-9;
    }

    // Query for button presses on the specified gamepad:
    boolean a(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].a, gamepadIndex, 0); }
    boolean b(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].b, gamepadIndex, 1); }
    boolean x(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].x, gamepadIndex, 2); }
    boolean y(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].y, gamepadIndex, 3); }
    boolean dpadUp(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].dpad_up, gamepadIndex, 4); }
    boolean dpadDown(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].dpad_down, gamepadIndex, 5); }
    boolean dpadLeft(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].dpad_left, gamepadIndex, 6); }
    boolean dpadRight(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].dpad_right, gamepadIndex, 7); }
    boolean leftTrigger(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].left_trigger >= ANALOG_THRESHOLD, gamepadIndex, 8); }
    boolean rightTrigger(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].right_trigger >= ANALOG_THRESHOLD, gamepadIndex, 9); }
    boolean leftBumper(int gamepadIndex) { return repeatableButtonPress(gamepads[gamepadIndex].left_bumper, gamepadIndex, 10); }
    boolean rightBumper(int gamepadIndex) { return repeatableButtonPress(gamepads[gamepadIndex].right_bumper, gamepadIndex, 11); }
    boolean guide(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].guide, gamepadIndex, 12); }
    boolean start(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].start, gamepadIndex, 13); }
    boolean back(int gamepadIndex) { return buttonPress(gamepads[gamepadIndex].back, gamepadIndex, 14); }

    boolean ok(int gamepadIndex) { return a(gamepadIndex); }
    boolean cancel(int gamepadIndex) { return b(gamepadIndex); }

    // Query for button presses on the default gamepad:
    boolean a() { return a(defaultGamepad); }
    boolean b() { return b(defaultGamepad); }
    boolean x() { return x(defaultGamepad); }
    boolean y() { return y(defaultGamepad); }
    boolean dpadUp() { return dpadUp(defaultGamepad); }
    boolean dpadDown() { return dpadDown(defaultGamepad); }
    boolean dpadLeft() { return dpadLeft(defaultGamepad); }
    boolean dpadRight() { return dpadRight(defaultGamepad); }
    boolean leftTrigger() { return leftTrigger(defaultGamepad); }
    boolean rightTrigger() { return rightTrigger(defaultGamepad); }
    boolean leftBumper() { return leftBumper(defaultGamepad); }
    boolean rightBumper() { return rightBumper(defaultGamepad); }
    boolean guide() { return guide(defaultGamepad); }
    boolean start() { return start(defaultGamepad); }
    boolean back() { return back(defaultGamepad); }

    boolean ok() { return ok(defaultGamepad); }
    boolean cancel() { return cancel(defaultGamepad); }

    // Set the default gamepad:
    void setGamepad(int gamepadIndex) {
        if (gamepadIndex < 1 || gamepadIndex > 2)
            throw new IllegalArgumentException("gamepadIndex must be 1 or 2");
        this.defaultGamepad = gamepadIndex;
    }
    // Return the default gamepad:
    Gamepad gamepad() {
        return gamepads[defaultGamepad];
    }

    // Incrementally add a string to the telemetry without emitting an newline character:
    void add(String string) {
        stringBuilder.append(string);
    }
    void add(String format, Object... args) {
        stringBuilder.append(String.format(format, args));
    }
    // Add a string to the telemetry and emit a newline character:
    void out(String string) {
        telemetry.addLine(stringBuilder.toString() + string);
        stringBuilder.setLength(0); // Clear the stringBuilder for the next output
    }
    void out(String format, Object... args) {
        telemetry.addLine(stringBuilder.toString() + String.format(format, args));
        stringBuilder.setLength(0); // Clear the stringBuilder for the next output
    }
    // Send the telemetry to the Driver Station:
    void update() {
        if (stringBuilder.length() > 0) {
            telemetry.addLine(stringBuilder.toString());
            stringBuilder.setLength(0); // Clear the stringBuilder for the next output
        }
        telemetry.update();
    }

    // Return 'true' if the user has requested the program to stop, 'false' otherwise:
    boolean isStopRequested() {
        return Thread.interrupted();
    }

    // Return 'true' if A was pressed, 'false' otherwise:
    boolean okCancel() {
        while (!isStopRequested() && !cancel()) {
            if (ok())
                return true;
        }
        return false;
    }
}
