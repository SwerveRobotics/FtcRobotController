package org.firstinspires.ftc.teamMentor;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;
import org.swerverobotics.ftc.GoBildaPinpointDriver;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/*
 * This package handles MentorBot calibration.
 */

/**
 * Class to handle screen output and gamepad input.
 */
class Io {
    final Telemetry telemetry;
    final Gamepad gamepad;

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

    Io(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    // Button press state:
    private final boolean[] buttonPressed = new boolean[15];
    private boolean buttonPress(boolean pressed, int index) {
        boolean press = pressed && !buttonPressed[index];
        buttonPressed[index] = pressed;
        return press;
    }

    double nextAdvanceTime;
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double REPEAT_DELAY = 0.1; // Seconds after any repeat to repeat again
    private boolean repeatableButtonPress(boolean pressed, int index) {
        boolean press = pressed && !buttonPressed[index];
        buttonPressed[index] = pressed;
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

    boolean ok() { return a(); }
    boolean cancel() { return b(); }
    boolean a() { return buttonPress(gamepad.a, 0); }
    boolean b() { return buttonPress(gamepad.b, 1); }
    boolean x() { return buttonPress(gamepad.x, 2); }
    boolean y() { return buttonPress(gamepad.y, 3); }
    boolean up() { return buttonPress(gamepad.dpad_up, 4); }
    boolean down() { return buttonPress(gamepad.dpad_down, 5); }
    boolean left() { return buttonPress(gamepad.dpad_left, 6); }
    boolean right() { return buttonPress(gamepad.dpad_right, 7); }
    boolean leftTrigger() { return buttonPress(gamepad.left_trigger >= ANALOG_THRESHOLD, 8); }
    boolean rightTrigger() { return buttonPress(gamepad.right_trigger >= ANALOG_THRESHOLD, 9); }
    boolean leftBumper() { return repeatableButtonPress(gamepad.left_bumper, 10); }
    boolean rightBumper() { return repeatableButtonPress(gamepad.right_bumper, 11); }
    boolean guide() { return buttonPress(gamepad.guide, 12); }
    boolean start() { return buttonPress(gamepad.start, 13); }
    boolean back() { return buttonPress(gamepad.back, 14); }

    void out(String string) {
        telemetry.addLine(string);
    }
    void out(String format, Object... args) {
        telemetry.addLine(String.format(format, args));
    }
    void update() {
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

/**
 * Class for encapsulating the menu system.
 * @noinspection UnnecessaryUnicodeEscape, unused
 */
class Menu {
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed
    static private final String DESCRIPTOR_SEPARATOR = "::";
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double REPEAT_DELAY = 0.1; // Seconds after any repeat to repeat again

    Io io; // Use this for user input and output
    ArrayList<Menu.Widget> menuStack = new ArrayList<>(); // Stack of menus, the last is the current
    int lastInput; // Last quantized input (-1, 0 or 1)
    double nextAdvanceTime; // Time at which to advance the value

    public abstract static class Widget {
        String description;
        boolean isEnabled;
        boolean isStarred;

        private Widget(String descriptor) {
            // The description comes after the last separator:
            int lastIndex = descriptor.lastIndexOf(DESCRIPTOR_SEPARATOR);
            if (lastIndex != -1) {
                descriptor = descriptor.substring(lastIndex + DESCRIPTOR_SEPARATOR.length());
            }
            description = descriptor;
            isEnabled = true; // Default is to be enabled
        }
        abstract public String string();
    }
    public static class MenuWidget extends Menu.Widget {
        ArrayList<Menu.Widget> widgets = new ArrayList<>(); // List of widgets in this menu
        int current; // Index in widgets that has the UI focus
        public MenuWidget(String descriptor) {
            super(descriptor);
        }
        public String string() {
            return description + "..."; // "\uD83D\uDCC1 is Folder symbol
        }
    }
    public static class ToggleWidget extends Menu.Widget {
        boolean value;
        Consumer<Boolean> callback;
        public ToggleWidget(String descriptor, boolean value, Consumer<Boolean> callback) {
            super(descriptor); this.value = value; this.callback = callback;
            callback.accept(value); // Inform callback of initial value
        }
        public String string() {
            return (description + ": <b><big>" + (value ? "\u2705" : "\u2610") + "</big></b>"); // checked-box, empty box
        }
    }
    public static class ListWidget extends Menu.Widget {
        int index;
        String[] list;
        BiConsumer<Integer, String> callback;
        public ListWidget(String descriptor, int index, String[] list, BiConsumer<Integer, String> callback) {
            super(descriptor); this.index = index; this.list = list; this.callback = callback;
            callback.accept(index, list[index]); // Inform callback of initial value
        }
        public String string() {
            return description + ": <b>" + list[index] + "</b> \u2194\uFE0F"; // blue left/right arrow
        }
    }
    public static class ActivationWidget extends Menu.Widget {
        Function<Boolean, String> callback;
        public ActivationWidget(String descriptor, Function<Boolean, String> callback) {
            super(descriptor); this.callback = callback;
            callback.apply(true); // Inform callback of initial value
        }
        public String string() { return "\u2757 " + callback.apply(false); } // red exclamation mark
    }
    public static class StatsWidget extends Menu.Widget {
        Supplier<String> callback;
        public StatsWidget(String descriptor, Supplier<String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() { return "\uD83D\uDCCA " + description + "..."; }
    }
    public static class RunWidget extends Menu.Widget {
        Runnable runnable;
        public RunWidget(String descriptor, Runnable runnable) {
            super(descriptor);
            this.runnable = runnable;
        }
        public String string() {
            return description;
        }
    }
    public static class NumericWidget extends Menu.Widget {
        String units;
        double value;
        NumericInput inputter;
        Consumer<Double> callback;
        public NumericWidget(String descriptor, String units, double initialValue, int startDigit, int decimalDigits, double minValue, double maxValue, Consumer<Double> callback) {
            super(descriptor);
            this.units = units;
            this.value = initialValue;
            this.inputter = new NumericInput(this, "value", startDigit, decimalDigits, minValue, maxValue);
            this.callback = callback;
        }
        public String string() {
            return description + ": <b>" + inputter.simpleString() + "</b> " + units;
        }
    }

    // Constructor:
    public Menu(Io io) {
        this.io = io;
        menuStack.add(new Menu.MenuWidget("")); // The root menu
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Update loop for the menu.
    String update() {
        StringBuilder output = new StringBuilder();
        String footer = "";

        // Add a header with submenu names:
        if (menuStack.size() > 1) {
            output.append("<big><b>");
            for (int i = 1; i < menuStack.size(); i++) {
                if (i > 1)
                    output.append("\u00b7");
                output.append(menuStack.get(i).description);
            }
            output.append("...</b></big>\n\n");
            footer = "\nPress " + Io.GUIDE + " for previous menu.";
        }

        // Process dpad up and down with auto-repeat and clamping:
        Menu.MenuWidget menu = (Menu.MenuWidget) menuStack.get(menuStack.size() - 1);
        int input = io.gamepad.dpad_up ? -1 : (io.gamepad.dpad_down ? 1 : 0); // -1, 0 or 1
        if (input != lastInput) {
            nextAdvanceTime = time() + INITIAL_DELAY;
            lastInput = input;
            menu.current += lastInput;
        } else if (time() > nextAdvanceTime) {
            nextAdvanceTime = time() + REPEAT_DELAY;
            menu.current += lastInput;
        }
        menu.current = Math.max(0, Math.min(menu.widgets.size() - 1, menu.current));

        // @@@ isEnabled should actually disable the widgets
        // @@@ Highlight just the changeable thing (or entire line if not changeable, as when disabled)
        // @@@ Encapsulate actions into the objects themselves

        // Now output the widgets:
        for (int i = 0; i < menu.widgets.size(); i++) {
            Menu.Widget widget = menu.widgets.get(i);
            String line;
            if (i != menu.current) {
                String bullet = (widget.isStarred) ? "\u2606" : "\u25c7"; // Hollow star or circle
                line = bullet + " " + widget.string() + "\n";
            } else {
                String bullet = (widget.isStarred) ? "\u2605" : "\u25c6"; // Solid star or circle
                if ((widget.isEnabled) && (widget instanceof Menu.NumericWidget)) {
                    // Numerics have their own highlighting and updating rule:
                    Menu.NumericWidget numericWidget = (Menu.NumericWidget) widget;
                    numericWidget.callback.accept(numericWidget.inputter.getValue());
                    line = bullet + " " + numericWidget.description + ": <big><big>" +
                            numericWidget.inputter.update(io.gamepad) + "</big></big> " +
                            numericWidget.units + "\n";
                } else {
                    // Highlight current item:
                    line = "<span style='background: " + Io.HIGHLIGHT_COLOR
                            + "'>" + bullet + " " + widget.string() + "</span>\n";
                }
            }
            if (!widget.isEnabled) {
                line = "<font color='#808080'>" + line + "</font>";
            }
            output.append(line);
        }

        Menu.Widget widget = menu.widgets.get(menu.current);
        if (io.guide()) {
            if (menuStack.size() > 1) {
                // Pop up the menu stack:
                menuStack.remove(menuStack.size() - 1);
            }
        } else if (widget instanceof Menu.ToggleWidget) {
            Menu.ToggleWidget toggleWidget = (Menu.ToggleWidget) widget;
            if (io.ok()) {
                toggleWidget.value = !toggleWidget.value;
                toggleWidget.callback.accept(toggleWidget.value);
            }
        } else if (widget instanceof Menu.ListWidget) {
            Menu.ListWidget listWidget = (Menu.ListWidget) widget;
            boolean left = io.left();
            boolean right = io.right();
            if (left || right) {
                if (left) {
                    listWidget.index--;
                    if (listWidget.index < 0)
                        listWidget.index = 0;
                }
                if (right) {
                    listWidget.index++;
                    if (listWidget.index >= listWidget.list.length)
                        listWidget.index = listWidget.list.length - 1;
                }
                listWidget.callback.accept(listWidget.index, listWidget.list[listWidget.index]);
            }
        } else if (widget instanceof Menu.ActivationWidget) {
            if (io.ok()) {
                Menu.ActivationWidget activationWidget = (Menu.ActivationWidget) widget;
                activationWidget.callback.apply(true);
            }
        } else if (widget instanceof Menu.StatsWidget) {
            if (io.ok()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof Menu.MenuWidget) {
            if (io.ok()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof Menu.RunWidget) {
            Menu.RunWidget runWidget = (Menu.RunWidget) widget;
            if (io.ok()) {
                if (runWidget.isEnabled) {
                    runWidget.runnable.run();
                }
            }
        }

        output.append(footer);
        return output.toString();
    }

    // Add a new widget to the appropriate spot in the menu hierarchy:
    public Menu.Widget add(Menu.Widget newWidget) {
        Menu.MenuWidget menu = (Menu.MenuWidget) menuStack.get(0); // Root menu
        String descriptor = newWidget.description;

        // Peel off the hierarchy which is in the form "Vision::Configuration::Setting":
        while (true) {
            int index = descriptor.indexOf(DESCRIPTOR_SEPARATOR);
            if (index == -1)
                break; // ====>

            // Peel off the first menu name from the descriptor:
            String submenuName = descriptor.substring(0, index);
            descriptor = descriptor.substring(index + DESCRIPTOR_SEPARATOR.length());

            // Find or create the submenu:
            Menu.MenuWidget submenu = null;
            for (Menu.Widget widget : menu.widgets) {
                if ((widget instanceof Menu.MenuWidget) && (widget.description.equals(submenuName))) {
                    submenu = (Menu.MenuWidget) widget;
                    break;
                }
            }
            if (submenu == null) {
                submenu = new Menu.MenuWidget(submenuName);
                menu.widgets.add(submenu);
            }
            // Descend into that submenu:
            menu = submenu;
        }
        menu.widgets.add(newWidget);
        return newWidget;
    }
}


/**
 * Class to handle gamepad input of decimal numbers.
 */
class NumericInput {
    final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    final double REPEAT_DELAY = 0.15; // Seconds after any repeat to repeat again

    Object object; // Object being modified
    String fieldName; // Name of the field in the object being modified
    int digit; // Focus digit (1 is tens, 0 is ones, -1 is tenths, etc.)
    int decimalDigits; // Number of digits to show after the decimal
    String showFormat; // Format string precomputed from decimalDigits
    double minValue; // Clamp ranges
    double maxValue;
    Field field; // Reference to the field being modified
    int lastInput; // Last quantized input (-1, 0 or 1)
    double nextAdvanceTime; // Time at which to advance the value
    boolean dpadLeftPressed;
    boolean dpadRightPressed;

    // Take a reference to the object and the name of its field to be updated. We use reflection
    // to make the calling code's life a little easier. 'startDigit' dictates the starting
    // focus, 'decimalDigits' is the number of decimal digits to support, 'minValue' and
    // 'maxValue' specify the acceptable ranges.
    NumericInput(Object object, String fieldName, int startDigit, int decimalDigits, double minValue, double maxValue) {
        this.object = object;
        this.fieldName = fieldName;
        this.digit = startDigit;
        this.decimalDigits = decimalDigits;
        this.showFormat = String.format("%%.%df", decimalDigits);
        this.minValue = minValue;
        this.maxValue = maxValue;

        try {
            field = object.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
        } catch (NoSuchFieldException|NullPointerException e) {
            throw new RuntimeException(e);
        }
    }

    // Get the current value:
    double getValue() {
        try {
            return field.getDouble(object);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    // Set the value into the object:
    void setValue(double value) {
        try {
            field.setDouble(object, value);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    // Return the time, in seconds.
    double time() {
        return System.nanoTime() / 1e9;
    }

    // Return the value as a simple string.
    String simpleString() {
        String format = String.format("%%.%df", decimalDigits);
        return String.format(format, getValue());
    }

    // Update the variable according to the latest gamepad input.
    String update(Gamepad gamepad) {
        if ((gamepad.dpad_left) && (!dpadLeftPressed)) {
            digit = Math.min(digit + 1, 2);
        }
        dpadLeftPressed = gamepad.dpad_left;
        if ((gamepad.dpad_right) && (!dpadRightPressed)) {
            digit = Math.max(digit - 1, -decimalDigits);
        }
        dpadRightPressed = gamepad.dpad_right;

        double value = getValue();
        if ((digit > 0) && (Math.pow(10, digit) > Math.abs(value))) {
            digit--;
        }

        // Advance the value according to the right stick state:
        int input = (gamepad.right_stick_y < -0.1) ? 1 : ((gamepad.right_stick_y > 0.1) ? -1 : 0); // -1, 0 or 1
        if (input != lastInput) {
            nextAdvanceTime = time() + INITIAL_DELAY;
            lastInput = input;
            value += lastInput * Math.pow(10, digit);
        } else if (time() > nextAdvanceTime) {
            nextAdvanceTime = time() + REPEAT_DELAY;
            value += lastInput * Math.pow(10, digit);
        }

        // Clamp new value to acceptable range:
        value = Math.max(minValue, Math.min(maxValue, value));

        // Set the updated value into the class:
        setValue(value);

        // Show the new value:
        String showValue = String.format(showFormat, value);
        int digitOffset = (digit >= 0) ? -digit - 1 : -digit;
        int digitIndex = showValue.indexOf(".") + digitOffset;
        digitIndex = Math.max(0, Math.min(digitIndex, showValue.length() - 1));
        String prefix = showValue.substring(0, digitIndex);
        String middle = showValue.substring(digitIndex, digitIndex + 1);
        String suffix = showValue.substring(digitIndex + 1);

        // Highlight the focus digit:
        middle = "<span style='background: " + Io.HIGHLIGHT_COLOR + "'>" + middle + "</span>";

        // Blink the underline every half second:
        if ((((int) (time() * 2)) & 1) != 0) {
            middle = "<u>" + middle + "</u>";
        }

        return prefix + middle + suffix;
    }
}

@TeleOp(name = "MentorBot Calibrator", group = "Tuning")
public class Calibrate extends LinearOpMode {
    Io io; // Input/output abstraction

    // Structure to define the various test screens.
    static class Screen {
        Entry[] entries; // Entries for the screen, built dynamically when screen is selected
        int selection; // Index of the currently selected entry
        String name; // Name of the joint that is configured in this screen
        String anglesDescription; // Description of the angles convention for this joint
        int id; // Joint ID
        int state; // Finite state machine state
        Servo[] servos; // Array of servos for this joint
        BiConsumer<Screen, Calibration> method;
        Screen(String name, int id, BiConsumer<Screen, Calibration> method, String description) {
            this.name = name;
            this.id = id;
            this.method = method;
            this.anglesDescription = description;
        }
    }

    // Structure to define the UI handing of a menu entry.
    static class Entry {
        final String LOWLIGHT_COLOR = "#808080"; // Grey lowlight color

        String name; // Name of the entry (e.g. "Start position")
        String suffix; // Suffix to show after the value (e.g. "degrees")
        boolean isAngle; // True if this entry is an angle (as opposed to a position)
        boolean isEnabled; // True if the user can edit this entry
        boolean isEditing; // True if the user is actively editing this entry
        NumericInput inputter;

        Entry(String name, boolean isAngle, NumericInput inputter) {
            this.name = name;
            this.isAngle = isAngle;
            this.suffix = isAngle ? "\u00b0" : "";
            this.inputter = inputter;
        }
    }

    static class CalibrateState {
        static final int INITIALIZE = 0;
        static final int ENTER_START = 1;
        static final int ENTER_ALL = 2;
    }

    // Calibrate a specific joint. This is called within the UI input loop.
    void calibrateJoint(Screen screen, Calibration calibration) {
        Calibration.JointCalibration joint = calibration.jointCalibrations[screen.id];

        // If either servo is null, we can't calibrate this joint:
        if (Arrays.stream(screen.servos).anyMatch(servo -> servo == null)) {
            for (int i = 0; i < screen.servos.length; i++) {
                if (screen.servos[i] == null) {
                    String servoName = Id.DEVICE_NAMES[screen.id][i];
                    io.out(Io.CRITICAL_ICON + "The servo '" + servoName + "' couldn't be found in the robot's Configuration.");
                }
            }
        } else if (screen.state == CalibrateState.INITIALIZE) {
            screen.entries = new Entry[]{
                    new Entry("Start position", false,
                            new NumericInput(joint, "start", -3, 3, 0, 1)),
                    new Entry("A position", false,
                            new NumericInput(joint, "positionA", -3, 3, 0, 1)),
                    new Entry("A angle", true,
                            new NumericInput(joint, "degreesA", -1, 1, -180, 180)),
                    new Entry("B position", false,
                            new NumericInput(joint, "positionB", -3, 3, 0, 1)),
                    new Entry("B angle", true,
                            new NumericInput(joint, "degreesB", -1, 1, -180, 180)),
                    new Entry("Minimum position", false,
                            new NumericInput(joint, "min", -3, 3, 0, 1)),
                    new Entry("Maximum position", false,
                            new NumericInput(joint, "max", -3, 3, 0, 1))
            };

            if (!joint.isValid()) {
                screen.state = CalibrateState.ENTER_START;
            } else {
                io.out(screen.name + "'s current settings:\n");
                for (int i = 0; i < screen.entries.length; i++) {
                    Entry entry = screen.entries[i];
                    io.out("&emsp;%s: %s%s", entry.name, entry.inputter.simpleString(), entry.suffix);
                }
                io.out("");
                io.out(Io.WARNING_ICON +
                        "Press " + Io.A + " to edit these values, " + Io.B + " to restart " +
                        "from scratch for this joint.");
                if (io.a()) {
                    // Edit the existing values:
                    screen.state = CalibrateState.ENTER_ALL;
                    for (int i = 0; i < screen.entries.length; i++) {
                        screen.entries[i].isEnabled = true;
                    }
                } else if (io.b()) {
                    // Start from scratch:
                    screen.state = CalibrateState.ENTER_START;
                    for (int i = 0; i < screen.entries.length; i++) {
                        screen.entries[i].inputter.setValue(0); // Reset everything to zero
                    }
                }
            }
        } else if (screen.state == CalibrateState.ENTER_START) {
            io.out("Enter the start position for the " + screen.name + " servo:\n");
            io.out("<big><big>&emsp;" + screen.entries[0].inputter.update(gamepad1) + "</big></big>\n");
            io.out("Use the right stick to adjust the value, Dpad to select digit, " +
                    Io.A + " when ready to activate the servo.");
            if (io.a()) {
                screen.state = CalibrateState.ENTER_ALL;
                screen.entries[0].isEnabled = true;
                screen.entries[0].isEditing = true;
            }
        } else if (screen.state == CalibrateState.ENTER_ALL) {
            io.out(screen.anglesDescription + " Servo's current angle according to calibration: %.1f\u00b0.\n",
                Math.toDegrees(joint.positionToRadians(screen.servos[0].getPosition())));

            for (int i = 0; i < screen.entries.length; i++) {
                Entry entry = screen.entries[i];
                String value;
                if ((entry.isEditing) && (i == screen.selection)) {
                    value = "<big><big>" + entry.inputter.update(gamepad1) + entry.suffix + "</big></big>";
                } else {
                    value = entry.inputter.simpleString() + entry.suffix;
                }
                String bullet = (i == screen.selection) ? "\u25c6" : "\u25c7"; // Filled or empty diamond
                String name = entry.name;
                String line = String.format("%s %s: %s", bullet, name, value); // Empty diamond as a bullet
                if (!entry.isEnabled) {
                    line = String.format("<font color='%s'>%s</font>", entry.LOWLIGHT_COLOR, line);
                }
                if ((!entry.isEditing) && (i == screen.selection)) {
                    line = "<span style='background: " + Io.HIGHLIGHT_COLOR + "'>" + line + "</span>";
                }
                io.out(line);
            }
            Entry selectedEntry = screen.entries[screen.selection];
            if (selectedEntry.isEditing) {
                io.out("\nUse the right stick to adjust the value, Dpad to select digit, " +
                        Io.A + " when done.");

                // Tell the hardware the new position, assuming this isn't an angle entry:
                if (!selectedEntry.isAngle) {
                    for (Servo servo: screen.servos) {
                        servo.setPosition(selectedEntry.inputter.getValue());
                    }
                }
            } else {
                io.out("\nDpad to select, " + Io.A + " to edit.");
                if (io.up()) {
                    screen.selection = Math.max(screen.selection - 1, 0);
                }
                if (io.down()) {
                    screen.selection = Math.min(screen.selection + 1, screen.entries.length - 1);
                }
            }

            if (io.a() && selectedEntry.isEnabled) {
                selectedEntry.isEditing = !selectedEntry.isEditing;
                if (!selectedEntry.isEditing) {
                    // Recalculate min and max values:
                    joint.min = Math.min(joint.min, Math.min(joint.start, Math.min(joint.positionA, joint.positionB)));
                    joint.max = Math.max(joint.max, Math.max(joint.start, Math.max(joint.positionA, joint.positionB)));

                    calibration.saveToFile(); // We stopped editing so save the changes
                    if (screen.selection < screen.entries.length - 1) {
                        // When tuning from scratch,
                        if (!screen.entries[screen.selection + 1].isEnabled) {
                            // Automatically propagate the just-edited value to the next field
                            // so that the joint can stay where it was:
                            if (selectedEntry.inputter.fieldName.equals("start")) {
                                joint.positionA = joint.start;
                            } else if (selectedEntry.inputter.fieldName.equals("positionA")) {
                                joint.positionB = joint.positionA;
                            }
                            screen.selection++;
                            screen.entries[screen.selection].isEnabled = true;
                        }
                    }
                }
            }
        }

        telemetry.update();
    }

    // Calibrate the arm's joints.
    void calibrateJoints() {
        Calibration calibration = Calibration.loadFromFile();
        if (calibration == null)
            calibration = new Calibration();

        final Screen[] screens = {
                new Screen("Shoulder", Id.SHOULDER, this::calibrateJoint,
                        "0\u00b0 is straight forward horizontally, 90\u00b0 is straight up."),
                new Screen("Elbow1", Id.ELBOW1, this::calibrateJoint,
                        "0\u00b0 is when the joint is perfectly straight, -170ish\u00b0 when " +
                                "folded in the home position."),
                new Screen("Elbow2", Id.ELBOW2, this::calibrateJoint,
                        "-180\u00b0 is when the joint is perfectly straight, 10ish\u00b0 when " +
                                "folded in the home position."),
                new Screen("Elbow3", Id.ELBOW3, this::calibrateJoint,
                        "0\u00b0 is when the joint is perfectly straight, -170ish\u00b0 when " +
                                "folded in the home position."),
                new Screen("Wrist", Id.WRIST, this::calibrateJoint,
                        "0\u00b0 is when the claw is aligned with the arm, positive when " +
                                "turned to the left."),
                new Screen("Claw", Id.CLAW, this::calibrateJoint,
                        "0\u00b0 is when the claw is closed, positive when open."),
                new Screen("Turret", Id.TURRET, this::calibrateJoint,
                        "0\u00b0 is when the arm is perfectly straight forward, positive " +
                                "when the arm is turned to the left."),
        };

        int screenIndex = 0; // Index of the current screen

        // Initialize the servo objects for each screen:
        for (Screen screen: screens) {
            screen.servos = new Servo[Id.DEVICE_NAMES[screen.id].length];
            for (int i = 0; i < screen.servos.length; i++) {
                Servo servo = hardwareMap.tryGet(Servo.class, Id.DEVICE_NAMES[screen.id][i]);
                screen.servos[i] = servo;
                if (servo != null) {
                    ServoController controller = servo.getController();
                    controller.pwmDisable(); // Disable power to the servo to allow it to be moved by hand
                }
            }
        }

        while (opModeIsActive()) {
            // Handle the UI:
            if (io.guide())
                return; // ====>
            if (io.leftBumper())
                screenIndex = Math.max(screenIndex - 1, 0);
            if (io.rightBumper())
                screenIndex = Math.min(screenIndex + 1, screens.length - 1);

            StringBuilder header = new StringBuilder(""); // "<small>\u2b9c</small>"
            for (int i = 0; i < screens.length; i++) {
                if (i == screenIndex) {
                    header.append(String.format("<span style='background: %s;'>", Io.HIGHLIGHT_COLOR));
                    header.append(screens[i].name);
                    header.append("</span>");
                } else {
                    header.append(screens[i].name);
                }
                if (i < screens.length - 1)
                    header.append("<small>|</small>");
            }
            header.append("\n"); // "<small>\u2b9e</small>\n"
            telemetry.addLine(header.toString());

            // Call the current screen's method
            screens[screenIndex].method.accept(screens[screenIndex], calibration);
        }
    }

    // Initialize the arm if the calibration is valid. Warn the user.
    Arm initializeArm() {
        Arm arm = null;
        Calibration calibration = Calibration.loadFromFile();
        while (opModeIsActive() && !io.guide()) {
            if (calibration == null) {
                io.out(Io.CRITICAL_ICON + "The calibration file couldn't be found. " +
                        "Please calibrate the arm first.");
            } else {
                for (int i = 0; i < calibration.jointCalibrations.length; i++) {
                    if (!calibration.jointCalibrations[i].isValid()) {
                        io.out(Io.CRITICAL_ICON + "The " + Id.DEVICE_NAMES[i][0] + " joint is missing valid data. " +
                                "Please calibrate first.");
                    }
                }
                if (arm == null) {
                    arm = new Arm(hardwareMap, telemetry);
                    for (int i = 0; i < arm.joints.length; i++) {
                        for (Servo servo: arm.joints[i].servos) {
                            ServoController controller = servo.getController();
                            controller.pwmDisable(); // Disable power to the servo to allow it to be moved by hand
                        }
                    }
                }
                io.out(Io.CRITICAL_ICON + "WARNING: As soon as you press " + Io.A + ", all servos " +
                        "will move to their calibrated start positions at the fastest velocity. " +
                        "Make sure that the arm is already in its start position before proceeding. " +
                        "\n\n" +
                        "Press " + Io.A + " to continue, " + Io.GUIDE + " to cancel. ");
                if (io.a()) {
                    return arm;
                }
            }
            io.update();
        }
        return null;
    }

    // Structure to hold the distance input:
    static class Distance {
        double distance;
    }

    // Measure the distance and height correction factors for the arm's kinematics.
    void measureFudge() {
        final int MEASUREMENT_COUNT = 10; // Number of samples to average
        final double TARGET_ARM_HEIGHT = 5; // Target arm height above the field, in inches
        final double MAX_DISTANCE = 42 - Specs.Robot.LENGTH/2 + Specs.Arm.TURRET_OFFSET.x;
        final double EXTENSION_INCHES_PER_SECOND = 5; // Arm movement speed for direct user control

        Arm arm = initializeArm();
        if (arm == null)
            return; // ====> Abort tuning

        while (opModeIsActive()) {
            io.out("This will measure new fudge factors for the arm's kinematics.\n");
            io.out(Io.WARNING_ICON + "WARNING: Pressing " + Io.A + " will reset the existing " +
                    "fudge factors!\n");
            io.out("Press " + Io.A + " to continue, " + Io.GUIDE + " to cancel.");
            io.update();

            if (io.guide())
                return; // ====>
            if (io.a()) {
                arm.calibration.setDefaultFudges();
                break; // ====>
            }
        }

        Calibration.Fudge[] fudges = new Calibration.Fudge[MEASUREMENT_COUNT];
        int measurement = 0;
        double theoreticalHeight = TARGET_ARM_HEIGHT; // Height to maintain while measuring
        double theoreticalDistance = 0; // Start closest to the robot
        double minMeasuredReach = 0;
        double minTheoreticalReach = 0;
        double lastTime = System.nanoTime() * 1e-9; // Time of the last measurement
        Distance measuredDistance = new Distance(); // Object to hold the measured distance

        NumericInput inputter = new NumericInput(measuredDistance, "distance", -1, 2, 0, MAX_DISTANCE);
        while (opModeIsActive()) {
            double time = System.nanoTime() * 1e-9;
            double deltaT = time - lastTime;
            lastTime = time;

            if (measurement == MEASUREMENT_COUNT) {
                io.out("Done measuring fudge factors!\n");
                io.out("Press " + Io.A + " to save and return to the main menu, " +
                        Io.B + " to cancel and discard the results.\n");
                io.update();
                if (io.a()) {
                    arm.calibration.minReach = minMeasuredReach;
                    arm.calibration.fudges = fudges; // Save the fudge factors
                    arm.calibration.saveToFile(); // Save the resulting calibration
                    return; // ====> Done!
                }
                if (io.b()) {
                    return; // ====> Discard the results and return to the main menu
                }
            } else {
                if (io.guide()) {
                    // Don't save the calibration if the user cancels before the final measurement:
                    return; // ====>
                }
                if (measurement == 0) {
                    io.out("Use the left stick to adjust the arm so that it touches " +
                            "the floor at the closest point to the robot. Measure the distance " +
                            "from the front of the robot and enter it using the Dpad and right " +
                            "stick.\n");
                } else {
                    io.out("Fudge measurement %d of %d\n", measurement + 1, MEASUREMENT_COUNT);
                    io.out("Use the left stick to adjust the arm's height so that it " +
                            "touches the floor. Measure the distance from the front of the " +
                            " robot and enter it using the Dpad and right stick.\n");
                }
                io.out("Enter measured distance: <big><big>%s</big></big> inches\n", inputter.update(gamepad1));
                io.out("Theoretical distance: %.2f\", height: %.2f\"\n", theoreticalDistance, theoreticalHeight);
                io.out("Press " + Io.A + " once touching the floor and the measured distance is entered.");

                theoreticalHeight += -gamepad1.left_stick_y * deltaT * EXTENSION_INCHES_PER_SECOND;
                theoreticalHeight = Math.max(-5, Math.min(theoreticalHeight, 12)); // Clamp height
                if (measurement == 0) {
                    theoreticalDistance += gamepad1.left_stick_x * deltaT * EXTENSION_INCHES_PER_SECOND;
                    theoreticalDistance = Math.max(0, Math.min(theoreticalDistance, MAX_DISTANCE)); // Clamp distance
                }
                arm.pickup(theoreticalDistance, theoreticalHeight); // Move the arm to the requested position

                if (io.a()) {
                    // The very first measurement sets the minimum reach:
                    if (measurement == 0) {
                        minMeasuredReach = measuredDistance.distance;
                        minTheoreticalReach = theoreticalDistance;
                    }

                    // Save the current results:
                    fudges[measurement] = new Calibration.Fudge(
                            measuredDistance.distance,
                            theoreticalDistance,
                            theoreticalHeight); // If theoretical height is 1", say, then dial 1" to get to true 0"

                    // Setup for the next measurement:
                    measurement++;
                    theoreticalHeight = TARGET_ARM_HEIGHT;
                    theoreticalDistance = minTheoreticalReach
                            + (measurement * (MAX_DISTANCE - minTheoreticalReach) / (MEASUREMENT_COUNT - 1));
                    measuredDistance.distance = theoreticalDistance; // Pre-seed
                }
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
                arm.update(new Pose2d(0, 0, 0), packet.fieldOverlay());
                MecanumDrive.sendTelemetryPacket(packet);
                io.update();
            }
        }
    }

    // Tune the velocity and acceleration of the arm while testing the motion.
    void tuneKinematics() {
        final double EXTENSION_INCHES_PER_SECOND = 20; // Claw movement speed for direct user control

        // This function's state, accessible by lambda functions:
        class LambdaState {
            final Arm arm;
            String descriptor;
            LambdaState(Arm arm) { this.arm = arm; }
        }

        Arm arm = initializeArm();
        if (arm == null)
            return; // ====> Abort tuning

        LambdaState state = new LambdaState(arm);
        double maxSpeedSeconds = Math.toRadians(60) / arm.calibration.maxSpeed;
        double maxAccelSeconds = arm.calibration.maxSpeed / arm.calibration.acceleration;
        double maxDecelSeconds = arm.calibration.maxSpeed / -arm.calibration.deceleration;

        Menu.Widget modeWidget = new Menu.ListWidget("Position", 0,
                new String[]{"High basket", "Pickup", "Start", "Wrist", "Claw", "Turret"}, (index, destination) -> {
            state.descriptor = destination;
        });
        Menu.Widget maxVelWidget = new Menu.NumericWidget("Max velocity", "s/60\u00b0",
                maxSpeedSeconds, -2, 2, 0.01, 1,
                value -> {
                    state.arm.calibration.maxSpeed = Math.toRadians(60) / value;
                    state.arm.calibration.acceleration = state.arm.calibration.maxSpeed / maxAccelSeconds;
                    state.arm.calibration.deceleration = -state.arm.calibration.maxSpeed / maxDecelSeconds;
                    state.arm.calibration.saveToFile();
                });
        Menu.Widget accelWidget = new Menu.NumericWidget("Acceleration", "s to max",
                maxAccelSeconds, -2, 2, 0.01, 10,
                value -> {
                    state.arm.calibration.acceleration = state.arm.calibration.maxSpeed / value;
                    state.arm.calibration.saveToFile();
                });
        Menu.Widget decelWidget = new Menu.NumericWidget("Deceleration", "s to stop",
                maxDecelSeconds, -2, 2, 0.01, 10,
                value -> {
                    state.arm.calibration.deceleration = state.arm.calibration.maxSpeed / -value;
                    state.arm.calibration.saveToFile();
                });
        Menu.Widget sloMoWidget = new Menu.ToggleWidget("Slo-mo", true,
                enable -> {
                    state.arm.sloMo = enable;
                    maxVelWidget.isEnabled = !enable;
                    accelWidget.isEnabled = !enable;
                    decelWidget.isEnabled = !enable;
                });

        Menu menu = new Menu(io);
        menu.add(modeWidget);
        menu.add(sloMoWidget);
        menu.add(maxVelWidget);
        menu.add(accelWidget);
        menu.add(decelWidget);

        double lastTime = System.nanoTime() * 1e-9;
        double distance = 20; // Distance to move the claw in inches
        double height = 5; // Height to move the claw in inches
        boolean isMoving = false;
        while (opModeIsActive()) {
            double time = System.nanoTime() * 1e-9;
            double deltaT = time - lastTime;
            lastTime = time;

            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            if (io.guide()) {
                arm.calibration.saveToFile();
                arm.sloMo = false;
                return; // ====>
            }

            Calibration.JointCalibration wristCalibration = arm.joints[Id.WRIST].calibration;
            Calibration.JointCalibration turretCalibration = arm.joints[Id.TURRET].calibration;

            io.out("<b>Tune kinematics</b>\n");
            if (io.gamepad.y) {
                isMoving = true;
                switch (state.descriptor) {
                    case "Wrist":
                        arm.wrist(wristCalibration.positionToRadians(wristCalibration.min));
                        break;
                    case "Claw":
                        arm.claw(false);
                        break;
                    case "Turret":
                        arm.turret(turretCalibration.positionToRadians(turretCalibration.min));
                        break;
                    default:
                        arm.home();
                        break;
                }
            } else if (io.gamepad.x) {
                isMoving = true;
                switch (state.descriptor) {
                    case "High basket":
                        arm.highBasket();
                        break;
                    case "Start":
                        arm.start();
                        break;
                    case "Pickup":
                        double newDistance = distance + io.gamepad.left_stick_x * EXTENSION_INCHES_PER_SECOND * deltaT;
                        double newHeight = height - io.gamepad.left_stick_y * EXTENSION_INCHES_PER_SECOND * deltaT;
                        if (Arm.computeTheoreticalReach(newDistance, newHeight) != null) {
                            distance = newDistance;
                            height = newHeight;
                        }
                        io.out("Distance: %.1f inches, Height: %.1f inches\n", distance, height);
                        arm.pickup(distance, height);
                        break;
                    case "Wrist":
                        arm.wrist(wristCalibration.positionToRadians(wristCalibration.max));
                        break;
                    case "Claw":
                        arm.claw(true);
                        break;
                    case "Turret":
                        arm.turret(turretCalibration.positionToRadians(turretCalibration.max));
                        break;
                }
            } else if (isMoving) {
                arm.halt();
                isMoving = false;
            }

            if (!isMoving) {
                io.out(menu.update());
            }
            io.out("Hold " + Io.X + " to move to " + state.descriptor + ", " +
                    Io.Y + " to go home, left stick to adjust pickup position.");
            arm.update(new Pose2d(0, 0, 0), canvas);

            MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    // Shape the stick response for driving:
    double shapeStick(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    // Show the raw input from the localization sensors.
    void testLocalizationSensors() {
        Poser.Ultrasonic[] ultrasonics = Poser.getUltrasonics();
        UltrasonicDistanceSensor[] ultrasonicDrivers = new UltrasonicDistanceSensor[ultrasonics.length];
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        for (int i = 0; i < ultrasonics.length; i++) {
            ultrasonicDrivers[i] = hardwareMap.tryGet(UltrasonicDistanceSensor.class, ultrasonics[i].name);
        }
        while (opModeIsActive()) {
            for (int i = 0; i < ultrasonics.length; i++) {
                if (ultrasonicDrivers[i] == null) {
                    telemetry.addLine("Ultrasonic sensor not found: " + ultrasonics[i].name);
                } else {
                    double distance = ultrasonicDrivers[i].getDistance(DistanceUnit.INCH);
                    telemetry.addLine(String.format("%s: %.1f inches", ultrasonics[i].name, distance));
                }
            }
            telemetry.addLine("\nPress " + Io.GUIDE + " to exit.");
            telemetry.update();
            if (io.guide()) {
                return; // ====>
            }
            sleep(20); // Don't query the sensors sooner than the return signal
        }
    }

    // Measure fudge factors for the ultrasonic sensors and locations on the field, all determined
    // by starting the robot in a corner with a repeatable known pose.
    void tuneLocalization() {
        Poser poser = Poser.getPoser(hardwareMap, telemetry, gamepad1, new Pose2d(0, 0, 0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, poser, telemetry, gamepad1);
        boolean poseSet = false;
        boolean zeroed = false;
        double xFudge = 0;
        double yFudge = 0;
        double lastUpdateTime = 0;
        String positionPrompt = "Position the robot in the human corner, facing TOWARDS the basket.\n";
        Pose2d startPose = new Pose2d(
                72 - Specs.Robot.LENGTH/2,
                -72 + Specs.Robot.WIDTH/2,
                Math.toRadians(180));
        if (MecanumDrive.isDevBot) {
            positionPrompt = "Position the robot in the human corner, facing AWAY FROM the basket.\n";
            startPose = new Pose2d(
                    72 - Specs.Robot.LENGTH/2,
                    -72 - Specs.Robot.WIDTH/2,
                    0);
        }

        while (opModeIsActive()) {
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            if (io.guide()) {
                return; // ====>
            }
            if (io.a()) {
                drive.setPose(startPose);
                poseSet = true;
            }
            if (io.b()) {
                poser.zeroFudges();
                zeroed = true;
            }
            if (!poseSet) {
                io.out(positionPrompt);
                io.out("Press " + Io.A + " when in position.\n");
            } else {
                io.out("Odometry pose: x=%.1f\", y=%.1f\", heading=%.1f\u00b0\n",
                        poser.odometryPose.position.x,
                        poser.odometryPose.position.y,
                        Math.toDegrees(poser.odometryPose.heading.log()));
                io.out("FieldSpecs.ABUTMENT_X if abutting (should be ~-15.5): %.2f\n",
                        -poser.odometryPose.position.x + Specs.Robot.LENGTH/2);
                io.out("Drive the robot to face the corner to tune ultrasonic fudges.\n");

                double time = System.nanoTime() * 1e-9;
                if (time - lastUpdateTime > 1) {
                    lastUpdateTime = time;
                    // In this instance, the odometry pose is the most accurate one. Compute
                    // the delta that we can add to the fused pose to get the accurate pose:
                    xFudge = poser.odometryPose.position.x - poser.fusedPose.position.x;
                    yFudge = poser.odometryPose.position.y - poser.fusedPose.position.y;
                }
                if (zeroed) {
                    io.out("New X fudge: %.2f\", new Y fudge: %.2f\"\n", xFudge, yFudge);
                } else {
                    double error = Math.hypot(xFudge, yFudge);
                    io.out("Error width fudges enabled: %.2f\" (x: %.2f\", y: %.2f\")\n", error, xFudge, yFudge);
                    io.out("Press " + Io.B + " to measure new fudge factors.\n");
                }
            }

            drive.updatePoseEstimate(false);
            poser.draw(canvas); // Draw the robot's pose

            PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                    shapeStick(-gamepad1.left_stick_y),
                    shapeStick(-gamepad1.left_stick_x)),
                    shapeStick(-gamepad1.right_stick_x));
            drive.setDrivePowers(velocity);

            MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    // Enable or disable PWM for all servos.
    void enableServos(boolean enable) {
        ArrayList<ServoController> servoControllers = new ArrayList<>();
        for (int id = 0; id < Id.COUNT; id++) {
            for (int i = 0; i < Id.DEVICE_NAMES[id].length; i++) {
                Servo servo = hardwareMap.tryGet(Servo.class, Id.DEVICE_NAMES[id][i]);
                if (servo != null) {
                    ServoController controller = servo.getController();
                    servoControllers.add(controller); // Keep a reference
                    if (enable) {
                        controller.pwmEnable();
                    } else {
                        controller.pwmDisable();
                    }
                }
            }
        }
        while (opModeIsActive() && !io.guide()) {
            telemetry.addLine(String.format("%d servos are now %s.", servoControllers.size(), (enable ? "enabled" : "disabled")));
            telemetry.update();
            if (io.guide()) {
                return; // ====>
            }
        }
    }

    @Override
    public void runOpMode() {
        io = new Io(telemetry, gamepad1);

        telemetry.addLine("MentorBot calibrator is ready to start!");
        telemetry.update();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        Menu menu = new Menu(io);
        menu.add(new Menu.RunWidget("Calibrate arm joints", this::calibrateJoints));
        menu.add(new Menu.RunWidget("Tune arm kinematics", this::tuneKinematics));
        menu.add(new Menu.RunWidget("Adjust arm fudge factors", this::measureFudge));
        menu.add(new Menu.RunWidget("Test localization sensors", this::testLocalizationSensors));
        menu.add(new Menu.RunWidget("Tune localization", this::tuneLocalization));
//        menu.add(new Menu.RunWidget("Enable servos", () -> enableServos(true)));
//        menu.add(new Menu.RunWidget("Disable servos", () -> enableServos(false)));

        waitForStart();
        while (opModeIsActive()) {
            io.out("<big>Calibration Main Menu</big>\n");
            io.out(menu.update());
            io.out("Press " + Io.GUIDE + " to return to this menu from any submenu.");
            telemetry.update();
        }
    }
}
