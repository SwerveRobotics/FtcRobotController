/**
 * Loony Tune is a parameters tuner for robots using Road Runner.
 */

package org.firstinspires.ftc.team417.roadrunner;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.canvas.CanvasOp;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.gson.Gson;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.wilyworks.common.WilyWorks;

import static java.lang.System.out;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.prefs.Preferences;

/**
 * Math helper for points and vectors:
 * @noinspection unused
 */
class Point { // Can't derive from vector2d because it's marked as final (by default?)
    public double x, y;
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point(Vector2d vector) {
        x = vector.x;
        y = vector.y;
    }
    public Point(VectorF vector) {
        x = vector.get(0);
        y = vector.get(1);
    }
    public Vector2d vector2d() { return new Vector2d(x, y); }
    public Point add(Point other) {
        return new Point(this.x + other.x, this.y + other.y);
    }
    public Point subtract(Point other) {
        return new Point(this.x - other.x, this.y - other.y);
    }
    public Point rotate(double theta) {
        return new Point(Math.cos(theta) * x - Math.sin(theta) * y,
                Math.sin(theta) * x + Math.cos(theta) * y);
    }
    public Point negate() { return new Point(-x, -y); }
    public Point multiply(double scalar) { return new Point(x * scalar, y * scalar); }
    public Point divide(double scalar) { return new Point(x / scalar, y / scalar); }
    public double dot(Point other) {
        return this.x * other.x + this.y * other.y;
    }
    public double cross(Point other) {
        return this.x * other.y - this.y * other.x;
    }
    public double distance(Point other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }
    public double length() {
        return Math.hypot(x, y);
    }
    public double atan2() { return Math.atan2(y, x); } // Rise over run
}

/**
 * Class responsible for managing all input and output for the tuner. Output goes to three
 * separate surfaces: Driver Station telemetry, FTC Dashboard telemetry, and the FTC Dashboard
 * field.
 * @noinspection unused
 */
class Io {
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed

    final private Telemetry telemetry; // Driver Station telemetry object
    private String welcomeMessage; // Welcome message to show only on Driver Station, if any
    public Gamepad gamepad; // Gamepad reference
    private boolean blankField = true; // True if the field should be blank, false if to show this year's game
    private String messageCopy = ""; // Copy of the most recently shown message
    private List<CanvasOp> canvasOpsCopy; // Copy of the most recently shown field canvas

    // The following are null when not in an active begin/end bracket:
    private StringBuilder message; // Message that is begin built
    private Canvas canvas; // Current FTC Dashboard field canvas
    public TelemetryPacket packet; // Current FTC Dashboard telemetry object

    Io(Gamepad gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        // Set the display format to use HTML and change the interval from 250 to 100ms for
        // more responsive UI:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(100);

        // Initialize the field view:
        begin();
        canvas(); // Draws the field
        end();
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
            nextAdvanceTime = LoonyTune.time() + INITIAL_DELAY;
        } else if ((pressed) && (LoonyTune.time() > nextAdvanceTime)) {
            nextAdvanceTime = LoonyTune.time() + REPEAT_DELAY;
            press = true;
        }
        return press;
    }

    // Button press status:
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

    // Set the message to be shown constantly on the Driver Station screen (at least until they
    // press Start):
    /** @noinspection SameParameterValue*/
    void setWelcomeMessage(String message) {
        welcomeMessage = message;
    }

    // Show a blank field if true, this year's game field if false:
    void setBlankField(boolean blankField) {
        this.blankField = blankField;
    }

    // Begin a UI update. The optional 'showField' parameter dictates whether draw the field or
    // show blank when updating the field:
    void begin() {
        assert(packet == null);
        assert(message == null);
        assert(canvas == null);

        packet = new TelemetryPacket();
        message = new StringBuilder();

        // Disable the welcome message if they've pressed the gamepad's 'start'"
        if (guide()) {
            welcomeMessage = null;
        }
    }

    // Add a string to the current UI update message. Note that this does NOT add a newline:
    void out(String string) {
        message.append(string);
    }
    void out(String format, Object... args) {
        message.append(String.format(format, args));
    }

    // Shortcut for begin()/add()/end() all in one:
    void message(String string) {
        begin(); out(string); end();
    }
    void message(String format, Object... args) {
        begin(); out(format, args); end();
    }

    // Redraw using the previous telemetry and field canvas:
    void redraw() {
        begin();
        end();
    }

    // End a UI update and send the results to the Driver Station telemetry, the FTC Dashboard
    // telemetry, and the FTC Dashboard field view:
    void end() {
        // We always redraw both the telemetry and the field even if nothing changed
        // to handle the situation where the user started/restarted FTC Dashboard
        // in the middle of a session:
        if (message.length() != 0) {
            // There's a new message, so use that:
            messageCopy = message.toString();
        }

        if (canvas != null) {
            // There was new canvas rendering, so copy it for posterity and then use it:
            canvasOpsCopy = new ArrayList<>(canvas.getOperations());
        } else {
            // There was no new canvas rendering so re-render the canvas from the previous iteration:
            packet.fieldOverlay().getOperations().addAll(canvasOpsCopy);
        }

        // Send the completed message to the FTC Dashboard telemetry (remembering that it doesn't
        // like newlines) and to the Driver Station (if enabled):
        packet.addLine(messageCopy.replace("\n", "<br>"));
        if (welcomeMessage != null) {
            telemetry.addLine(welcomeMessage);
        } else {
            telemetry.addLine(messageCopy);
        }
        telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Prepare for the next begin():
        message = null;
        packet = null;
        canvas = null;
    }

    // Get a canvas for drawing on the field. Callable only when in a begin/end bracket:
    Canvas canvas() {
        if (canvas == null) {
            // By default, Road Runner draws the field so positive y goes left, positive x
            // goes up. Rotate the field clockwise so that positive positive y goes up, positive x
            // goes right. This rotation is 90 (rather than -90) degrees in page-frame space.
            // Then draw the grid on top and finally set the transform to rotate all subsequent
            // rendering.
            canvas = packet.fieldOverlay();
            if (blankField) {
                canvas.setFill("#000000");
                canvas.fillRect(-72, -72, 144, 144);
            } else {
                canvas.drawImage("/dash/into-the-deep.png", 0, 0, 144, 144,
                        Math.toRadians(90), 0, 144, true);
            }
            canvas.drawGrid(0, 0, 144, 144, 6, 6);

            canvas.setRotation(Math.toRadians(-90));
            // Fade field and grid to white by drawing transparent white over it:
            canvas.setAlpha(0.8);
            canvas.setFill("#ffffff");
            canvas.fillRect(-72, -72, 144, 144);
            canvas.setAlpha(1.0);
        }
        return canvas;
    }

    // Toss any rendering that was done to the canvas:
    void abortCanvas() {
        canvas = null;
    }

    // Erase everything on the field. Can't be in an active begin/end bracket:
    void clearCanvas() {
        begin();
        canvas = packet.fieldOverlay();
        canvas.setFill("#ffffff");
        canvas.fillRect(-72, -72, 144, 144);
        end();
    }

    // Put a sticky FTC Dashboard telemetry value:
    void put(String key, Object value) {
        packet.put(key, value);
    }

    // Put a sticky divider string in the telemetry:
    void putDivider() {
        StringBuilder divider = new StringBuilder();
        for (int i = 0; i < 10; i++)
            //noinspection UnnecessaryUnicodeEscape
            divider.append("\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af");
        packet.put(divider.toString(), "");
    }

    // Get rid of any sticky variables that have accumulated in FTC Dashboard's telemetry:
    void clearDashboardTelemetry() {
        FtcDashboard.getInstance().clearTelemetry();
    }
}

/**
 * Class for remembering all of the tuned settings, stored in the /sdcard folder on the robot.
 * @noinspection AccessStaticViaInstance
 */
class TuneParameters {
    MecanumDrive.Params params;
    boolean passedWheelTest;
    boolean passedTrackingTest;
    boolean passedCompletionTest;

    // Get the settings from the current MecanumDrive object:
    public TuneParameters(MecanumDrive drive) {
        params = drive.PARAMS;
    }

    // Load the passed test state from the saved state:
    public void loadPassedState() {
        TuneParameters savedParameters = getSavedParameters();
        if (savedParameters != null) {
            passedWheelTest = savedParameters.passedWheelTest;
            passedTrackingTest = savedParameters.passedTrackingTest;
            passedCompletionTest = savedParameters.passedCompletionTest;
        }
    }

    // Return a deep-copy clone of the current Settings object:
    public TuneParameters createClone() {
        Gson gson = new Gson();
        return gson.fromJson(gson.toJson(this), TuneParameters.class);
    }

    // Save the current settings to the preferences database:
    public void save() {
        Preferences preferences = Preferences.userNodeForPackage(TuneParameters.class);
        Gson gson = new Gson();
        String json = gson.toJson(this);
        preferences.put("settings", json);
    }

    // Compare the saved and current values for a configuration parameter. If they're different,
    // return a string that tells the user the new value and the old one:
    boolean useHtml = false;
    String comparison = "";
    void compare(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, oldValue);
        String newString = String.format(format, newValue);

        // Never let String.format() round fractional values to 0.0 or 1.0; we want the
        // ceiling in those cases instead. We do this so that we can detect settings values
        // that haven't been tuned yet and are initialized to default values.
        if (((Double.parseDouble(newString) == 0.0) && (newValue != 0.0)) ||
            ((Double.parseDouble(newString) == 1.0) && (newValue != 1.0))) {

            // Change the least significant digit in the string to a "1":
            newString = newString.substring(0, newString.length() - 1) + "1";
        }

        if (!oldString.equals(newString)) {
            comparison += (useHtml) ? "&ensp;" : "    ";
            comparison += String.format("%s = %s; // Was %s\n", parameter, newString, oldString);
        }
    }
    /** @noinspection SameParameterValue*/
    void compareRadians(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, Math.toDegrees(oldValue));
        String newString = String.format(format, Math.toDegrees(newValue));
        if (!oldString.equals(newString)) {
            comparison += (useHtml) ? "&ensp;" : "    ";
            comparison += String.format("%s = Math.toRadians(%s);\n", parameter, newString);
            comparison += (useHtml) ? "&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;" : "                          ";
            comparison += String.format("// Was Math.toRadians(%s)\n", oldString);
        }
    }

    // Validate that the settings are valid and apply to the current robot:
    TuneParameters getSavedParameters() {
        // Load the saved settings from the preferences database:
        Preferences preferences = Preferences.userNodeForPackage(TuneParameters.class);
        Gson gson = new Gson();
        String json = preferences.get("settings", "");
        TuneParameters savedParameters = gson.fromJson(json, TuneParameters.class);

        // Now compare to the current settings:
        if ((savedParameters == null) || (savedParameters.params == null))
            return null; // No saved settings were found
        return savedParameters;
    }

    // Compare the current settings to the last saved settings. Returns a string that
    // describes how to fix the code if there are any mismatches. It may be an empty string.
    public String compare(TuneParameters oldSettings, boolean useHtml) {
        this.useHtml = useHtml;
        this.comparison = "";

        compare("inPerTick", "%.5f", oldSettings.params.inPerTick, params.inPerTick);
        compare("lateralInPerTick", "%.3f", oldSettings.params.lateralInPerTick, params.lateralInPerTick);
        compare("trackWidthTicks", "%.2f", oldSettings.params.trackWidthTicks, params.trackWidthTicks);
        compare("kS", "%.3f", oldSettings.params.kS, params.kS);
        compare("kV", "%.3f", oldSettings.params.kV, params.kV);
        compare("kA", "%.4f", oldSettings.params.kA, params.kA);
        compare("axialGain", "%.2f", oldSettings.params.axialGain, params.axialGain);
        compare("axialVelGain", "%.2f", oldSettings.params.axialVelGain, params.axialVelGain);
        compare("lateralGain", "%.2f", oldSettings.params.lateralGain, params.lateralGain);
        compare("lateralVelGain", "%.2f", oldSettings.params.lateralVelGain, params.lateralVelGain);
        compare("headingGain", "%.2f", oldSettings.params.headingGain, params.headingGain);
        compare("headingVelGain", "%.2f", oldSettings.params.headingVelGain, params.headingVelGain);
        compare("otos.offset.x", "%.3f", oldSettings.params.otos.offset.x, params.otos.offset.x);
        compare("otos.offset.y", "%.3f", oldSettings.params.otos.offset.y, params.otos.offset.y);
        compareRadians("otos.offset.h", "%.2f", oldSettings.params.otos.offset.h, params.otos.offset.h);
        compare("otos.linearScalar", "%.3f", oldSettings.params.otos.linearScalar, params.otos.linearScalar);
        compare("otos.angularScalar", "%.4f", oldSettings.params.otos.angularScalar, params.otos.angularScalar);
        compare("maxWheelVel", "%.2f", oldSettings.params.maxWheelVel, params.maxWheelVel);
        compare("minProfileAccel", "%.2f", oldSettings.params.minProfileAccel, params.minProfileAccel);
        compare("maxProfileAccel", "%.2f", oldSettings.params.maxProfileAccel, params.maxProfileAccel);
        compare("maxAngVel", "%.2f", oldSettings.params.maxAngVel, params.maxAngVel);
        compare("maxAngAccel", "%.2f", oldSettings.params.maxAngAccel, params.maxAngAccel);
        return comparison;
    }
}

/**
 * Class for encapsulating the menu system.
 * @noinspection StringConcatenationInsideStringBufferAppend, UnnecessaryUnicodeEscape, unused
 */
class Menu {
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed
    static private final String DESCRIPTOR_SEPARATOR = "::";
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double REPEAT_DELAY = 0.1; // Seconds after any repeat to repeat again
    private static Menu menu; // Points to our own singleton object
    Io io; // Use this for user input and output
    ArrayList<Widget> menuStack = new ArrayList<>(); // Stack of menus, the last is the current
    int lastInput; // Last quantized input (-1, 0 or 1)
    double nextAdvanceTime; // Time at which to advance the value

    abstract public static class Widget {
        String description;
        boolean isEnabled;
        boolean isStarred;
        Widget(String descriptor) {
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
    private static class MenuWidget extends Widget {
        ArrayList<Widget> widgets = new ArrayList<>(); // List of widgets in this menu
        int current; // Index in widgets that has the UI focus
        public MenuWidget(String descriptor) {
            super(descriptor);
        }
        public String string() {
            return description + "..."; // "\uD83D\uDCC1 is Folder symbol
        }
    }
    private static class ToggleWidget extends Widget {
        boolean value;
        Consumer<Boolean> callback;
        public ToggleWidget(String descriptor, boolean value, Consumer<Boolean> callback) {
            super(descriptor); this.value = value; this.callback = callback;
        }
        public String string() {
            return (value ? "\u2612" : "\u2610") + " " + description; // checked-box, empty box
        }
    }
    private static class ListWidget extends Widget {
        int index;
        String[] list;
        BiConsumer<Integer, String> callback;
        public ListWidget(String descriptor, int index, String[] list, BiConsumer<Integer, String> callback) {
            super(descriptor); this.index = index; this.list = list; this.callback = callback;
        }
        public String string() {
            return "\u2194\uFE0F <b>" + list[index] + "</b>: " + description; // blue left/right arrow
        }
    }
    private static class ActivationWidget extends Widget {
        Function<Boolean, String> callback;
        public ActivationWidget(String descriptor, Function<Boolean, String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() { return "\u2757 " + callback.apply(false); } // red exclamation mark
    }
    private static class StatsWidget extends Widget {
        Supplier<String> callback;
        public StatsWidget(String descriptor, Supplier<String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() { return "\uD83D\uDCCA " + description + "..."; }
    }
    private static class RunWidget extends Widget {
        Runnable runnable;
        public RunWidget(String descriptor, Runnable runnable) {
            super(descriptor);
            this.runnable = runnable;
        }
        public String string() {
            return isEnabled ? description : "<font color='#808080'>" + description + "</font>";
        }
    }

    // Constructor:
    public Menu(Io io) {
        Menu.menu = this;
        this.io = io;
        menuStack.add(new MenuWidget("")); // The root menu
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Update loop for the Gui.
    String update() {
        StringBuilder output = new StringBuilder();

        // Add a header with submenu names:
        output.append("<big><big>");
        if (menuStack.size() <= 1) {
            output.append("Dpad to navigate, "+ LoonyTune.A+" to select");
        } else {
            for (int i = 1; i < menuStack.size(); i++) {
                if (i > 1)
                    output.append("\u00b7");
                output.append(menuStack.get(i).description);
            }
            output.append(", "+LoonyTune.B+" to exit");
        }
        output.append("</big></big><br><small><small><br></small></small>"); // Leave half a line blank

        // Process dpad up and down with auto-repeat and clamping:
        MenuWidget menu = (MenuWidget) menuStack.get(menuStack.size() - 1);
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

        // Now output the widgets:
        for (int i = 0; i < menu.widgets.size(); i++) {
            Widget widget = menu.widgets.get(i);
            if (i != menu.current) {
                String bullet = (widget.isStarred) ? "\u2606" : "\u25c7"; // Hollow star or circle
                output.append(bullet + " " + widget.string() + "\n");
            } else {
                // Highlight current item:
                String bullet = (widget.isStarred) ? "\u2605" : "\u25c6"; // Solid star or circle
                output.append("<span style='background: #88285a'>" + bullet + " " + widget.string() + "</span>\n");
            }
        }

        Widget widget = menu.widgets.get(menu.current);
        if (io.cancel()) {
            if (menuStack.size() > 1) {
                // Pop up the menu stack:
                menuStack.remove(menuStack.size() - 1);
            }
        }
        else if (widget instanceof ToggleWidget) {
            ToggleWidget toggleWidget = (ToggleWidget) widget;
            if (io.ok()) {
                toggleWidget.value = !toggleWidget.value;
                toggleWidget.callback.accept(toggleWidget.value);
            }
        } else if (widget instanceof ListWidget) {
            ListWidget listWidget = (ListWidget) widget;
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
        } else if (widget instanceof ActivationWidget) {
            if (io.ok()) {
                ActivationWidget activationWidget = (ActivationWidget) widget;
                activationWidget.callback.apply(true);
            }
        } else if (widget instanceof StatsWidget) {
            if (io.ok()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof MenuWidget) {
            if (io.ok()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof RunWidget) {
            RunWidget runWidget = (RunWidget) widget;
            if (io.ok()) {
                if (runWidget.isEnabled) {
                    runWidget.runnable.run();
                }
            }
        }

        return output.toString();
    }

    // Add a new widget to the appropriate spot in the menu hierarchy:
    private Widget add(String descriptor, Widget newWidget) {
        MenuWidget menu = (MenuWidget) menuStack.get(0); // Root menu

        // Peel off the hierarchy which is in the form "Vision::Configuration::Setting":
        while (true) {
            int index = descriptor.indexOf(DESCRIPTOR_SEPARATOR);
            if (index == -1)
                break; // ====>

            // Peel off the first menu name from the descriptor:
            String submenuName = descriptor.substring(0, index);
            descriptor = descriptor.substring(index + DESCRIPTOR_SEPARATOR.length());

            // Find or create the submenu:
            MenuWidget submenu = null;
            for (Widget widget : menu.widgets) {
                if ((widget instanceof MenuWidget) && (widget.description.equals(submenuName))) {
                    submenu = (MenuWidget) widget;
                    break;
                }
            }
            if (submenu == null) {
                submenu = new MenuWidget(submenuName);
                menu.widgets.add(submenu);
            }
            // Descend into that submenu:
            menu = submenu;
        }
        menu.widgets.add(newWidget);
        return newWidget;
    }

    // Add a toggleable widget to the menu:
    public static Widget addToggle(String descriptor, boolean initialValue, Consumer<Boolean> callback) {
        callback.accept(initialValue);
        return menu.add(descriptor, new ToggleWidget(descriptor, initialValue, callback));
    }
    // Add a list widget to the menu:
    public static Widget addList(String descriptor, String[] list, int initialIndex, BiConsumer<Integer, String> callback) {
        callback.accept(initialIndex, list[initialIndex]);
        return menu.add(descriptor, new ListWidget(descriptor, initialIndex, list, callback));
    }
    // Add a widget that can only be activated:
    public static Widget addActivation(String descriptor, Function<Boolean, String> callback) {
        callback.apply(true);
        return menu.add(descriptor, new ActivationWidget(descriptor, callback));
    }
    public static Widget addStats(String descriptor, Supplier<String> callback) {
        return menu.add(descriptor, new StatsWidget(descriptor, callback));
    }
    // Add a widget that can be run:
    public static Widget addRunnable(String descriptor, Runnable callback) {
        return menu.add(descriptor, new RunWidget(descriptor, callback));
    }
}

/**
 * Loony Tune's opMode class for running the tuning tests.
 *
 * @noinspection UnnecessaryUnicodeEscape, AccessStaticViaInstance, ClassEscapesDefinedScope
 */
@SuppressLint("DefaultLocale")
@TeleOp(name="Loony Tune", group="Tuning")
public class LoonyTune extends LinearOpMode {
    static String buttonString(String button) {
        return String.format("<span style='background:#808080'>%s</span>", button);
    }
    static final String A = "\ud83c\udd50"; // Symbol for the gamepad A button
    static final String B = "\ud83c\udd51"; // Symbol for the gamepad B button
    static final String X = "\ud83c\udd67"; // Symbol for the gamepad X button
    static final String Y = "\ud83c\udd68"; // Symbol for the gamepad Y button
    static final String BUMPER = buttonString("bumper");
    static final String TRIGGERS = buttonString("triggers");
    static final String LEFT_TRIGGER = buttonString("LT");
    static final String RIGHT_TRIGGER = buttonString("RT");
    static final String LEFT_BUMPER = buttonString("LB");
    static final String RIGHT_BUMPER = buttonString("RB");
    static final String DPAD_LEFT_RIGHT = buttonString("DPAD \u2194");
    static final String DPAD_UP_DOWN = buttonString("DPAD \u2195");
    static final String START = buttonString("\u25B6 START");

    // Types of interactive PiD tuners:
    enum PidTunerType { AXIAL, LATERAL, HEADING, ALL }

    // Menu widgets for each of the tuners:
    enum Tuner {
        NONE(0),

        WHEEL_TEST(1),
        PUSH(2),
        SPIN(3),
        TRACKING_TEST(4),
        ACCELERATING(5),
        FEED_FORWARD(6),
        LATERAL_MULTIPLIER(7),
        AXIAL_GAIN(8),
        LATERAL_GAIN(9),
        HEADING_GAIN(10),
        COMPLETION_TEST(11),
        RETUNE(12),

        COUNT(13); // Count of tuners

        final int index;
        Tuner(int index) { this.index = index; }
    }

    Menu.Widget[] widgets = new Menu.Widget[Tuner.COUNT.index];

    // Member fields referenced by every test:
    Io io;
    Menu menu;
    Poll poll;
    Dialog dialog;
    MecanumDrive drive;
    TuneParameters currentParameters;
    TuneParameters originalParameters;
    int nextRetuneIndex = Tuner.COUNT.index; // Next tuner to run when re-tuning

    // Tests and tuners:
    PushTuner pushTuner = new PushTuner();
    SpinTuner spinTuner = new SpinTuner();
    AcceleratingStraightLineTuner acceleratingTuner = new AcceleratingStraightLineTuner();
    InteractiveFeedForwardTuner feedForwardTuner = new InteractiveFeedForwardTuner();
    LateralMultiplierTuner lateralMultiplierTuner = new LateralMultiplierTuner();
    InteractivePidTuner pidTuner = new InteractivePidTuner();

    // Constants:
    final Pose2d zeroPose = new Pose2d(0, 0, 0);

    // Add tuners to the menu:
    void addTuner(Tuner tuner, Runnable runnable, String description) {
        widgets[tuner.index] = menu.addRunnable(description, runnable);
    }

    // Update which tuners need to be disabled and which need to be run:
    void updateTunerDependencies(Tuner completedTuner) {

        MecanumDrive.Params params = currentParameters.params;
        MecanumDrive.Params.Otos otos = params.otos;

        Tuner firstFailure;
        if (!currentParameters.passedWheelTest)
            firstFailure = Tuner.WHEEL_TEST;
        else if (otos.linearScalar == 0 || otos.offset.h == 0)
            firstFailure = Tuner.PUSH;
        else if (params.trackWidthTicks == 0 || otos.angularScalar == 0 ||otos.offset.x == 0 || otos.offset.y == 0)
            firstFailure = Tuner.SPIN;
        else if (!currentParameters.passedTrackingTest)
            firstFailure = Tuner.TRACKING_TEST;
        else if (params.kS == 0 || params.kV == 0)
            firstFailure = Tuner.ACCELERATING;
        else if (params.kA == 0)
            firstFailure = Tuner.FEED_FORWARD;
        else if (params.lateralInPerTick == 0 || params.lateralInPerTick == 1)
            firstFailure = Tuner.LATERAL_MULTIPLIER;
        else if (params.axialGain == 0)
            firstFailure = Tuner.AXIAL_GAIN;
        else if (params.lateralGain == 0)
            firstFailure = Tuner.LATERAL_GAIN;
        else if (params.headingGain == 0)
            firstFailure = Tuner.HEADING_GAIN;
        else if (currentParameters.passedCompletionTest)
            firstFailure = Tuner.COMPLETION_TEST;
        else
            firstFailure = Tuner.COUNT; // Don't star the completion test if it already passed

        // When retuning, advance one-at-a-time based on the furthest tuners completed to now:
        nextRetuneIndex = Math.max(nextRetuneIndex, completedTuner.index + 1);

        // Mark the next tuner to be run:
        int nextIndex = Math.min(firstFailure.index, nextRetuneIndex);
        for (int i = 1; i < Tuner.COUNT.index; i++) { // 0 is reserved for 'NONE'
            widgets[i].isEnabled = (i <= nextIndex);
            widgets[i].isStarred = (i == nextIndex);
        }
    }

    // Check if the robot code setting the MecanumDrive configuration parameters is up to date
    // with the last results from tuning:
    /** @noinspection BusyWait*/
    static public void verifyCodeMatchesTuneResults(MecanumDrive drive, Telemetry telemetry, Gamepad gamepad) {
        // There's no point in complaining about mismatches when running under the simulator:
        if (WilyWorks.isSimulating)
            return; // ====>

        TuneParameters currentSettings = new TuneParameters(drive);
        TuneParameters savedSettings = currentSettings.getSavedParameters();
        if (savedSettings != null) {
            String comparison = savedSettings.compare(currentSettings, false);
            if (!comparison.isEmpty()) {
                // There is no way to query the current display format so we have to assume it
                // could be either HTML or non-HTML.
                telemetry.clear();
                telemetry.addLine("YOUR CODE IS OUT OF SYNC WITH LOONY TUNE");
                telemetry.addLine();
                telemetry.addLine("The code's configuration parameters don't match the last "
                        + "results saved in Loony Tune. To use the Loony Tune results, double-tap the shift key in Android "
                        + "Studio, enter 'md.params' to jump to the MecanumDrive Params constructor, "
                        + "then update as follows:");
                telemetry.addLine();
                telemetry.addLine(comparison);
                telemetry.addLine("Please update your code and restart now. Or, to proceed anyway and "
                        + "delete the Loony Tune results, triple-tap the BACK button on the gamepad.");
                telemetry.update();

                // Wait for a triple-tap of the button:
                for (int i = 0; i < 3; i++) {
                    try {
                        while (!gamepad.back)
                            Thread.sleep(1);
                        while (gamepad.back)
                            Thread.sleep(1);
                    } catch (InterruptedException e) {
                        return; // Don't save if STOP has been pressed
                    }
                }

                // If we reached this point, the user has chosen to ignore the last tuning results.
                // Override those results with the current settings:
                currentSettings.save();
                telemetry.addLine("Loony Tune results have been overridden");
                telemetry.update();
            }
        }
    }

    /**
     * Class for doing an animated preview of a Road Runner trajectory action.
     */
    static class TrajectoryPreviewer {
        Io io;
        final double PAUSE_TIME = 1.0; // Pause this many seconds between preview cycles
        Action sequentialAction;
        List<Action> actions = new ArrayList<>();
        Iterator<Action> actionIterator;
        Action currentAction = null; // Current action in the actions list
        double startTime = time();
        TrajectoryPreviewer(Io io, Action action) {
            this.io = io;
            this.sequentialAction = action;
            if (action instanceof SequentialAction) {
                SequentialAction sequential = (SequentialAction) action;

                // Get the SequentialAction 'actions' field even though it's marked private:
                try {
                    Field field = SequentialAction.class.getDeclaredField("actions");
                    field.setAccessible(true);
                    //noinspection unchecked
                    actions = (List<Action>) field.get(sequential);
                } catch (NoSuchFieldException|IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
                if (actions != null) {
                    actionIterator = actions.iterator();
                    currentAction = actionIterator.next();
                }
            }
        }

        // Get a trajectory action's target pose at the current time. Return null if past the
        // the end of the entire sequential action's duration:
        Pose2d getTargetPose() {
            while (true) {
                double deltaT = time() - startTime;
                if (currentAction instanceof MecanumDrive.TurnAction) {
                    TimeTurn turn = ((MecanumDrive.TurnAction) currentAction).turn;
                    if (deltaT > turn.duration) {
                        if (!actionIterator.hasNext()) {
                            actionIterator = actions.iterator();
                            currentAction = actionIterator.next();
                            return null;
                        }
                        currentAction = actionIterator.next();
                        startTime = time();
                        continue; // ====>
                    }
                    Pose2dDual<Time> txWorldTarget = turn.get(deltaT);
                    return txWorldTarget.value();
                } else if (currentAction instanceof MecanumDrive.FollowTrajectoryAction) {
                    TimeTrajectory timeTrajectory = ((MecanumDrive.FollowTrajectoryAction) currentAction).timeTrajectory;
                    if (deltaT > timeTrajectory.duration) {
                        if (!actionIterator.hasNext()) {
                            actionIterator = actions.iterator();
                            currentAction = actionIterator.next();
                            return null;
                        }
                        currentAction = actionIterator.next();
                        startTime = time();
                        continue; // ===>
                    }
                    Pose2dDual<Time> txWorldTarget = timeTrajectory.get(deltaT);
                    return txWorldTarget.value();
                } else {
                    return null;
                }
            }
        }

        // Update the preview. Must be in an active io.begin/end bracket.
        void update() {
            Canvas canvas = io.canvas();
            canvas.fillText("Preview", -19, 5, "", 0, false);
            sequentialAction.preview(canvas);
            canvas.setStroke("#000000"); // Black
            double time = time();
            if (time > startTime) {
                Pose2d pose = getTargetPose();
                if (pose == null)
                    startTime = time + PAUSE_TIME; // Start new cycle after a delay
                else
                    Drawing.drawRobot(canvas, pose);
            }
        }
    }

    /**
     * Class that encapsulates user queries.
     */
    class Poll {
        // Show a message, drive the robot, and wait for either an A/B button press, or an
        // A/X button press. If accept is pressed, return success. If the other button is
        // pressed, return failure. The robot CAN be driven while waiting.
        boolean okCancelWithDriving() {
            boolean success = true;
            while (!io.ok()) {
                io.redraw();
                if (isStopRequested()) {
                    success = false;
                    break;
                }
                if (io.cancel()) {
                    success = false;
                    break;
                }
                updateGamepadDriving();
                spinTuner.updateRotation();
            }
            stopMotors();
            drive.setPose(zeroPose); // Reset the pose once they stopped
            return success;
        }

        boolean startCancelWithDriving() { // @@@ Needs work (and a better name)
            boolean success = true;
            while (!io.start()) {
                io.redraw();
                if (isStopRequested()) {
                    success = false;
                    break;
                }
                if (io.cancel()) {
                    success = false;
                    break;
                }
                updateGamepadDriving();
                spinTuner.updateRotation();
            }
            stopMotors();
            drive.setPose(zeroPose); // Reset the pose once they stopped
            return success;
        }

        // Wait for an A button press:
        void ok() {
            while (!isStopRequested() && !io.ok())
                io.redraw();
        }

        // Return 'true' if A was pressed, 'false' otherwise:
        boolean okCancel() {
            while (!isStopRequested() && !io.cancel()) {
                if (io.ok())
                    return true;
            }
            return false;
        }
    }

    /**
     * Class for dialog boxes.
     * @noinspection unused
     */
    class Dialog {
        static final String QUESTION_ICON = "<big>\u2753</big>";
        static final String INFORMATION_ICON = "<big><big>\ud83d\udec8</big></big>";
        static final String CRITICAL_ICON = "<big>\u274c</big>";
        static final String WARNING_ICON = "<big>\u26a0\ufe0f</big>";

        // Returns true if the user is sure, false to cancel:
        boolean areYouSure() {
            while (!isStopRequested()) {
                io.message(WARNING_ICON + " Are you sure you want to discard your results?\n"
                    + "\n"
                    + A + " to discard, " + B + " to cancel.");
                if (io.ok())
                    return true;
                if (io.cancel())
                    return false;
            }
            return true;
        }
    }

    // Return a string that represents the distance the test will run:
    String testDistance(int distance) {
        return String.format("%d inches (%.1f tiles)", distance, distance / 24.0);
    }

    // Return a string that represents the amount of clearance needed:
    /** @noinspection SameParameterValue*/
    String clearanceDistance(int distance) {
        return String.format("%.0f tiles", Math.ceil(distance / 24.0));
    }

    // Run an Action but end it early if Cancel is pressed.
    // Returns True if it ran without cancelling, False if it was cancelled.
    private boolean runCancelableAction(String header, Action action) {
        drive.runParallel(action);
        while (opModeIsActive() && !io.cancel()) {
            io.begin();
            io.out(header + "Press "+B+" to cancel and stop the robot.");
            boolean more = drive.doActionsWork(io.packet);
            if (!more) {
                // We successfully completed the Action! Abort the current canvas:
                io.abortCanvas();
                io.end();
                return true; // ====>
            }
            io.end();
        }
        // The user either pressed Cancel or End:
        drive.abortActions();
        stopMotors();
        return false;
    }

    // Road Runner expects the hardware to be in different states when using high-level MecanumDrive
    // functionality vs. its lower-level tuning functionality.
    private void configureToDrive(boolean enableRoadRunnerDefaults) {
        io.clearDashboardTelemetry();

        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        if (enableRoadRunnerDefaults) {
            // Initialize hardware state the same way that MecanumDrive does:
            for (DcMotorEx motor: motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } else {
            // Initialize hardware state in the way that FTC defaults to:
            for (DcMotorEx motor: motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }
        }
    }

    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValueD) {
        float stickValue = (float) stickValueD;
        // Make slow driving easier on the real robot. Don't bother under Wily Works because
        // then it's too slow:
        float power = WilyWorks.isSimulating ? 1.0f : 2.0f;
        float result = Math.signum(stickValue) * Math.abs((float) Math.pow(stickValue, power));

        // Output spew when weird compiler bug hits because result should never be more than
        // stickValue:
        if (Math.abs(result) > Math.abs(stickValue))
            out.printf("LooneyTuner: raw stick: %.2f, shaped: %.2f, power: %.2f, signum: %.2f\n", stickValue, result, power, Math.signum(stickValue));
        return result;
    }

    // Poll the gamepad input and set the drive motor power accordingly:
    public void updateGamepadDriving() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
            shapeStick(-gamepad1.left_stick_y),
            shapeStick(-gamepad1.left_stick_x)),
            shapeStick(-gamepad1.right_stick_x)));
    }

    // Prompt the user for how to set the new parameters and save them to the registry:
    public void acceptParameters(TuneParameters newParameters) {
        String comparison = newParameters.compare(currentParameters, true);
        if (comparison.isEmpty()) {
            io.message("The new results match your current settings.\n\nPress "+A+" to continue.");
            poll.ok();
        } else {
            MecanumDrive.PARAMS = newParameters.params;
            currentParameters = newParameters;
            currentParameters.save();
            io.message("Double-tap the shift key in Android Studio, enter '<b>md.params</b>' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
            poll.ok();
        }
    }

    // Set the hardware to the current parameters:
    public void setOtosHardware() {
        drive.initializeOpticalTracker();
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Normalize a radians angle to (-pi, pi]:
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    // Stop all motors:
    void stopMotors() { stopMotors(true); }
    void stopMotors(boolean slowAndSure) {
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        if (slowAndSure)
            sleep(333); // Give the robot a little time to actually stop
    }


    // Draw the points to the graph on the FTC Dashboard field:
    void strokeSamples(Canvas canvas, Point scale, List<Point> points) {
        double[] xPoints = new double[points.size()];
        double[] yPoints = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            xPoints[i] = points.get(i).x * scale.x;
            yPoints[i] = points.get(i).y * scale.y;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    // Helper for managing the various screens of the tests.
    class Screen {
        public final String[] screens; // Name for each screen
        public int index; // Currently active screen
        public String header = ""; // Header to be drawn at the top of the screen
        public String buttons = ""; // Buttons message describing what the bumper buttons do
        Runnable runnable; // Runnable to execute when the user switches screens

        Screen(String[] screens) {
            this.screens = screens;
        }

        // Handle all of the loop bookkeeping for screens controlled by the bumpers. Check the
        // gamepad input and update the status string as well as the buttons string:
        void update() {
            int oldIndex = index;
            if (io.leftBumper()) {
                index = Math.max(index - 1, 0);
            }
            if (io.rightBumper()) {
                index = Math.min(index + 1, screens.length - 1);
            }
            if (index != oldIndex) {
                if (runnable != null)
                    runnable.run();
                stopMotors(false);
                drive.setPose(zeroPose);
                io.clearDashboardTelemetry();
            }
            header = String.format("<span style='background:#88285a'>Screen %d of %d: %s</span>\n\n", index + 1, screens.length, screens[index]);
            if (index == 0) {
                buttons = RIGHT_BUMPER+" (the bumper button above the right trigger) to go to the "
                    + "next screen";
            } else if (index == screens.length - 1) {
                buttons = LEFT_BUMPER+" for the previous screen";
            } else {
                buttons = RIGHT_BUMPER+"/"+LEFT_BUMPER+" for the next/previous screen";
            }
        }

        // Register a callback to be executed when the user switches screens:
        void onSwitch(Runnable runnable) { this.runnable = runnable; }
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
        double value; // Current value
        int lastInput; // Last quantized input (-1, 0 or 1)
        double nextAdvanceTime; // Time at which to advance the value

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
                //noinspection DataFlowIssue
                value = (double) field.get(object);
            } catch (IllegalAccessException|NoSuchFieldException|NullPointerException e) {
                throw new RuntimeException(e);
            }
        }

        // Update the variable according to the latest gamepad input.
        String update() {
            if (io.left()) {
                digit = Math.min(digit + 1, 2);
            }
            if (io.right()) {
                digit = Math.max(digit - 1, -decimalDigits);
            }
            if ((digit > 0) && (Math.pow(10, digit) > Math.abs(value))) {
                digit--;
            }

            // Advance the value according to the thumb stick state:
            int input = io.gamepad.dpad_up ? 1 : (io.gamepad.dpad_down ? -1 : 0); // -1, 0 or 1
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

            // Set the value into the class:
            try {
                field.setDouble(object, value);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }

            // Show the new value:
            String showValue = String.format(showFormat, value);
            int digitOffset = (digit >= 0) ? -digit - 1 : -digit;
            int digitIndex = showValue.indexOf(".") + digitOffset;
            digitIndex = Math.max(0, Math.min(digitIndex, showValue.length() - 1));
            String prefix = showValue.substring(0, digitIndex);
            String middle = showValue.substring(digitIndex, digitIndex + 1);
            String suffix = showValue.substring(digitIndex + 1);

            // Highlight the focus digit:
            middle = "<span style='background: #88285a'>" + middle + "</span>";

            // Blink the underline every half second:
            if ((((int) (time() * 2)) & 1) != 0) {
                middle = "<u>" + middle + "</u>";
            }

            return prefix + middle + suffix;
        }
    }

    // Run a simple trajectory with a preview and an option to disable odometry. The message
    // can be null:
    void runTrajectory(Action action) { runTrajectory(action, null);}
    void runTrajectory(Action action, String promptMessage) {
        configureToDrive(true); // Do use MecanumDrive
        TrajectoryPreviewer preview = new TrajectoryPreviewer(io, action);

        boolean useOdometry = true;
        while (opModeIsActive() && !io.cancel()) {
            String message = promptMessage;
            if (message == null)
                message = "The robot will run the trajectory shown in FTC Dashboard.";
            if (useOdometry)
                message += "\n\nRunning with normal odometry correction. Press "+BUMPER+" to "
                        + "disable so as to test how well all the non-odometry settings are tuned. "
                        + "If well tuned, the robot should drive close to the intended path. ";
            else
                message += "\n\nRunning <b>without</b> odometry correction. Press "+BUMPER+" to re-enable.";

            io.message(message + "\n\nPress "+A+" to start, "+B+" to cancel, "+BUMPER+" to toggle odometry.");
            updateGamepadDriving();
            io.begin();
            preview.update();
            io.end();

            if (io.leftBumper() || io.rightBumper()) {
                useOdometry = !useOdometry;
            }
            if (io.ok()) {
                stopMotors();
                drive.setPose(zeroPose);
                if (useOdometry) {
                    runCancelableAction("", action);
                } else {
                    // Point MecanumDrive to some temporary parameters, run the action, then
                    // restore the settings:
                    TuneParameters testParameters = currentParameters.createClone();
                    testParameters.params.axialGain = 0;
                    testParameters.params.axialVelGain = 0;
                    testParameters.params.lateralGain = 0;
                    testParameters.params.lateralVelGain = 0;
                    testParameters.params.headingGain = 0;
                    testParameters.params.headingVelGain = 0;
                    MecanumDrive.PARAMS = testParameters.params;
                    runCancelableAction("", action);
                    MecanumDrive.PARAMS = currentParameters.params;
                }
                break; // ====>
            }
        }
        stopMotors();
        drive.setPose(zeroPose); // Reset the pose once they stopped
    }

    // All tuner results are derived from this Result class:
    abstract static class Result {
        // Convert the result to a one-line descriptive string:
        abstract public String getString();

        // Apply the results to the collection of tuned parameters:
        abstract public void applyTo(TuneParameters parameters);

        // Show a history of results. The first result will be tagged as the original, and the
        // final result will be bolded and tagged as the newest.
        static void showHistory(Io io, List<Result> history) {
            // The first entry is always the original settings:
            if (history.size() > 1) {
                io.out("Result history:\n\n");
                for (int i = 0; i < history.size(); i++) {
                    Result result = history.get(i);
                    io.out("&emsp;%s", result.getString());
                    if (i == 0) {
                        io.out(" (starting)");
                    } else if (i == history.size() - 1) {
                        io.out(" (newest)");
                    }
                    io.out("\n");
                }
                io.out("\n");
                io.out("If you've done multiple measurements and they're consistent, go "
                        + "to the next screen.\n");
                io.out("\n");
            }
        }
    }

    // Returns true if the tuning is done, false if it should continue.
    boolean resultsDoneScreen(List<Result> history, Screen screen, Tuner tunerType) {
        Result finalResult = history.get(history.size() - 1);
        TuneParameters newParameters = currentParameters.createClone();
        finalResult.applyTo(newParameters);
        String comparison = newParameters.compare(currentParameters, true);
        if ((history.size() <= 1) || (comparison.isEmpty())) {
            io.out(A + " to exit, " + screen.buttons);
            io.end();
            if (io.ok())
                return true; // ====>
        } else {
            io.out("Resulting configuration changes:\n\n" + comparison + "\n");
            io.out(A + " to save these results, " + B + " to discard and exit, ");
            io.out(screen.buttons + ".");
            io.end();

            if (io.ok()) {
                acceptParameters(newParameters);
                updateTunerDependencies(tunerType);
                return true; // ====>
            }
        }
        if (io.cancel()) {
            if (comparison.isEmpty())
                return true; // ====>

            return dialog.areYouSure(); // ====>
        }
        return false;
    }

    //**********************************************************************************************

    // Test the wheels on the robot by driving around and testing each wheel individually.
    void wheelTest() {
        configureToDrive(true); // Do use MecanumDrive

        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        String[] motorNames = { "leftFront", "leftBack", "rightBack", "rightFront" };
        Screen screen = new Screen(new String[] {
                "Test 'leftFront' wheel",       // 0
                "Test 'leftBack' wheel",        // 1
                "Test 'rightBack' wheel",       // 2
                "Test 'rightFront' wheel",      // 3
                "Test all wheels by driving",   // 4
                "Save and exit" });             // 5

        while (opModeIsActive()) {
            screen.update();
            io.begin();
            io.out(screen.header);
            if (screen.index < 4) { // Individual wheel screen
                int motor = screen.index;
                double power = gamepad1.right_trigger - gamepad1.left_trigger;
                motors[motor].setPower(power);
                DcMotorSimple.Direction direction =  motors[motor].getDirection();
                String motorName = motorNames[motor];

                io.out("Use the triggers to power this wheel:\n\n");
                io.out("&emsp;<b>%s.setPower(%.2f)</b>", motorName, power);
                io.out("\n\n"
                        + "If this wheel turns in the wrong direction, double-tap the shift "
                        + "key in Android Studio, enter 'md.configure', and ");
                if (direction == DcMotorSimple.Direction.FORWARD) {
                    io.out("add a call to <b>'%s.setDirection(DcMotorEx.Direction.REVERSE)'</b>.", motorName);
                } else {
                    io.out("disable the line <b>'%s.setDirection(DcMotorEx.Direction.REVERSE);'</b>.", motorName);
                }
                io.out("\n\nPress "+RIGHT_TRIGGER+" to rotate wheel forward, "+LEFT_TRIGGER+" for reverse, "+screen.buttons + ".");
                io.end();
            } else if (screen.index == 4) { // All wheels screen
                updateGamepadDriving();
                io.out("Test the robot by driving it around. Left stick controls movement, "
                        + "right stick controls rotation."
                        + "\n\n"
                        + "If it strafes in the wrong direction (e.g., left when you want right), or "
                        + "if it drives weird even though it passed the previous steps, the Mecanum "
                        + "wheels may be mounted wrong. When looking from above, the direction "
                        + "of the rollers on the 4 wheels should form an 'X'.");
                io.out("\n\nSticks to drive, " + screen.buttons + ".");
                io.end();
            } else if (screen.index == 5) { // Exit screen
                io.out("Press "+A+" if everything passed, "+B+" if there was a failure, or "+screen.buttons +".");
                io.end();
                if (io.ok()) {
                    currentParameters.passedWheelTest = true;
                    currentParameters.save();
                    updateTunerDependencies(Tuner.WHEEL_TEST);
                    break; // ====>
                }
                if (io.cancel()) {
                    currentParameters.passedWheelTest = false;
                    currentParameters.save();
                    updateTunerDependencies(Tuner.WHEEL_TEST);
                    break; // ====>
                }
            }
        }
    }

    // Test tracking on the robot by driving around.
    void trackingTest() {
        // We dedicate an unobstructed corner of the field to use as the home position for this
        // test. The robot can nestle in the corner and thereby establish a consistent physical
        // location and orientation for measuring tracking drift error. We choose the lower-right
        // corner, and assume that the robot is 18 inches on each side:
        Pose2d homePose = new Pose2d(72 - 9, -72 + 9, 0);

        configureToDrive(true); // Do use MecanumDrive

        drive.setPose(zeroPose);
        Pose2d previousPose = zeroPose; // Robot's pose on the previous iteration, for measuring deltas
        double totalDistance = 0; // Inches
        double totalRotation = 0; // Radians
        String lastSeenStatus = ""; // Most recently reported non-zero status from the OTOS
        double lastSeenTime = 0; // Time at which lastSeenStatus was set

        double baselineImu = 0; // IMU heading when the baseline was set
        Pose2d baselinePose = null; // Position where the baseline was set
        double maxLinearSpeed = 0; // Max linear speed seen, inches/s
        double maxLinearAcceleration = 0; // Max linear acceleration seen, inches/s/s
        double maxRotationalSpeed = 0; // Max rotational speed seen, radians/s
        double maxRotationalAcceleration = 0; // Max rotation acceleration seen, radians/s/s

        Screen screen = new Screen(new String[]{"Free drive", "Measure error", "Detail", "Save and exit"});
        while (opModeIsActive()) {
            screen.update();
            io.begin();
            io.out(screen.header);

            // Update our guess for the robot location:
            PoseVelocity2d velocity = drive.updatePoseEstimate();

            Pose2D otosPosition = new Pose2D(0, 0, 0);
            Pose2D otosVelocity = new Pose2D(0, 0, 0);
            Pose2D otosAcceleration = new Pose2D(0, 0, 0);

            // Calling getPosVelAcc() again is expensive performance-wise so don't do it when
            // taking measurements:
            if (screen.index != 1)
                drive.opticalTracker.getPosVelAcc(otosPosition, otosVelocity, otosAcceleration);

            // Query the OTOS for any problems:
            SparkFunOTOS.Status status = drive.opticalTracker.getStatus();
            String currentStatus = "";
            if (status.errorLsm)
                currentStatus += "errorLsm ";
            if (status.errorPaa)
                currentStatus += "errorPaa ";
            if (status.warnOpticalTracking)
                currentStatus += "warnOpticalTracking ";
            if (status.warnTiltAngle)
                currentStatus += "warnTileAngle ";
            if (!currentStatus.isEmpty()) {
                lastSeenStatus = currentStatus;
                lastSeenTime = time();
            }

            // Calculate some statistics about the pose results:
            double linearSpeed = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);
            double rotationalSpeed = Math.abs(velocity.angVel);
            maxLinearSpeed = Math.max(maxLinearSpeed, linearSpeed);
            maxRotationalSpeed = Math.max(maxRotationalSpeed, rotationalSpeed);

            double linearAcceleration = Math.hypot(otosAcceleration.x, otosAcceleration.y);
            double rotationalAcceleration = Math.abs(otosAcceleration.h);
            maxLinearAcceleration = Math.max(maxLinearAcceleration, linearAcceleration);
            maxRotationalAcceleration = Math.max(maxRotationalAcceleration, rotationalAcceleration);

            // Given that updatePoseEstimate was called, get the new pose and track the distance
            // traveled:
            Pose2d pose = drive.pose;
            totalDistance += Math.hypot(
                    pose.position.x - previousPose.position.x,
                    pose.position.y - previousPose.position.y);
            totalRotation += Math.abs(pose.heading.toDouble() - previousPose.heading.toDouble());
            previousPose = pose;

            if (screen.index == 0) { // Free drive screen
                updateGamepadDriving();

                io.setBlankField(true); // Draw only a blank field
                Canvas canvas = io.canvas();
                canvas.setStroke("#3F51B5");
                Drawing.drawRobot(canvas, pose);

                io.out("At this point, pose estimation should fully work. Drive around and "
                        + "look for two things:\n"
                        + "\n"
                        + "\u2022 Does the robot shown in FTC Dashboard correctly track "
                        + "the actual movement? (The next screen will do exact error measurements "
                        + "so just look for approximate correctness here.)\n"
                        + "\u2022 When the robot rotates in place using only the right stick, the "
                        + "on-screen robot shouldn't move its <i>(x, y)</i> position at all. Does it?\n"
                        + "\n");
                io.out("Pose: (%.2f\", %.2f\"), %.2f\u00b0\n\n",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
                io.out("Sticks to drive, " + screen.buttons);
                io.end();
            } else if (screen.index == 1) { // Measure error screen
                updateGamepadDriving();

                io.setBlankField(false); // Draw the actual field
                Canvas canvas = io.canvas();
                canvas.setStroke("#ffd700"); // Gold
                Drawing.drawRobot(canvas, homePose);

                if (baselinePose == null) {
                    io.out("To start measuring error, press "+X+" when the robot is physically "
                            + "positioned in the corner as indicated by the golden pose on FTC "
                            + "Dashboard.\n\n");
                    io.out("Press "+X+" to set the golden pose, "+screen.buttons+".");
                } else {
                    canvas.setStroke("#3F51B5"); // Blueish
                    Drawing.drawRobot(canvas, pose); // Pose estimation

                    io.out("Drive around and then physically return the robot back to the "
                            + "golden pose corner. The following error measurements are valid <i><b>only</b></i> "
                            + "when the robot is back in exact same physical location:\n\n");

                    double dx = pose.position.x - baselinePose.position.x;
                    double dy = pose.position.y - baselinePose.position.y;
                    double otosTheta = normalizeAngle(pose.heading.toDouble() - baselinePose.heading.toDouble());
                    double imuTheta = normalizeAngle(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - baselineImu);

                    io.out("&ensp;OTOS error: (%.2f\", %.2f\"), %.2f\u00b0\n", dx, dy, Math.toDegrees(otosTheta));
                    io.out("&ensp;Control Hub IMU error (for comparison): %.2f\u00b0\n", Math.toDegrees(imuTheta));

                    if ((totalDistance != 0) && (totalRotation != 0)) {
                        double distanceError = Math.abs(dx) / totalDistance;
                        double rotationError = Math.abs(Math.toDegrees(otosTheta)) / totalDistance;
                        io.out("&ensp;Error: %.2f%% (distance), %.3f\u00b0/inch (rotational)",
                                distanceError * 100, rotationError);
                    }

                    io.out("\nPress "+X+" to restart when back at the golden pose, "
                        + Y + " to forget the golden pose, " + screen.buttons + ".");
                }
                io.end();

                if (io.x()) { // Set baseline at home
                    drive.setPose(homePose);
                    previousPose = homePose;
                    baselineImu = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    baselinePose = drive.pose;
                    totalDistance = 0;
                    totalRotation = 0;
                    maxLinearSpeed = 0;
                    maxRotationalSpeed = 0;
                }
                if (io.y()) { // Clear the baseline
                    baselinePose = null;
                }

            } else if (screen.index == 2) { // Detail screen
                io.setBlankField(true); // Draw only a blank field
                updateGamepadDriving();

                io.put("Linear speed", linearSpeed);
                io.put("Rotational speed", Math.toDegrees(rotationalSpeed));
                io.put("Linear acceleration", linearAcceleration);
                io.put("Rotational acceleration", Math.toDegrees(rotationalAcceleration));
                io.putDivider();

                if (currentStatus.isEmpty()) {
                    if (lastSeenStatus.isEmpty())
                        io.out("OTOS status: Good!\n");
                    else {
                        double minutesAgo = (time() - lastSeenTime) / 60.0;
                        io.out("OTOS status: Was '%s' %.1f minutes ago\n", lastSeenStatus, minutesAgo);
                    }
                } else {
                    io.out("OTOS status: %s\n", currentStatus);
                }

                io.out("Max velocities: %.1f\"/s, %.0f\u00b0/s\n",
                        maxLinearSpeed, Math.toDegrees(maxRotationalSpeed));
                io.out("Max accelerations: %.2f\"/s/s, %.1f\u00b0/s/s\n",
                    maxLinearAcceleration, Math.toDegrees(maxRotationalAcceleration));

                io.out("\nPress " + screen.buttons + ".");
                io.end();

            } else if (screen.index == 3) { // Exit screen
                io.out("Press "+A+" if everything passed, "+B+" if there was a failure, or "+screen.buttons +".");
                io.end();
                if (io.ok()) {
                    currentParameters.passedTrackingTest = true;
                    currentParameters.save();
                    updateTunerDependencies(Tuner.TRACKING_TEST);
                    break; // ====>
                }
                if (io.cancel()) {
                    currentParameters.passedTrackingTest = false;
                    currentParameters.save();
                    updateTunerDependencies(Tuner.TRACKING_TEST);
                    break; // ====>
                }
            }
        }
        io.setBlankField(true);
    }

    /**
     * Class to encapsulate all push-tuner logic.
      */
    class PushTuner {
        // Structure used to encapsulate a result from push tuning:
        class PushResult extends Result {
            double linearScalar;
            double offsetHeading;

            public PushResult(double linearScalar, double offsetHeading) {
                this.linearScalar = linearScalar;
                this.offsetHeading = offsetHeading;
            }

            @Override
            public String getString() {
                return String.format("linearScalar: %.3f, offset heading: %.2f",
                        linearScalar, Math.toDegrees(offsetHeading));
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.otos.linearScalar = linearScalar;
                parameters.params.otos.offset.h = offsetHeading;
            }
        }

        // Do the measuring work for pushTuner. Returns the result once done; will be null if there
        // was an apparent error.
        /** @noinspection SameParameterValue*/
        PushResult measure(String header, int targetDistance, double oldLinearScalar, double oldOffsetHeading) {
            double distance = 0;
            double heading = 0;

            // Reset the optical tracker to the origin:
            drive.opticalTracker.resetTracking();

            while (!io.ok()) {
                // Use the optical tracker directory instead of using Road Runner's pose because we
                // haven't calibrated enough yet for the latter to be correct!
                Pose2D pose = drive.opticalTracker.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                io.begin();
                io.out(header);
                io.out("Push forward exactly " + testDistance(targetDistance) + " along the field wall.\n\n");
                io.out("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h));
                io.out("&ensp;Effective distance: %.2f\"\n", distance);

                // The heading angle will jump wildly at the very start so don't show any values until
                // there's enough distance that the angle becomes more reliable:
                io.out("&ensp;Heading angle: ");
                if (distance > 24) {
                    io.out("%.2f\u00b0\n", Math.toDegrees(heading)); // Degree symbol
                } else {
                    io.out("\u2014\n"); // Em dash
                }
                io.out("\n");
                io.out("Press " + A + " when you've finished pushing, " + B + " to cancel.");
                io.end();

                if (io.cancel() || isStopRequested())
                    return null; // ====>
            }

            // Avoid divide-by-zeroes on aborts:
            if (distance == 0)
                distance = 0.001;

            // Calculate the new settings and undo the corrections that the OTOS sensor
            // was applying:
            double newLinearScalar = (targetDistance / distance) * oldLinearScalar;
            double newOffsetHeading = normalizeAngle(heading + oldOffsetHeading);

            if (newLinearScalar < SparkFunOTOS.MIN_SCALAR) {
                io.begin();
                io.out(Dialog.WARNING_ICON + "The measured distance of %.1f\" is not close enough to the expected distance " +
                                "of %d\". It can't measure more than %.1f\". ",
                        distance, targetDistance, targetDistance / SparkFunOTOS.MIN_SCALAR);
                io.out("Either you didn't push straight for " + testDistance(targetDistance) + " or " +
                        "something is wrong with the sensor. ");
                io.out("Maybe the distance of the sensor to the tile is less than 10.0 mm? ");
                io.out("\n\nDiscarded results, press " + A + " to continue.");
                io.end();
                poll.ok();
                return null; // ====>
            } else if (newLinearScalar > SparkFunOTOS.MAX_SCALAR) {
                io.begin();
                io.out(Dialog.WARNING_ICON + "The measured distance of %.1f\" is not close enough to the expected distance " +
                                "of %d\". It can't measure less than %.1f\". ",
                        distance, targetDistance, targetDistance / SparkFunOTOS.MAX_SCALAR);
                io.out("Either you didn't push straight for " + testDistance(targetDistance) + " or " +
                        "something is wrong with the sensor. ");

                // If the measured distance is close to zero, don't bother with the following
                // suggestion:
                if (newLinearScalar < 1.5) {
                    io.out("Maybe the distance of the sensor to the tile is more than 10.0 mm?");
                }
                io.out("\n\nDiscarded results, press " + A + " to continue.");
                io.end();
                poll.ok();
                return null; // ====>
            }
            return new PushResult(newLinearScalar, newOffsetHeading);
        }

        // Measure the optical linear scale and orientation.
        void execute() {
            final int DISTANCE = 96; // Test distance in inches
            configureToDrive(false); // Don't use MecanumDrive

            double oldLinearScalar = currentParameters.params.otos.linearScalar;
            if (oldLinearScalar == 0)
                oldLinearScalar = 1.0; // Can happen on the very first run, stock Road Runner sets to zero
            double oldOffsetHeading = currentParameters.params.otos.offset.h;

            Action preview = drive.actionBuilder(new Pose2d(-DISTANCE / 2.0, -60, 0))
                    .lineToX(DISTANCE / 2.0)
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new PushResult(
                    currentParameters.params.otos.linearScalar,
                    currentParameters.params.otos.offset.h));

            Screen screen = new Screen(new String[]{"Overview", "Measure", "Save and exit"});
            while (opModeIsActive()) {
                screen.update();
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("You'll push the robot forward in a straight line along a field wall for "
                            + "exactly " + testDistance(DISTANCE) + ". This will measure the following:\n"
                            + "\n"
                            + "\u2022 <b>linearScalar</b> accounts for the height of the OTOS sensor from "
                            + "the surface of the field when measuring distance.\n"
                            + "\u2022 <b>offset heading</b> corrects the robot's heading to account for the "
                            + "direction that the sensor is mounted on the robot.\n"
                            + "\n"
                            + "Press " + screen.buttons + ".");
                    io.end();

                } else if (screen.index == 1) { // Measure screen

                    previewer.update(); // Animate the trajectory preview
                    Result.showHistory(io, history); // Show measurement history and advise when done

                    io.out("To start a measurement, align the robot by hand to its starting point "
                            + "aligned to a field wall, with room for " + testDistance(DISTANCE) + ".");
                    io.out("\n\n");
                    io.out("Press " + A + " to start a measurement, " + screen.buttons + ".");
                    io.end();

                    if (io.ok()) {
                        io.clearCanvas();
                        PushResult result = measure(screen.header, DISTANCE, oldLinearScalar, oldOffsetHeading);
                        if (result != null)
                            history.add(result);
                    }

                } else if (screen.index == 2) { // Exit screen
                    if (resultsDoneScreen(history, screen, Tuner.PUSH)) {
                        // Make sure the OTOS hardware is informed of the new parameters too:
                        setOtosHardware();
                        break; // ====>
                    }
                }
            }
        }
    }

    /**
     * Class to encapsulate all Spin Tuner logic.
     */
    class SpinTuner {
        // Struct to represent Spin Tuner results:
        class SpinResult extends Result {
            double trackWidthTicks;
            double otosAngularScalar;
            double otosOffsetX;
            double otosOffsetY;

            public SpinResult(double trackWidthTicks, double otosAngularScalar, double otosOffsetX, double otosOffsetY) {
                this.trackWidthTicks = trackWidthTicks;
                this.otosAngularScalar = otosAngularScalar;
                this.otosOffsetX = otosOffsetX;
                this.otosOffsetY = otosOffsetY;
            }

            @Override
            public String getString() {
                return String.format("trackWidthTicks: %.2f, angularScalar: %.4f, offset: (%.3f, %.3f)",
                        trackWidthTicks, otosAngularScalar, otosOffsetX, otosOffsetY);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.trackWidthTicks = trackWidthTicks;
                parameters.params.otos.angularScalar = otosAngularScalar;
                parameters.params.otos.offset.x = otosOffsetX;
                parameters.params.otos.offset.y = otosOffsetY;
            }
        }

        final double REVOLUTION_COUNT = 10.0; // Number of revolutions to use
        final double SPIN_POWER = 0.5; // Speed of the revolutions

        // Detail about the most recent result:
        String detail;

        // Persisted state for initiateSparkFunRotation and updateSparkFunRotation:
        double previousSparkFunHeading;
        double accumulatedSparkFunRotation;

        // Structure to describe the center of rotation for the robot:
        class Circle {
            double x;
            double y;
            double radius;

            public Circle(double x, double y, double radius) {
                this.x = x;
                this.y = y;
                this.radius = radius;
            }
        }

        // Perform a least-squares fit of an array of points to a circle without using matrices,
        // courtesy of Copilot:
        Circle fitCircle(List<Point> points, double centerX, double centerY) {
            double radius = 0.0;

            // Iteratively refine the center and radius
            for (int iter = 0; iter < 100; iter++) {
                double sumX = 0.0;
                double sumY = 0.0;
                double sumR = 0.0;

                for (Point p : points) {
                    double dx = p.x - centerX;
                    double dy = p.y - centerY;
                    double dist = Math.sqrt(dx * dx + dy * dy);
                    sumX += dx / dist;
                    sumY += dy / dist;
                    sumR += dist;
                }

                centerX += sumX / points.size();
                centerY += sumY / points.size();
                radius = sumR / points.size();
            }
            return new Circle(centerX, centerY, radius);
        }

        // Calculate the center of the cluster of points via a simple average:
        Point clusterCenter(List<Point> points) {
            int size = points.size();
            if (size == 0)
                return new Point(0, 0);

            Point sum = new Point(0, 0);
            for (Point point : points) {
                sum = sum.add(point);
            }
            return new Point(sum.x / size, sum.y / size);
        }

        // Start tracking total amount of rotation:
        double initiateSparkFunRotation() {
            accumulatedSparkFunRotation = 0;
            previousSparkFunHeading = drive.opticalTracker.getPosition().h;
            return previousSparkFunHeading;
        }

        // Call this regularly to update the tracked amount of rotation:
        void updateRotation() {
            updateRotationAndGetPose();
        }

        Pose2D updateRotationAndGetPose() {
            if (drive.opticalTracker == null) // Handle case where we're using encoders
                return null;

            Pose2D pose = drive.opticalTracker.getPosition();
            accumulatedSparkFunRotation += normalizeAngle(pose.h - previousSparkFunHeading);
            previousSparkFunHeading = pose.h;
            return pose;
        }

        // Get the resulting total rotation amount. You should call updateRotation() before calling
        // this!
        double getSparkFunRotation() {
            return accumulatedSparkFunRotation;
        }

        // Draw the spin sample points, plus the optional best-fit circle, on FTC Dashboard:
        void drawSpinPoints(ArrayList<Point> points, Circle circle) {
            io.begin();
            Canvas canvas = io.canvas();
            double[] xPoints = new double[points.size()];
            double[] yPoints = new double[points.size()];
            for (int i = 0; i < points.size(); i++) {
                xPoints[i] = points.get(i).x;
                yPoints[i] = points.get(i).y;
            }
            canvas.setStroke("#00ff00");
            canvas.strokePolyline(xPoints, yPoints);

            // Draw the best-fit circle:
            if (circle != null) {
                canvas.setStrokeWidth(1);
                canvas.setStroke("#ff0000");
                canvas.strokeCircle(circle.x, circle.y, circle.radius);
            }

            io.end();
        }

        // Ramp the motors up or down to or from the target spin speed. Turns counter-clockwise
        // when provided a positive power value.
        void rampMotorsSpin(MecanumDrive drive, double targetPower) {
            final double RAMP_TIME = 0.5; // Seconds
            double startPower = drive.rightFront.getPower();
            double deltaPower = targetPower - startPower;
            double startTime = time();
            while (opModeIsActive()) {
                double duration = Math.min(time() - startTime, RAMP_TIME);
                double power = (duration / RAMP_TIME) * deltaPower + startPower;
                drive.rightFront.setPower(power);
                drive.rightBack.setPower(power);
                drive.leftFront.setPower(-power);
                drive.leftBack.setPower(-power);

                spinTuner.updateRotation();
                if (duration == RAMP_TIME)
                    break; // ===>
            }
        }

        // Process the spin results:
        SpinResult process(Point clusterCenter, Circle center, double totalMeasuredRotation, double distancePerRevolution, double imuYawDelta) {
            double fractionalMeasuredCircles = totalMeasuredRotation / (2 * Math.PI);
            double integerCircles = Math.round(fractionalMeasuredCircles);
            double angularScalar = integerCircles / fractionalMeasuredCircles;
            double imuYawScalar = integerCircles / (integerCircles + imuYawDelta / (2 * Math.PI));

            // Now that we have measured the angular scalar, we can correct the distance-per-revolution:
            distancePerRevolution *= angularScalar;

            // 'Track width' is really the radius of the circle needed to make a complete rotation:
            double trackWidth = distancePerRevolution / (2 * Math.PI);
            double trackWidthTicks = trackWidth / drive.PARAMS.inPerTick;

            // Undo the offset heading that the OTOS sensor automatically applies:
            Point rawOffset = new Point(center.x, center.y).rotate(-drive.PARAMS.otos.offset.h);
            Point clusterOffset = new Point(clusterCenter.x, clusterCenter.y).rotate(-drive.PARAMS.otos.offset.h);

            // Our initial origin where we established (0, 0) at the start of every circle is much
            // less reliable than the best-fit circle center and radius because the former uses
            // only a single sample point while the latter uses all sample points. Consequently, make
            // the offset fit the radius while maintaining the same angle to the center of the circle:
            double theta = rawOffset.atan2();
            Point offset = new Point(Math.cos(theta) * center.radius, Math.sin(theta) * center.radius);

            // Remember detail about these results:
            detail = String.format("Sensor thinks %.2f circles were completed.\n\n", fractionalMeasuredCircles);
            detail += String.format("Cluster center: (%.2f, %.2f)\n", clusterOffset.x, clusterOffset.y);
            detail += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", rawOffset.x, rawOffset.y, center.radius);
            detail += String.format("Radius-corrected position: (%.2f, %.2f)\n", offset.x, offset.y);
            detail += String.format("Angular scalar: %.4f\n", angularScalar);
            detail += String.format("Track width: %.2f\"\n", trackWidth);
            detail += String.format("IMU yaw scalar: %.4f\n", imuYawScalar);
            detail += "\n";

            // Do some sanity checking on the results:
            if ((Math.abs(offset.x) > 12) || (Math.abs(offset.y) > 12)) {
                io.message(Dialog.WARNING_ICON, "The calculated OTOS offset to the center of rotation "
                    + "is (%.2f, %.2f) and is clearly bogus. "
                    + "\n\n"
                    + "Discarded results, press "+A+" to continue.", offset.x, offset.y);
                poll.ok();
                return null; // ====>
            } else if ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
                io.message(Dialog.WARNING_ICON, "The calculated OTOS angular sclar of %.4f and "
                    + "out of valid range. Did you properly align the robot on the wall the same "
                    + "way at both the start and end of this test?\n\n"
                    + "Aborted, press "+A+" to continue.", angularScalar);
                poll.ok();
                return null; // ====>
            }

            return new SpinResult(trackWidthTicks, angularScalar, clusterOffset.x, clusterOffset.y);
        }

        // Do the Spin measurement step:
        SpinResult measure(String header) {
            // Let the user position the robot:
            io.message("Now drive the robot far enough away from the wall (and any objects) so "
                    + "that it can freely rotate in place."
                    + "\n\nPress "+START+" when ready for the robot to rotate, "+B+" to cancel");

            if (poll.startCancelWithDriving()) {

                ArrayList<Point> points = new ArrayList<>();

                // Spin-up the robot, starting to measure rotation for the 'scalar' computation at
                // this point:
                double scalarStartRotation = initiateSparkFunRotation();
                rampMotorsSpin(drive, SPIN_POWER);

                // Prepare for calculating how far the wheels have traveled:
                double voltageSum = 0;
                int voltageSamples = 0;
                double startTime = time();

                // Do some preparation for termination:
                final double terminationRotation = REVOLUTION_COUNT * 2 * Math.PI;
                double farthestDistance = 0;
                Point farthestPoint = new Point(0, 0);

                Point originPosition = new Point(0, 0); // The origin of the start of every circle
                double nextCircleRotation = 0;

                double imuYawStart = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2D offsetStartPosition = updateRotationAndGetPose();
                double offsetStartHeading = offsetStartPosition.h;
                double offsetStartRotation = getSparkFunRotation();
                Point rawPosition = new Point(offsetStartPosition.x, offsetStartPosition.y);

                // Now do all of the full-speed spins:
                boolean success = false;
                do {
                    // Sample the voltage to compute an average later:
                    voltageSum += drive.voltageSensor.getVoltage();
                    voltageSamples++;

                    // Check if we're at the start of a new circle:
                    double offsetRotation = getSparkFunRotation() - offsetStartRotation;
                    if (offsetRotation >= nextCircleRotation) {
                        // Remember the raw position as the start of the new circle:
                        originPosition = rawPosition;
                        nextCircleRotation += 2 * Math.PI;
                    }

                    // Now that we've potentially done the last adjustment, see if we're all done:
                    if (offsetRotation >= terminationRotation) {
                        success = true;
                        break; // ====>
                    }

                    Point currentPoint = rawPosition.subtract(originPosition).rotate(-offsetStartHeading);
                    points.add(currentPoint);
                    double distanceFromOrigin = Math.hypot(currentPoint.x, currentPoint.y);
                    if (distanceFromOrigin > farthestDistance) {
                        farthestDistance = distanceFromOrigin;
                        farthestPoint = currentPoint;
                    }

                    // Update the telemetry:
                    drawSpinPoints(points, null);
                    double rotationsRemaining = (terminationRotation - offsetRotation) / (2 * Math.PI);

                    io.begin();
                    io.out(header);
                    io.out("%.2f rotations remaining, %d samples", rotationsRemaining, points.size());
                    io.out("\n\nPress "+B+" to abort.");
                    io.end();

                    // Update for next iteration of the loop:
                    Pose2D rawPose = updateRotationAndGetPose();
                    rawPosition = new Point(rawPose.x, rawPose.y);
                } while (opModeIsActive() && !io.cancel());

                double imuYawDelta = normalizeAngle(drive.lazyImu.get().getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS) - imuYawStart);
                double endTime = time();

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                io.message("Now drive the robot to align it in the same orientation at the "
                        + "same wall as it started."
                        + "\n\nDrive the robot to its wall position, press " + A + " when done, " + B + " to cancel");
                if ((success) && poll.okCancelWithDriving()) {
                    Point clusterCenter = clusterCenter(points);
                    Circle circle = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

                    // Draw results with the fitted circle:
                    drawSpinPoints(points, circle);

                    updateRotationAndGetPose();
                    double averageVoltage = voltageSum / voltageSamples;
                    double averageVelocity = (SPIN_POWER * averageVoltage - drive.PARAMS.kS) /
                            (drive.PARAMS.kV / drive.PARAMS.inPerTick); // Velocity in inches per second

                    double totalMeasuredRotation = getSparkFunRotation() - scalarStartRotation;
                    double distancePerRevolution = averageVelocity * (endTime - startTime) / REVOLUTION_COUNT;
                    return process(clusterCenter, circle, totalMeasuredRotation, distancePerRevolution, imuYawDelta);
                }
            }
            return null;
        }

        // This is the robot spin test for calibrating the optical sensor angular scale and offset:
        void execute() {
            configureToDrive(true); // Use MecanumDrive

            // Zero these settings for the purpose of this test:
            drive.opticalTracker.setOffset(new Pose2D(0, 0, 0));
            drive.opticalTracker.setAngularScalar(0);

            Action preview = drive.actionBuilder(new Pose2d(0, -60, 0))
                    .strafeTo(new Vector2d(0, -48))
                    .turn(2 * 2 * Math.PI)
                    .strafeTo(new Vector2d(0, -60))
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new SpinResult(
                    currentParameters.params.trackWidthTicks,
                    currentParameters.params.otos.angularScalar,
                    currentParameters.params.otos.offset.x,
                    currentParameters.params.otos.offset.y));

            Screen screen = new Screen(new String[]{"Overview", "Measure", "Detail", "Save and exit"});
            while (opModeIsActive()) {
                screen.update();
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("You'll position the robot against a wall, then drive it out so that the robot "
                            + "can rotate in place %.1f times, then drive the robot against the wall "
                            + "again. This will measure the following:", REVOLUTION_COUNT);
                    io.out("\n\n"
                            + "\u2022 <b>trackWidthTicks</b> accounts for how far the wheels have to travel when "
                            + "rotating in place 360\u00b0.\n"

                            + "\u2022 <b>otos.angularScalar</b> is the calibration factor to apply to make "
                            + "the OTOS gyro more accurate.\n"

                            + "\u2022 <b>otos.offset position</b> is the location on the robot where the "
                            + "OTOS sensor is mounted, relative to the center of rotation.\n"
                            + "\n"
                            + "Press "+screen.buttons+".");
                    io.end();

                } else if (screen.index == 1) { // Measure screen

                    previewer.update(); // Animate the trajectory preview
                    Result.showHistory(io, history); // Show measurement history and advise when done

                    io.out("To start a measurement, carefully drive the robot to a wall and align "
                            + "it so that it's facing forward. This marks the start orientation for "
                            + "calibration."
                            + "\n\n"
                            + "Press "+A+" to start a measurement, "+screen.buttons+".");
                    io.end();

                    if (io.ok()) {
                        io.clearCanvas();
                        SpinResult result = measure(screen.header);
                        if (result != null)
                            history.add(result);
                    }

                } else if (screen.index == 2) { // Details screen

                    if (history.size() <= 1) {
                        io.out("This will contain detail about the most recent result.\n\n");
                    } else {
                        io.out("Here is detail about the most recent result. Feel free to ignore.\n\n");
                        io.out(detail);
                        io.out("\n\n");
                    }
                    io.out("Press " + screen.buttons);
                    io.end();

                } else if (screen.index == 3) { // Exit screen
                    if (resultsDoneScreen(history, screen, Tuner.PUSH)) {
                        // Make sure the OTOS hardware is informed of the new parameters too:
                        setOtosHardware();
                        break; // ====>
                    }
                }
            }

            // Restore the hardware settings:
            setOtosHardware();
        }
    }

    /**
     * Class to encapsulate all Accelerating Straight Line Tuner logic.
     */
    class AcceleratingStraightLineTuner {
        class AcceleratingResult extends Result {
            double kS;
            double kV;
            public AcceleratingResult(double kS, double kV) {
                this.kS = kS;
                this.kV = kV;
            }

            @Override
            public String getString() {
                return String.format("kS: %.3f, kV: %.3f", kS, kV);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.kS = kS;
                parameters.params.kV = kV;
            }
        }

        final int DISTANCE = 72; // Test distance in inches
        String graphExplanation; // Explanation of the graph
        ArrayList<Point> acceptedSamples; // x is velocity, y is power
        Point max; // x is velocity max, y is power max

        /**
         * Structure for describing the best-fit-line for a set of points.
         */
        class BestFitLine {
            double slope;
            double intercept;

            public BestFitLine(double slope, double intercept) {
                this.slope = slope;
                this.intercept = intercept;
            }
        }

        // Find the best-fit line.
        BestFitLine fitLine(ArrayList<Point> points) {
            // Calculate the means of x and y
            Point sum = new Point(0, 0);
            for (Point point: points) {
                sum = sum.add(point);
            }
            Point mean = new Point(sum.x / points.size(), sum.y / points.size());

            // Calculate the sum of (xi - xMean) * (yi - yMean) and (xi - xMean)^2
            double numerator = 0;
            double denominator = 0;
            for (int i = 0; i < points.size(); i++) {
                double diffX = points.get(i).x - mean.x;
                double diffY = points.get(i).y - mean.y;
                numerator += diffX * diffY;
                denominator += diffX * diffX;
            }

            if (denominator == 0)
                // All points are coincident or in a vertical line:
                return new BestFitLine(0, 0);

            // Calculate the slope (m) and intercept (c)
            double slope = numerator / denominator;
            double intercept = mean.y - slope * mean.x;

            return new BestFitLine(slope, intercept);
        }

        // Helper that drives an accelerating straight line and samples voltage and velocity:
        /** @noinspection SameParameterValue*/
        ArrayList<Point> driveLine(String header, double targetDistance, double direction) {
            final double VELOCITY_EPSILON = 2.0; // Inches/s
            final double POWER_FACTOR_ADDER_PER_SECOND = 0.1;
            final double MIN_POWER_FACTOR = 0.05;
            final double MAX_POWER_FACTOR = 0.9;

            // Reset the pose every time:
            drive.setPose(zeroPose);

            ArrayList<Point> points = new ArrayList<>();
            double startTime = time();
            while (opModeIsActive() && !io.cancel()) {
                // Slowly ramp up the voltage. Increase power by the specified power adder:
                double scaledPower = (time() - startTime) * POWER_FACTOR_ADDER_PER_SECOND;
                double powerFactor = scaledPower + MIN_POWER_FACTOR;

                drive.rightFront.setPower(powerFactor * direction);
                drive.rightBack.setPower(powerFactor * direction);
                drive.leftFront.setPower(powerFactor * direction);
                drive.leftBack.setPower(powerFactor * direction);

                Pose2D velocityVector = drive.opticalTracker.getVelocity();
                double velocity = Math.hypot(velocityVector.x, velocityVector.y);

                // Discard zero velocities that will happen when the provided power isn't
                // enough yet to overcome static friction:
                if (velocity > VELOCITY_EPSILON) {
                    double power = powerFactor * drive.voltageSensor.getVoltage();
                    points.add(new Point(velocity, power));
                }
                Pose2D position = drive.opticalTracker.getPosition();
                double distance = Math.hypot(position.x, position.y);

                // We're done if we've reach the maximum target voltage or gone far enough:
                if ((powerFactor > MAX_POWER_FACTOR) || (distance > targetDistance)) {
                    stopMotors();
                    return points; // ====>
                }

                double remaining = Math.max(0, targetDistance - distance);
                io.begin();
                io.out(header);
                io.out("Inches remaining: %.1f, power: %.1f\n\n", remaining, powerFactor);
                io.out("Press "+B+" to cancel and stop the robot.");
                io.end();
            }

            // Stop the robot:
            stopMotors();
            return null;
        }

        // Find the independent (x, y) maximum values:
        void findMaxValues(Point max, List<Point> points) {
            for (Point point: points) {
                max.x = Math.max(max.x, point.x);
                max.y = Math.max(max.y, point.y);
            }
        }

        // Do the accelerating measurement step:
        AcceleratingResult measure(String header) {
            io.clearCanvas();

            // Do one forward and back run:
            ArrayList<Point> forwardSamples = driveLine(header, DISTANCE, 1.0);
            if (forwardSamples != null) {
                ArrayList<Point> reverseSamples = driveLine(header, DISTANCE, -1.0);
                if (reverseSamples != null) {
                    findMaxValues(max, forwardSamples);
                    findMaxValues(max, reverseSamples);
                    if (max.x == 0) {
                        io.message(Dialog.WARNING_ICON + "The optical tracking sensor returned only zero velocities. "
                                + "Is it working properly?"
                                + "\n\nAborted, press " + A + " to continue.");
                        poll.ok();
                        return null; // ====>
                    } else {
                        // Draw the results to the FTC dashboard:
                        io.begin();
                        Canvas canvas = io.canvas();

                        // Set a solid white background and an offset then draw the point samples:
                        canvas.setFill("#ffffff");
                        canvas.fillRect(-72, -72, 144, 144);
                        canvas.setTranslation(-72, 72);

                        // The canvas coordinates go from -72 to 72 so scale appropriately. We don't
                        // use canvas.setScale() because that scales up the stroke widths and so makes
                        // the lines too fat.
                        Point scale = new Point(144 / max.x, 144 / max.y);
                        canvas.setStroke("#00ff00");
                        strokeSamples(canvas, scale, forwardSamples);
                        canvas.setStroke("#0000ff");
                        strokeSamples(canvas, scale, reverseSamples);

                        // Compute and draw the best-fit line for both sets of points:
                        ArrayList<Point> combinedSamples = new ArrayList<>();
                        combinedSamples.addAll(acceptedSamples);
                        combinedSamples.addAll(forwardSamples);
                        combinedSamples.addAll(reverseSamples);
                        BestFitLine bestFitLine = fitLine(combinedSamples);

                        canvas.setStrokeWidth(1);
                        canvas.setStroke("#ff0000");
                        canvas.strokeLine(0, bestFitLine.intercept * scale.y,
                                200 * scale.x, (bestFitLine.intercept + 200 * bestFitLine.slope) * scale.y);

                        double kS = bestFitLine.intercept;
                        double kV = bestFitLine.slope * currentParameters.params.inPerTick;

                        graphExplanation = String.format("The graph in FTC Dashboard shows "
                                + "velocity as the <i>x</i> axis going up to %.1f\"/s, and voltage "
                                + "as the <i>y</i> axis going up to %.2fV.\n\n", max.x, max.y);

                        // Automatically accept the combined results. (We could ask...)
                        acceptedSamples = combinedSamples;
                        return new AcceleratingResult(kS, kV);
                    }
                }
            }
            return null;
        }

        // Automatically calculate the kS and kV terms of the feed-forward approximation by
        // ramping up the velocity in a straight line. We increase power by a fixed increment.
        void execute() {
            // Reset persisted state that we don't want preserved across invocations:
            graphExplanation = "";
            acceptedSamples = new ArrayList<>();
            max = new Point(0, 0);

            configureToDrive(true); // Set the brakes

            Action preview = drive.actionBuilder(new Pose2d(-DISTANCE / 2.0, 0, 0))
                    .lineToX(DISTANCE / 2.0, null, new ProfileAccelConstraint(-100, 10))
                    .lineToX(-DISTANCE / 2.0, null, new ProfileAccelConstraint(-100, 20))
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new AcceleratingResult(
                    currentParameters.params.kS,
                    currentParameters.params.kV));

            Screen screen = new Screen(new String[]{"Overview", "Measure", "Save and exit"});
            while (opModeIsActive()) {
                screen.update();
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("The robot will drive forward and backward for "
                            + testDistance(DISTANCE) + ". It will start slowly but get faster and "
                            + "faster in each direction. This will measure the following:\n");
                    io.out("\n"
                            + "\u2022 <b>kS</b> is the motor voltage required to overcome static "
                            + "friction and make the robot move after it's stopped. It's the "
                            + "intercept on the velocity/voltage graph.\n"
                            + "\u2022 A motor's velocity is proportional to the voltage applied, and "
                            + "<b>kV</b> is the ratio of motor voltage to robot velocity. It's the "
                            + "slope of the velocity/voltage graph.\n"
                            + "\n"
                            + "Press " + screen.buttons + ".");
                    io.end();

                } else if (screen.index == 1) { // Measure screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive

                    io.out(graphExplanation);
                    Result.showHistory(io, history); // Show measurement history and advise when done

                    io.out("To start a measurement, drive the robot to a spot that is clear "
                            + "in front for " + testDistance(DISTANCE) + ".\n"
                            + "\n"
                            + "Press " + START + " to start the robot, " + screen.buttons + ".");
                    io.end();
                    if (io.start()) {
                        io.clearCanvas();
                        AcceleratingResult result = measure(screen.header);
                        if (result != null)
                            history.add(result);
                    }

                } else if (screen.index == 2) { // Exit screen
                    if (resultsDoneScreen(history, screen, Tuner.ACCELERATING)) {
                        // Make sure the OTOS hardware is informed of the new parameters too:
                        setOtosHardware();
                        break; // ====>
                    }
                }
            }
        }
    }

    /**
     * Class to encapsulate all Interface Feed Forward Tuner logic.
     */
    class InteractiveFeedForwardTuner {
        class FeedForwardResult extends Result {
            double kV;
            double kA;
            public FeedForwardResult(double kV, double kA) {
                this.kV = kV;
                this.kA = kA;
            }

            @Override
            public String getString() { return String.format("kV: %.3f, kA: %.4f", kV, kA); }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.kV = kV;
                parameters.params.kA = kA;
            }
        }

        final int DISTANCE = 72; // Test distance in inches

        // Tune the kV and kA feed forward parameters:
        void execute() {
            configureToDrive(false); // Don't use MecanumDrive

            // Trigger a reset the first time into the loop:
            double profileStartTime = 0; // Profile start time, in seconds
            double cycleStartTime = 0; // Cycle start time, in second (each cycle consumes 2 profiles)
            double maxVelocityFactor = 0.8;
            TimeProfile profile = null;
            boolean movingForwards = false;
            int qeuedStarts = 0;
            double maxVelocity = 0; // Maximum measured velocity
            double maxDuration = 4; // Maximum graph duration, in seconds

            // Allocate a repository for all of our velocity samples:
            class Sample {
                final double time; // Seconds
                final double target; // Target velocity, inches/s
                final double actual; // Actual velocity, inches/s

                public Sample(double time, double target, double actual) {
                    this.time = time; this.target = target; this.actual = actual;
                }
            }

            final String TARGET_COLOR = "#4CAF50"; // Green
            final String ACTUAL_COLOR = "#3F51B5"; // Blue
            final double GRAPH_THROTTLE = 0.1; // Only update the graph every 0.1 seconds
            double lastGraphTime = 0;
            LinkedList<Sample> samples = new LinkedList<>();

            // Disable all lateral gains so that backward and forward behavior is not affected by the
            // PID/Ramsete algorithm. It's okay for the axial and rotation gains to be either zero
            // or non-zero:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.lateralGain = 0;
            testParameters.params.lateralVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            NumericInput vInput = new NumericInput(drive.PARAMS, "kV", -2, 3, 0.000001, 20);
            NumericInput aInput = new NumericInput(drive.PARAMS, "kA", -3, 4, 0, 1);

            Action preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, 0, 0))
                    .lineToX(DISTANCE/2.0)
                    .lineToX(-DISTANCE/2.0)
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new FeedForwardResult(currentParameters.params.kV, currentParameters.params.kA));

            Screen screen = new Screen(new String[]{"Overview", "Adjust kV", "Adjust kA", "Save and exit"});
            while (opModeIsActive()) {
                // Note we don't do screen.update() here because we restrict when it's called
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen
                    screen.update();
                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("The robot will drive forward then backward for " + testDistance(DISTANCE) + "."
                            + "Tune the following:");
                    io.out("\n\n"
                            + "\u2022 <b>kV</b> is the same parameter as for the preceding tuner. "
                            + "It's the ratio of motor voltage to robot velocity.\n"
                            + "\u2022 <b>kA</b> defines how much additional voltage is required when "
                            + "applying acceleration.\n");
                    io.out("\n"
                            + "These are used in a <i>Feed Forward</i> model (just like the <i>F</i> "
                            + "in a <i>PIDF</i>) to determine how much voltage to apply to attain a "
                            + "target velocity:\n"
                            + "&emsp;<i>v<sub>approx</sub> = k<sub>v</sub> \u00b7 v + k<sub>a</sub> \u00b7 a + k<sub>static</sub></i>\n");

                    // "Follow <u><a href='https://learnroadrunner.com/feedforward-tuning.html#tuning'>LearnRoadRunner's guide</a></u>.\n\n"

                    io.out("\n"
                            + "Press " + screen.buttons + ".");
                    io.end();
                } else if (screen.index == 3) { // Exit screen
                    screen.update();

                    // Create results from the current settings:
                    FeedForwardResult result = new FeedForwardResult(drive.PARAMS.kV, MecanumDrive.PARAMS.kA);

                    // Keep the history at a size of two:
                    if (history.size() > 1)
                        history.remove(1);

                    // Make the new result the second in the history if it's different from the first:
                    if (!result.getString().equals(history.get(0).getString()))
                        history.add(result);

                    if (resultsDoneScreen(history, screen, Tuner.FEED_FORWARD))
                        break; // ====>

                } else { // kV and kA tuning screens
                    updateGamepadDriving(); // Let the user drive

                    if (screen.index == 1) {
                        io.out("&emsp;kV: <big><big>%s</big></big>\n", vInput.update());
                        io.out("&emsp;Max velocity is <b>%.0f%%</b>.\n\n", maxVelocityFactor * 100.0);
                        io.out("View the graph in FTC Dashboard and adjust "
                                + "<b>kV</b> to make the horizontal lines as close as possible in height. "
                                + "<b>vTarget</b> is green, <b>vActual</b> is blue, <i>kV = vTarget / vActual</i>.\n\n");
                    } else {
                        io.out("&emsp;kA: <big><big>%s</big></big>\n", aInput.update());
                        io.out("&emsp;Max velocity is <b>%.0f%%</b>.\n\n", maxVelocityFactor * 100.0);
                        io.out("View the graph in FTC Dashboard and adjust "
                                + "<b>kA</b> to shift <b>vActual</b> left and right so the angled lines overlap "
                                + "where the robot accelerates. Don't worry about the deceleration portions.\n\n");
                    }
                    if (io.start())
                        qeuedStarts++; // Let starts be queued up even while running

                    // If there's a profile, that means we're actively moving:
                    if (profile != null) {
                        io.out("Press " + B + " to cancel and stop the robot.\n");
                        io.end();

                        double time = time();
                        double elapsedTime = time - profileStartTime;
                        if (elapsedTime > profile.duration) {
                            if (movingForwards) {
                                movingForwards = false;
                                profileStartTime = time;
                            } else {
                                profile = null;
                                continue; // ====>
                            }
                        }

                        DualNum<Time> v = profile.get(elapsedTime).drop(1);
                        if (!movingForwards) {
                            v = v.unaryMinus();
                        }

                        // Calculate the new sample and log it:
                        Pose2D velocityPose = drive.opticalTracker.getVelocity();
                        double targetVelocity = v.get(0);
                        double actualVelocity = Math.signum(velocityPose.x) * Math.hypot(velocityPose.x, velocityPose.y);
                        maxVelocity = Math.max(maxVelocity, Math.max(Math.abs(targetVelocity), Math.abs(actualVelocity)));
                        maxDuration = Math.max(maxDuration, time - cycleStartTime);
                        samples.addLast(new Sample(time, v.get(0), actualVelocity));

                        // Throttle the Dashboard updates so that it doesn't get overwhelmed as it
                        // has very bad rate control:
                        if (time - lastGraphTime > GRAPH_THROTTLE) {
                            lastGraphTime = time;

                            io.begin();
                            Canvas canvas = io.canvas();
                            canvas.setFill("#ffffff");
                            canvas.fillRect(-72, -72, 144, 144);
                            canvas.setTranslation(0, 72);
                            double xScale = (maxDuration == 0) ? 1 : 144 / maxDuration;
                            double yScale = (maxVelocity == 0) ? 1 : 72 / maxVelocity;

                            int count = samples.size();
                            double[] xPoints = new double[count];
                            double[] yTargets = new double[count];
                            double[] yActuals = new double[count];
                            for (int i = 0; i < count; i++) {
                                Sample sample = samples.get(i);
                                xPoints[i] = (sample.time - (time - maxDuration)) * xScale;
                                yTargets[i] = sample.target * yScale;
                                yActuals[i] = sample.actual * yScale;
                            }
                            canvas.setStroke(TARGET_COLOR);
                            canvas.strokePolyline(xPoints, yTargets);
                            canvas.setStroke(ACTUAL_COLOR);
                            canvas.strokePolyline(xPoints, yActuals);

                            // Make the results available to FTC Dashboard so that they can be graphed
                            // there as well:
                            io.put("vActual", actualVelocity);
                            io.put("vTarget", v.get(0));
                            io.putDivider();
                            io.end();
                        }

                        // Set the motors to the appropriate values:
                        MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

                        double power = feedForward.compute(v) / drive.voltageSensor.getVoltage();
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));

                        if (io.cancel()) {
                            // Cancel the current cycle but remain in this test:
                            stopMotors();
                            qeuedStarts = 0;
                            profile = null;
                        }
                    } else {
                        io.out("Changing max velocity will lengthen or shorten the horizontal lines.\n"
                            + "\n");
                        io.out(DPAD_UP_DOWN + " to change value, " + DPAD_LEFT_RIGHT + " to move "
                                + "the cursor, " + TRIGGERS + " to change max velocity, "
                                + START + " to start the robot, " + screen.buttons);
                        io.end();

                        screen.update(); // Allow the screen to be changed.
                        updateGamepadDriving();

                        if (io.leftTrigger()) {
                            maxVelocityFactor = Math.max(maxVelocityFactor - 0.1, 0.2);
                            maxDuration = 0; // A different speed will have a different window size
                        }
                        if (io.rightTrigger()) {
                            maxVelocityFactor = Math.min(maxVelocityFactor + 0.1, 1.0);
                            maxDuration = 0; // A different speed will have a different window size
                        }
                        if (qeuedStarts > 0) {
                            qeuedStarts--;
                            stopMotors(); // Stop the user's driving
                            movingForwards = true;
                            cycleStartTime = time();
                            profileStartTime = time();
                            profile = new TimeProfile(constantProfile(
                                    DISTANCE, 0.0,
                                    MecanumDrive.PARAMS.maxWheelVel * maxVelocityFactor,
                                    MecanumDrive.PARAMS.minProfileAccel,
                                    MecanumDrive.PARAMS.maxProfileAccel).baseProfile);
                            samples = new LinkedList<>();

                            // Reset the start position on every cycle. This ensures that getVelocity().x
                            // is the appropriate velocity to read, and it resets after we reposition
                            // the robot:
                            drive.setPose(new Pose2d(-DISTANCE / 2.0, 0, 0));
                        }
                    }
                }
            }

            // We're done, undo any temporary state we set:
            MecanumDrive.PARAMS = currentParameters.params;
            stopMotors();
            io.clearDashboardTelemetry();
        }
    }

    /**
     * Class to encapsulate all Lateral Tuner logic.
     */
    class LateralMultiplierTuner {
        class LateralResult extends Result {
            double lateralInPerTick;

            public LateralResult(double lateralInPerTick) {
                this.lateralInPerTick = lateralInPerTick;
            }

            @Override
            public String getString() {
                return String.format("lateralInPerTick: %.3f", lateralInPerTick);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.lateralInPerTick = lateralInPerTick;
            }
        }

        final int DISTANCE = 72; // Test distance in inches
        String latestResult; // Multiplier result from latest test

        // Do the lateral multiplier measurement step:
        LateralResult measure(String header, TuneParameters testParameters, List<Result> history) {
            // Accelerate and decelerate slowly so we don't overshoot:
            ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(-10, 15);

            // Strafe left and then right:
            Pose2d startPose1 = new Pose2d(0, -DISTANCE / 2.0, 0);
            drive.setPose(startPose1);
            if (runCancelableAction(header, drive.actionBuilder(startPose1)
                    .strafeTo(new Vector2d(0, DISTANCE / 2.0), null, accelConstraint)
                    .build())) {

                double actualDistance1 = Math.hypot(drive.pose.position.x, drive.pose.position.y);

                Pose2d startPose2 = new Pose2d(0, DISTANCE / 2.0, 0);
                drive.setPose(startPose2);
                if (runCancelableAction(header, drive.actionBuilder(startPose2)
                        .strafeTo(new Vector2d(0, -DISTANCE / 2.0), null, accelConstraint)
                        .build())) {

                    double actualDistance2 = Math.hypot(drive.pose.position.x, drive.pose.position.y);
                    double multiplier1 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance1 / DISTANCE);
                    double multiplier2 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance2 / DISTANCE);

                    if (Math.min(multiplier1, multiplier2) < 0.25) {
                        io.message(Dialog.WARNING_ICON + "The measured distance is too low to be correct. "
                                + "Did it not move, or is the distance sensor not working properly?"
                                + "\n\nPress " + A + " to continue.");
                        poll.ok();
                        return null;
                    }

                    double newMultiplier = (multiplier1 + multiplier2) / 2.0; // Compute the average
                    latestResult = String.format("Measured multiplier: %.3f\n\n", newMultiplier);

                    // Compute the average of the history *plus* the new multiplier but skipping
                    // the first entry:
                    double sum = newMultiplier;
                    for (int i = 1; i < history.size(); i++) {
                        sum += ((LateralResult) history.get(i)).lateralInPerTick;
                    }
                    double newLateralInPerTick = sum / history.size();

                    // Adopt the new settings:
                    testParameters.params.lateralInPerTick = newLateralInPerTick;
                    MecanumDrive.PARAMS = testParameters.params;
                    drive.initializeKinematics();

                    return new LateralResult(newLateralInPerTick); // ====>
                }
            }
            return null; // ====>
        }

        // Tune for the lateral multiplier on Mecanum drives.
        void execute() {
            configureToDrive(true); // Do use MecanumDrive
            latestResult = "";

            // Disable the PID gains so that the distance traveled isn't corrected:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.lateralGain = 0;
            testParameters.params.lateralVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            // Now recreate the Kinematics object based on the new settings:
            drive.initializeKinematics();

            Action preview = drive.actionBuilder(new Pose2d(0, -DISTANCE / 2.0, 0))
                    .strafeTo(new Vector2d(0, DISTANCE / 2.0))
                    .strafeTo(new Vector2d(0, -DISTANCE / 2.0))
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new LateralResult(currentParameters.params.lateralInPerTick));

            Screen screen = new Screen(new String[]{"Overview", "Measure", "Save and exit"});
            while (opModeIsActive()) {
                screen.update();
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("The robot will strafe left then right for up to "
                            + testDistance(DISTANCE) + ". This will measure the following:\n");
                    io.out("\n"
                            + "\u2022 <b>lateralInPerTick</b> is the factor of how "
                            + "much shorter the robot will move sideways than forward for the same "
                            + "amount of wheel rotation. Mecanum drives are less efficient moving "
                            + "sideways than forward or reverse.\n"
                            + "\n"
                            + "Press " + screen.buttons + ".");
                    io.end();

                } else if (screen.index == 1) { // Measure screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive

                    io.out(latestResult);
                    Result.showHistory(io, history);

                    io.out("If starting a measurement, ensure that " + testDistance(DISTANCE) + " "
                            + "is clear to the robot's left.\n"
                            + "\n"
                            + "Press " + START + " to start the robot, " + screen.buttons + ".");
                    io.end();
                    if (io.start()) {
                        io.clearCanvas();
                        LateralResult result = measure(screen.header, testParameters, history);
                        if (result != null)
                            history.add(result);
                    }
                } else if (screen.index == 2) { // Exit screen
                    if (resultsDoneScreen(history, screen, Tuner.LATERAL_MULTIPLIER)) {
                        break; // ====>
                    }
                }
            }

            // Restore the kinematics:
            MecanumDrive.PARAMS = currentParameters.params;
            drive.initializeKinematics();
        }
    }

    class InteractivePidTuner {
        class PidResult extends Result {
            PidTunerType type;
            double axialGain;
            double axialVelGain;
            double lateralGain;
            double lateralVelGain;
            double headingGain;
            double headingVelGain;

            public PidResult(PidTunerType type, MecanumDrive.Params params) {
                this.type = type;
                this.axialGain = params.axialGain;
                this.axialVelGain = params.axialVelGain;
                this.lateralGain = params.lateralGain;
                this.lateralVelGain = params.lateralVelGain;
                this.headingGain = params.headingGain;
                this.headingVelGain = params.headingVelGain;
            }

            @Override
            public String getString() {
                if (type == PidTunerType.AXIAL) {
                    return String.format("axialGain: %.3f, axialVelGain: %.3f", axialGain, axialVelGain);
                } else if (type == PidTunerType.LATERAL) {
                    return String.format("lateralGain: %.3f, lateralVelGain: %.3f", lateralGain, lateralVelGain);
                } else if (type == PidTunerType.HEADING) {
                    return String.format("headingGain: %.3f, headingVelGain: %.3f", headingGain, headingVelGain);
                } else {
                    return String.format("Gains: %.3f, %.3f, %.3f, VelGains: %.3f, %.3f, %.3f",
                            axialGain, lateralGain, headingGain, axialVelGain, lateralVelGain, headingVelGain);
                }
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.axialGain = axialGain;
                parameters.params.axialVelGain = axialVelGain;
                parameters.params.lateralGain = lateralGain;
                parameters.params.lateralVelGain = lateralVelGain;
                parameters.params.headingGain = headingGain;
                parameters.params.headingVelGain = headingVelGain;
            }
        }

        final int DISTANCE = 48; // Test distance in inches
        String errorSummary; // String describing the current amount of error
        double maxAxialError; // Maximum error accumulated over the current robot run
        double maxLateralError;
        double maxHeadingError;

        // Run the tuning update:
        boolean updateTuning(PidTunerType type) {
            // Execute the trajectory:
            boolean more = drive.doActionsWork(io.packet);

            // Update the error summary if we're actively running a trajectory, or if it's
            // previously been updated:
            if ((more) || !errorSummary.isEmpty()) {
                // Compute the errors:
                Point errorVector = new Point(drive.pose.position.x, drive.pose.position.y).subtract(
                        new Point(drive.targetPose.position.x, drive.targetPose.position.y));
                double errorTheta = errorVector.atan2() - drive.targetPose.heading.toDouble();
                double errorLength = errorVector.length();
                double axialError = errorLength * Math.cos(errorTheta);
                double lateralError = errorLength * Math.sin(errorTheta);
                double headingError = normalizeAngle(drive.pose.heading.toDouble()
                        - drive.targetPose.heading.toDouble());

                maxAxialError = Math.max(maxAxialError, axialError);
                maxLateralError = Math.max(maxLateralError, lateralError);
                maxHeadingError = Math.max(maxHeadingError, headingError);

                errorSummary = "Max error: ";
                if (type == PidTunerType.AXIAL)
                    errorSummary += String.format("%.2f\"", maxAxialError);
                else if (type == PidTunerType.LATERAL)
                    errorSummary += String.format("%.2f\"", maxLateralError);
                else if (type == PidTunerType.HEADING)
                    errorSummary += String.format("%.2f\u00b0", Math.toDegrees(maxHeadingError));
                else
                    errorSummary += String.format("%.2f\", %.2f\", %.2f\u00b0",
                            maxAxialError, maxLateralError, Math.toDegrees(maxHeadingError));

                if (!more) {
                    errorSummary += (type == PidTunerType.ALL) ? "\n" : ", ";
                    errorSummary += "End error: ";
                    if (type == PidTunerType.AXIAL)
                        errorSummary += String.format("%.2f\"", axialError);
                    else if (type == PidTunerType.LATERAL)
                        errorSummary += String.format("%.2f\"", lateralError);
                    else if (type == PidTunerType.HEADING)
                        errorSummary += String.format("%.2f\u00b0", Math.toDegrees(headingError));
                    else
                        errorSummary += String.format("%.2f\", %.2f\", %.2f\u00b0",
                                axialError, lateralError, Math.toDegrees(headingError));
                }
                errorSummary += "\n\n";
            }
            return more;
        }

        void execute(PidTunerType type) {
            configureToDrive(true); // Do use MecanumDrive
            errorSummary = "";

            TuneParameters testParameters = currentParameters.createClone();
            MecanumDrive.PARAMS = testParameters.params;
            String overview;
            String clearance;
            String[] gainNames;
            Tuner tuner;

            TrajectoryActionBuilder trajectory = drive.actionBuilder(zeroPose);
            Action preview;
            String adjective;
            if (type == PidTunerType.AXIAL) {
                overview = "The robot will drive forward then backward " + testDistance(DISTANCE) + ". "
                    + "Tune these to reduce the forward/backward error between target and actual position:\n"
                    + "\n"
                    + "\u2022 <b>axialGain</b> sets the magnitude of response to the error. "
                    + "A higher value more aggressively corrects but can cause overshoot.\n"
                    + "\u2022 <b>axialVelGain</b> is a damper and can reduce overshoot and oscillation.\n";
                clearance = "ensure "+  clearanceDistance(DISTANCE) + " forward clearance";
                adjective = "axially ";
                gainNames = new String[] { "axialGain", "axialVelGain" };
                tuner = Tuner.AXIAL_GAIN;
                trajectory = trajectory.lineToX(DISTANCE).lineToX(0);
                preview = drive.actionBuilder(new Pose2d(-DISTANCE / 2.0, 0, 0))
                        .lineToX(DISTANCE / 2.0)
                        .lineToX(-DISTANCE / 2.0)
                        .build();

            } else if (type == PidTunerType.LATERAL) {
                overview = "The robot will strafe left and then right " + testDistance(DISTANCE) + ". "
                        + "Tune these to reduce the lateral error between target and actual positions:\n"
                        + "\n"
                        + "\u2022 <b>lateralGain</b> sets the magnitude of response to the error. "
                        + "A higher value more aggressively corrects but can cause overshoot.\n"
                        + "\u2022 <b>lateralVelGain</b> is a damper and can reduce overshoot and oscillation.\n";
                clearance = "ensure " + clearanceDistance(DISTANCE) + " to the left";
                adjective = "laterally ";
                gainNames = new String[] { "lateralGain", "lateralVelGain" };
                tuner = Tuner.LATERAL_GAIN;
                trajectory = trajectory.strafeTo(new Vector2d(0, DISTANCE)).strafeTo(new Vector2d(0, 0));
                preview = drive.actionBuilder(new Pose2d(0, -DISTANCE / 2.0, 0))
                        .strafeTo(new Vector2d(0, DISTANCE / 2.0))
                        .strafeTo(new Vector2d(0, -DISTANCE / 2.0))
                        .build();

            } else if (type == PidTunerType.HEADING) {
                overview = "The robot will rotate in place 180\u00b0. Tune these to reduce the "
                        + "error between target and actual headings:"
                        + "\n"
                        + "\u2022 <b>headingGain</b> sets the magnitude of response to the error. "
                        + "A higher value more aggressively corrects but can cause overshoot.\n"
                        + "\u2022 <b>headingVelGain</b> is a damper and can reduce overshoot and oscillation.\n";
                clearance = "ensure enough clearance to spin";
                adjective = "rotationally ";
                gainNames = new String[] { "headingGain", "headingVelGain" };
                tuner = Tuner.HEADING_GAIN;
                trajectory = trajectory.turn(Math.PI).turn(-Math.PI);
                preview = drive.actionBuilder(zeroPose)
                        .turn(Math.PI)
                        .turn(-Math.PI)
                        .build();
            } else {
                overview = "The robot will drive forward then backward " + testDistance(DISTANCE)
                    + " while turning. Tune the gains as appropriate.\n";
                clearance = "ensure sufficient clearance";
                adjective = "";
                gainNames = new String[] { "axialGain", "axialVelGain", "lateralGain", "lateralVelGain", "headingGain", "headingVelGain" };
                tuner = Tuner.NONE;
                trajectory = trajectory
                    .lineToXLinearHeading(DISTANCE, Math.PI)
                    .endTrajectory() // Stop at the end of the line
                    .setTangent(Math.PI) // When we start again, go in the direction of 180 degrees
                    .lineToXLinearHeading(0, 0);
                preview = trajectory.build();
            }

            ArrayList<NumericInput> inputs = new ArrayList<>();
            ArrayList<String> screenNames = new ArrayList<>();
            screenNames.add("Overview");
            for (String gainName: gainNames) {
                screenNames.add(String.format("Tune %s", gainName));
                inputs.add(new NumericInput(drive.PARAMS, gainName, -1, 2, 0, 20));
            }
            screenNames.add("Save and exit");

            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            ArrayList<Result> history = new ArrayList<>();
            history.add(new PidResult(type, currentParameters.params));

            int queuedStarts = 0;
            Screen screen = new Screen(screenNames.toArray(new String[0]));
            screen.onSwitch(()-> errorSummary = ""); // Clear summary on screen changes
            while (opModeIsActive()) {
                screen.update();
                io.begin();
                io.out(screen.header);
                if (screen.index == 0) { // Overview screen
                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out(overview);
                    io.out("\n"
                            + "These are essentially the <b>P</b> and <b>D</b> (<i>proportional</i> and "
                            + "<i>differential</i>) terms for a PID control system.\n"
                            + "\n"
                            + "Press " + screen.buttons + ".");
                    io.end();
                } else if (screen.index == screenNames.size() - 1) { // Exit screen
                    screen.update();

                    // Create results from the current settings:
                    PidResult result = new PidResult(type, drive.PARAMS);

                    // Keep the history at a size of two:
                    if (history.size() > 1)
                        history.remove(1);

                    // Make the new result the second in the history if it's different from the first:
                    if (!result.getString().equals(history.get(0).getString()))
                        history.add(result);

                    if (resultsDoneScreen(history, screen, tuner))
                        break; // ====>
                } else { // Tune screens
                    int index = screen.index - 1;
                    NumericInput input = inputs.get(index);
                    io.out("&emsp;%s: <big><big>%s</big></big>\n", input.fieldName, input.update());

                    if ((index & 1) == 0) { // Tuning a proportional gain
                        io.out("\n<b>"+ errorSummary + "</b>");
                        io.out("Increase the gain to make the circles %scoincident and to minimize "
                            + "the maximum and final error. ", adjective);
                        io.out("Don't increase so much that the robot has "
                            + "significant shaking or oscillation. ");
                        io.out("(A small amount can be corrected by adjusting the corresponding "
                            + "velocity gain.) ");
                    } else { // Tuning a derivative gain
                        io.out("&emsp;Don't exceed %.2f (\u2153 the other gain)\n", // One third
                                0.33 * inputs.get(index ^1).value);
                        io.out("\n<b>"+ errorSummary + "</b>");
                        io.out("Increase the velocity gain to dampen oscillation "
                            + "and shaking, but not so much that it makes it worse. ");
                    }
                    io.out("Press "+ DPAD_UP_DOWN + " to change the value, " + DPAD_LEFT_RIGHT
                                    + " to move the cursor\n\n");

                    if (io.start())
                        queuedStarts++;

                    // Continue the trajectory, if any:
                    if (updateTuning(type)) {
                        io.out("Press " + B + " to cancel and stop the robot.");
                        io.end();
                        if (io.cancel()) {
                            // Cancel the current cycle but remain in this test:
                            drive.abortActions();
                            queuedStarts = 0;
                        }
                    } else {
                        updateGamepadDriving(); // Let the user drive

                        io.out("Press "+ START + " to start the robot (%s), %s.",
                            clearance, screen.buttons);
                        io.end();

                        if (io.start())
                            queuedStarts++;

                        if (queuedStarts > 0) {
                            // Kick off a new run:
                            queuedStarts--;
                            stopMotors(); // Stop the user's driving
                            drive.setPose(zeroPose);
                            if (type == PidTunerType.HEADING) {
                                // An apparent Road Runner bug prevents a turn trajectory from being reused:
                                drive.runParallel(drive.actionBuilder(zeroPose).turn(Math.PI).turn(-Math.PI).build());
                            } else {
                                drive.runParallel(trajectory.build());
                            }
                            maxAxialError = 0;
                            maxHeadingError = 0;
                            maxLateralError = 0;
                        }
                    }
                }
            }
            MecanumDrive.PARAMS = currentParameters.params;
        }
    }

    // Navigate a short spline as a completion test.
    void completionTest() {
        Action action = drive.actionBuilder(zeroPose)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(180)), Math.toRadians(60))
                .endTrajectory()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-0.0001)), Math.toRadians(-180))
                .build();
        String message = "The robot will drive forward 48 inches using a spline. "
                + "It needs half a tile clearance on either side. ";
        runTrajectory(action, message);

        currentParameters.passedCompletionTest = true;
        currentParameters.save();
        updateTunerDependencies(Tuner.COMPLETION_TEST);
    }

    // Simple verification test for 'TrackWidth':
    void rotationTest() {
        configureToDrive(true); // Do use MecanumDrive

        io.message("To test 'trackWidthTicks', the robot will turn in-place for two complete "
                + "rotations.\n\nPress "+A+" to start, "+B+" to cancel.");
        if (poll.okCancelWithDriving()) {
            // Disable the rotational PID/Ramsete behavior so that we can test just the
            // feed-forward rotation:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.headingGain = 0;
            testParameters.params.headingVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            Action action = drive.actionBuilder(zeroPose)
                    .turn(2 * Math.toRadians(360), new TurnConstraints(
                            MecanumDrive.PARAMS.maxAngVel / 3,
                            -MecanumDrive.PARAMS.maxAngAccel,
                            MecanumDrive.PARAMS.maxAngAccel))
                    .build();
            runCancelableAction("", action);

            io.message("The robot should be facing the same direction as when it started. It it's "
                    + "not, run the spin tuner again to re-tune 'trackWidthTicks'."
                    + "\n\nPress "+A+" to continue.");
            poll.ok();

            // Restore the parameters:
            MecanumDrive.PARAMS = currentParameters.params;
        }
    }

    // Examples:
    void splineExample() {
        runTrajectory(drive.actionBuilder(zeroPose)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build());
    }
    void lineToTurnExample() {
        runTrajectory(drive.actionBuilder(zeroPose)
                .lineToX(24)
                .turn(Math.PI/2)
                .lineToY(24)
                .turn(Math.PI/2)
                .lineToX(0)
                .turn(Math.PI/2)
                .lineToY(0)
                .turn(Math.PI/2)
                .build());
    }
    void lineWithRotationExample() {
        final double DISTANCE = 60.0;
        runTrajectory(drive.actionBuilder(zeroPose)
                // To simply turn 180 degrees while traveling down a line, do this:
                //   .lineToXLinearHeading(DISTANCE, Math.PI)
                // But we want to turn a complete 360 degrees, and then return, so it's complicated:
                .splineToSplineHeading(new Pose2d(DISTANCE / 2.0, 0, Math.PI), 0)
                .splineToSplineHeading(new Pose2d(DISTANCE, 0, -0.0001), 0)
                .endTrajectory() // Stop at the end of the line
                .setTangent(Math.PI) // When we start again, go in the direction of 180 degrees
                .splineToSplineHeading(new Pose2d(DISTANCE / 2.0, 0, Math.PI), Math.PI)
                .splineToSplineHeading(new Pose2d(0, 0, 0.0001), Math.PI)
                .build());
    }

    // Show all of the parameters that have been updated in this run:
    public void updatedParametersDialog() {
        String comparison = currentParameters.compare(originalParameters, true);
        if (comparison.isEmpty()) {
            io.message("There are no changes from your current settings.\n\nPress "+A+" to continue.");
            poll.ok();
        } else {
            io.message("Here are all of the parameter updates from your current run. "
                    + "Double-tap the shift key in Android Studio, enter 'md.params' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
            poll.ok();
        }
    }

    // Show the SparkFun OTOS hardware and firmware version:
    public void otosVersionDialog() {
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        drive.opticalTracker.getVersionInfo(hwVersion, fwVersion);
        io.message("SparkFun OTOS hardware version: %d.%d, firmware version: %d.%d\n\n"
                        + "Press "+A+" to continue.",
                hwVersion.major, hwVersion.minor, fwVersion.major, fwVersion.minor);
        poll.ok();
    }

    public void retuneDialog() {
        io.message(Dialog.QUESTION_ICON + " Do you want to re-tune your robot "
            + "after a big hardware change? "
            + "This will walk you through the re-tuning step-by-step.\n"
            + "\n"
            + "Press "+A+" if yes, "+B+" to cancel.");
        if (poll.okCancel()) {
            nextRetuneIndex = 0; // Reset to the beginning
            updateTunerDependencies(Tuner.WHEEL_TEST); // Pretend we just finished the wheel test
        }
    }

    @Override
    public void runOpMode() {
        // Initialize member fields:
        io = new Io(gamepad1, telemetry);
        menu = new Menu(io);
        poll = new Poll();
        dialog = new Dialog();
        drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, zeroPose);
        currentParameters = new TuneParameters(drive);
        currentParameters.loadPassedState();
        originalParameters = currentParameters.createClone();

        while (!isStarted()) {
            io.begin();
            io.out("<big><big><big><big><big><big><b>Loony Tune!</b></big></big></big></big></big></big>\n");
            io.out("<big><big>By Swerve Robotics, Woodinville</big></big>\n");
            io.out("\n\n\n\n\n\n<big><big><big><big><big><b>Tap \u25B6 to begin");
            io.canvas().fillText("Loony Tune!", -32, 32, "", 0, false);
            io.end();
        }

        // Require that a button be pressed on the gamepad to continue. We require this
        // because enabling the gamepad on the DS after it's been booted causes an A press to
        // be sent to the app, and we don't want that to accidentally invoke a menu option.
        //
        // Skip the welcome message and initial button press if simulating:
        if (!WilyWorks.isSimulating) {
            io.setWelcomeMessage("This tuner requires FTC Dashboard. Wi-Fi connect your laptop "
                    + "to the robot and go to <u>http://192.168.43.1:8080/dash</u> in your web browser. "
                    + "\n\n"
                    + "Configure FTC Dashboard to show both the <b>Field</b> and "
                    + "<b>Telemetry</b> views by selecting either 'Field' or 'Custom' from the dropdown "
                    + "at its top. "
                    + "\n\n"
                    + "Follow the prompts in the FTC Dashboard Telemetry window while using the Driver "
                    + "Station's gamepad to navigate the UI. "
                    + "\n\n"
                    + "<b>Don't break your robot, press " + B + " at any time to stop the robot during a "
                    + "test!</b>"
                    + "\n\n"
                    + "<small><font color='#a0a0a0'>(If you really want to see the UI here on the Driver "
                    + "Station, press the Guide button in the middle of the gamepad.)</font></small>");
            io.message("<big><big><big><big><big>Press " + A + " to begin</big></big></big></big></big>");
            while (!isStopRequested() && !io.ok())
                io.redraw();
        }

        if ((drive.opticalTracker == null) ||
            ((drive.opticalTracker.getAngularUnit() != AngleUnit.RADIANS) ||
             (drive.opticalTracker.getLinearUnit() != DistanceUnit.INCH))) {
            io.message("The SparkFun OTOS must be present and configured for radians and inches.\n\n"
                    + "Press "+A+" to quit.");
            poll.ok();
            return; // ====>
        }

        // Dynamically build the list of tests:
        addTuner(Tuner.WHEEL_TEST,          this::wheelTest,                             "Wheel test (wheels, motors verification)");
        addTuner(Tuner.PUSH,                pushTuner::execute,                          "Push tuner (OTOS offset heading, linearScalar)");
        addTuner(Tuner.SPIN,                spinTuner::execute,                          "Spin tuner (trackWidthTicks, OTOS angularScalar, x/y offset)");
        addTuner(Tuner.TRACKING_TEST,       this::trackingTest,                          "Tracking test (OTOS verification)");
        addTuner(Tuner.ACCELERATING,        acceleratingTuner::execute,                  "Accelerating straight line tuner (kS, kV)");
        addTuner(Tuner.FEED_FORWARD,        feedForwardTuner::execute,                   "Interactive feed forward tuner (kV, kA)");
        addTuner(Tuner.LATERAL_MULTIPLIER,  lateralMultiplierTuner::execute,             "Lateral tuner (lateralInPerTick)");
        addTuner(Tuner.AXIAL_GAIN,          ()-> pidTuner.execute(PidTunerType.AXIAL),   "Interactive PiD tuner (axial gains)");
        addTuner(Tuner.LATERAL_GAIN,        ()-> pidTuner.execute(PidTunerType.LATERAL), "Interactive PiD tuner (lateral gains)");
        addTuner(Tuner.HEADING_GAIN,        ()-> pidTuner.execute(PidTunerType.HEADING), "Interactive PiD tuner (heading gains)");
        addTuner(Tuner.COMPLETION_TEST,     this::completionTest,                        "Completion test (overall verification)");
        addTuner(Tuner.RETUNE,              this::retuneDialog,                          "Re-tune");

        menu.addRunnable("More::Show accumulated parameter changes", this::updatedParametersDialog);
        menu.addRunnable("More::Show SparkFun OTOS version", this::otosVersionDialog);

        // Set the initial enable/disable status for the core widgets:
        updateTunerDependencies(Tuner.NONE);

        // Add more options if tuning is complete:
        if (widgets[Tuner.COMPLETION_TEST.index].isEnabled) {
            menu.addRunnable("More::Interactive PiD tuner (all gains)", () -> pidTuner.execute(PidTunerType.ALL));
            menu.addRunnable("More::Rotation test (verify trackWidthTicks)", this::rotationTest);

            menu.addRunnable("Examples::Spline", this::splineExample);
            menu.addRunnable("Examples::LineTo/Turn example", this::lineToTurnExample);
            menu.addRunnable("Examples::Line with rotation", this::lineWithRotationExample);
        }

        // Run the menu loop:
        while (opModeIsActive()) {
            io.message(menu.update());
        }
    }
}