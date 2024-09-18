/**
 * Looney Tune is a parameters tuner for robots using Road Runner.
 */

// Short-term:
//
// Long-term:
// @@@ Add dependency graph when a change is made
// @@@ Add LED support
// @@@ Add max-velocity/max-acceleration testing for both linear and angular
// @@@ Add offset test to extras
// @@@ Draw acceleration portion of feedback test in different color

package org.firstinspires.ftc.team417;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.wilyworks.common.WilyWorks;

import static java.lang.System.out;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
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
 * Class for remembering all of the tuned settings.
 * @noinspection AccessStaticViaInstance
 */
class TuneParameters {
    String robotName;
    MecanumDrive.Params params;

    // Get the settings from the current MecanumDrive object:
    public TuneParameters(MecanumDrive drive) {
        robotName = MecanumDrive.getBotName();
        params = drive.PARAMS;
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
    // return a string that tells the user
    boolean useHtml = false;
    String comparison = "";
    void compare(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, oldValue);
        String newString = String.format(format, newValue);
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
        if (!savedParameters.robotName.equals(robotName))
            return null; // Different robots, so discard
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
 * Class for encapsulating all Gui operations.
 * @noinspection StringConcatenationInsideStringBufferAppend, UnnecessaryUnicodeEscape, unused
 */
class Gui {
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed
    static private final String DESCRIPTOR_SEPARATOR = "::";
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double ADVANCE_DELAY = 0.1; // Seconds after any repeat to repeat again
    private static Gui gui; // Points to our own singleton object
    Gamepad gamepad; // Gamepad to use for control
    ArrayList<Widget> menuStack = new ArrayList<>(); // Stack of menus, the last is the current
    int lastInput; // Last quantized input (-1, 0 or 1)
    double nextAdvanceTime; // Time at which to advance the value

    abstract private static class Widget {
        String description;
        BooleanSupplier isEnabled;
        Widget(String descriptor) {
            // The description comes after the last separator:
            int lastIndex = descriptor.lastIndexOf(DESCRIPTOR_SEPARATOR);
            if (lastIndex != -1) {
                descriptor = descriptor.substring(lastIndex + DESCRIPTOR_SEPARATOR.length());
            }
            description = descriptor;
            isEnabled = ()->true; // Default is to be enabled
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
        public RunWidget(String descriptor, Runnable runnable, BooleanSupplier isEnabled) {
            super(descriptor);
            this.runnable = runnable;
            if (isEnabled != null)
                this.isEnabled = isEnabled;
        }
        public String string() {
            return isEnabled.getAsBoolean() ? description : "<font color='#808080'>" + description + "</font>";
        }
    }

    // Button press state:
    private final boolean[] buttonPressed = new boolean[11];
    private boolean buttonPress(boolean pressed, int index) {
        boolean press = pressed && !buttonPressed[index];
        buttonPressed[index] = pressed;
        return press;
    }

    // Button press status:
    boolean accept() { return buttonPress(gamepad.a, 0); }
    boolean cancel() { return buttonPress(gamepad.b, 1); }
    boolean xButton() { return buttonPress(gamepad.x, 2); }
    boolean yButton() { return buttonPress(gamepad.y, 3); }
    boolean up() { return buttonPress(gamepad.dpad_up, 4); }
    boolean down() { return buttonPress(gamepad.dpad_down, 5); }
    boolean left() { return buttonPress(gamepad.dpad_left, 6); }
    boolean right() { return buttonPress(gamepad.dpad_right, 7); }
    boolean leftTrigger() { return buttonPress(gamepad.left_trigger >= ANALOG_THRESHOLD, 8); }
    boolean rightTrigger() { return buttonPress(gamepad.right_trigger >= ANALOG_THRESHOLD, 9); }
    boolean leftBumper() { return buttonPress(gamepad.left_bumper, 10); }

    // Constructor:
    public Gui(Gamepad gamepad) {
        Gui.gui = this;
        this.gamepad = gamepad;

        // Create the root menu:
        menuStack.add(new MenuWidget(""));
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
            output.append("Dpad to navigate, "+LooneyTune.A+" to select");
        } else {
            for (int i = 1; i < menuStack.size(); i++) {
                if (i > 1)
                    output.append("\u00b7");
                output.append(menuStack.get(i).description);
            }
            output.append(", "+LooneyTune.A+" to select, "+LooneyTune.B+" to exit");
        }
        output.append("</big></big><br><small><small><br></small></small>"); // Leave half a line blank

        // Process dpad up and down with auto-repeat and clamping:
        MenuWidget menu = (MenuWidget) menuStack.get(menuStack.size() - 1);
        int input = gamepad.dpad_up ? -1 : (gamepad.dpad_down ? 1 : 0); // -1, 0 or 1
        if (input != lastInput) {
            nextAdvanceTime = time() + INITIAL_DELAY;
            lastInput = input;
            menu.current += lastInput;
        } else if (time() > nextAdvanceTime) {
            nextAdvanceTime = time() + ADVANCE_DELAY;
            menu.current += lastInput;
        }
        menu.current = Math.max(0, Math.min(menu.widgets.size() - 1, menu.current));

        // Now output the widgets:
        for (int i = 0; i < menu.widgets.size(); i++) {
            Widget widget = menu.widgets.get(i);
            if (i != menu.current)
                output.append("\u25c7 " + widget.string() + "\n");
            else
                // Highlight current item:
                output.append("<span style='background: #88285a'>\u25c6 " + widget.string() + "</span>\n");
        }

        Widget widget = menu.widgets.get(menu.current);
        if (cancel()) {
            if (menuStack.size() > 1) {
                // Pop up the menu stack:
                menuStack.remove(menuStack.size() - 1);
            }
        }
        else if (widget instanceof ToggleWidget) {
            ToggleWidget toggleWidget = (ToggleWidget) widget;
            if (accept()) {
                toggleWidget.value = !toggleWidget.value;
                toggleWidget.callback.accept(toggleWidget.value);
            }
        } else if (widget instanceof ListWidget) {
            ListWidget listWidget = (ListWidget) widget;
            boolean left = left();
            boolean right = right();
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
            if (accept()) {
                ActivationWidget activationWidget = (ActivationWidget) widget;
                activationWidget.callback.apply(true);
            }
        } else if (widget instanceof StatsWidget) {
            if (accept()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof MenuWidget) {
            if (accept()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof RunWidget) {
            RunWidget runWidget = (RunWidget) widget;
            if (accept()) {
                if ((runWidget.isEnabled == null) || (runWidget.isEnabled.getAsBoolean())) {
                    runWidget.runnable.run();
                }
            }
        }

        return output.toString();
    }

    // Add a new widget to the appropriate spot in the menu hierarchy:
    private void add(String descriptor, Widget newWidget) {
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
    }

    // Add a toggleable widget to the menu:
    /** @noinspection unused*/
    public static void addToggle(String descriptor, boolean initialValue, Consumer<Boolean> callback) {
        callback.accept(initialValue);
        gui.add(descriptor, new ToggleWidget(descriptor, initialValue, callback));
    }
    // Add a list widget to the menu:
    /** @noinspection unused*/
    public static void addList(String descriptor, String[] list, int initialIndex, BiConsumer<Integer, String> callback) {
        callback.accept(initialIndex, list[initialIndex]);
        gui.add(descriptor, new ListWidget(descriptor, initialIndex, list, callback));
    }
    // Add a widget that can only be activated:
    /** @noinspection unused*/
    public static void addActivation(String descriptor, Function<Boolean, String> callback) {
        callback.apply(true);
        gui.add(descriptor, new ActivationWidget(descriptor, callback));
    }
    /** @noinspection unused*/
    public static void addStats(String descriptor, Supplier<String> callback) {
        gui.add(descriptor, new StatsWidget(descriptor, callback));
    }
    // Add a widget that can be run:
    public static void addRunnable(String descriptor, Runnable callback, BooleanSupplier isEnabled) {
        gui.add(descriptor, new RunWidget(descriptor, callback, isEnabled));
    }
    public static void addRunnable(String descriptor, Runnable callback) {
        gui.add(descriptor, new RunWidget(descriptor, callback, ()->true));
    }
}

/**
 * Looney Tune's opMode class for running the tuning tests.
 *
 * @noinspection UnnecessaryUnicodeEscape, AccessStaticViaInstance, ClassEscapesDefinedScope
 */
@SuppressLint("DefaultLocale")
@TeleOp(name="Looney Tune", group="Tuning")
public class LooneyTune extends LinearOpMode {
    static final String A = "\ud83c\udd50"; // Symbol for the gamepad A button
    static final String B = "\ud83c\udd51"; // Symbol for the gamepad B button
    static final String X = "\ud83c\udd67"; // Symbol for the gamepad X button
    static final String Y = "\ud83c\udd68"; // Symbol for the gamepad Y button

    // Member fields referenced by every test:
    Gui gui;
    Dialogs dialogs;
    MecanumDrive drive;
    TuneParameters currentParameters;
    TuneParameters originalParameters;

    // Constants:
    final Pose2d zeroPose = new Pose2d(0, 0, 0);

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
                telemetry.addLine("YOUR CODE IS OUT OF SYNC WITH LOONEY TUNE");
                telemetry.addLine();
                telemetry.addLine("The code's configuration parameters don't match the last "
                        + "results saved in Looney Tune. To use the Looney Tune results, double-tap the shift key in Android "
                        + "Studio, enter 'md.params' to jump to the MecanumDrive Params constructor, "
                        + "then update as follows:");
                telemetry.addLine();
                telemetry.addLine(comparison);
                telemetry.addLine("Please update your code and restart now. Or, to proceed anyway and "
                        + "delete the Looney Tune results, triple-tap the BACK button on the gamepad.");
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
                telemetry.addLine("Looney Tune results have been overridden");
                telemetry.update();
            }
        }
    }

    enum Prompt { SAVE, EXIT, CANCEL }

    /**
     * Class for doing an animated preview of a Road Runner trajectory action.
     */
    static class TrajectoryPreviewer {
        final double PAUSE_TIME = 1.0; // Pause this many seconds between preview cycles
        Action sequentialAction;
        List<Action> actions = new ArrayList<>();
        Iterator<Action> actionIterator;
        Action currentAction = null; // Current action in the actions list
        double startTime = time();
        TrajectoryPreviewer(Action action) {
            sequentialAction = action;
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

        // Update the preview
        void update() {
            if (currentAction != null) {
                TelemetryPacket telemetry = MecanumDrive.getTelemetryPacket(false);
                Canvas canvas = telemetry.fieldOverlay();
                canvas.fillText("Preview", -19, 5, "", 0, false);
                sequentialAction.preview(canvas);
                canvas.setStroke("#c0c0c0"); // Gray
                double time = time();
                if (time > startTime) {
                    Pose2d pose = getTargetPose();
                    if (pose == null)
                        startTime = time + PAUSE_TIME; // Start new cycle after a delay
                    else
                        Drawing.drawRobot(canvas, pose);
                }
                MecanumDrive.sendTelemetryPacket(telemetry);
            }
        }
    }

    /**
     * Class that encapsulates the dialogs framework.
     */
    class Dialogs {
        // Show a message:
        void message(String message) {
            telemetryAdd(message);
            telemetryUpdate();
        }

        // Show a message, preview an optional trajectory, drive the robot, and wait for an A or
        // B button press. If accept (A) is pressed, return success. If cancel (B) is pressed,
        // return failure. The robot CAN be driven while waiting.
        boolean drivePrompt(String message) { return drivePrompt(null, message); }
        boolean drivePrompt(Action trajectory, String message) {
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(trajectory);
            boolean success = false;
            while (opModeIsActive() && !gui.cancel()) {
                previewer.update();
                message(message);
                if (gui.accept()) {
                    success = true;
                    break;
                }
                updateGamepadDriving();
                updateRotation();
            }
            stopMotors();
            drive.setPose(zeroPose); // Reset the pose once they stopped
            return success;
        }

        // Show a message and wait for an A or B button press. If accept (A) is pressed, return
        // success. If cancel (B) is pressed, return failure. The robot CANNOT be driven while
        // waiting.
        boolean staticPrompt(String message) {
            while (opModeIsActive() && !gui.cancel()) {
                message(message);
                if (gui.accept())
                    return true;
            }
            return false;
        }

        // Show a prompt asking to save. If accept (A) is pressed, return Prompt.SAVE. If
        // cancel (B) is pressed, return Prompt.CANCEL. If exit (X) is pressed, return Prompt.EXIT.
        Prompt savePrompt() {
            while (opModeIsActive()) {
                message("Do you want to save your results?\n\n"
                        + "Press "+A+" to save, "+B+" to cancel, "+X+" to exit without saving.");
                if (gui.accept())
                    return Prompt.SAVE;
                if (gui.xButton())
                    return Prompt.EXIT;
                if (gui.cancel())
                    return Prompt.CANCEL;
            }
            return Prompt.EXIT;
        }
    }

    // Return a string that represents the distance the test will run:
    String testDistance(int distance) {
        return String.format("%d inches (%.1f tiles)", distance, distance / 24.0);
    }

    // Convert telemetry newlines to HTML breaks so that FTC Dashboard renders the text properly:
    void telemetryAdd(String line) {
        telemetry.addLine(line.replace("\n", "<br>"));
    }

    // Send the telemetry packet to the Driver Station:
    void telemetryUpdate() {
        telemetry.update();
    }

    // Get a divider string for telemetry purposes:
    void putDivider(TelemetryPacket packet) {
        StringBuilder divider = new StringBuilder();
        for (int i = 0; i < 10; i++)
            divider.append("\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af");
        packet.put(divider.toString(), "");
    }

    // Run an Action but end it early if Cancel is pressed.
    // Returns True if it ran without cancelling, False if it was cancelled.
    private boolean runCancelableAction(Action action) {
        drive.runParallel(action);
        while (opModeIsActive() && !gui.cancel()) {
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
            dialogs.message("Press "+B+" to stop");
            boolean more = drive.doActionsWork(packet);
            if (more) {
                MecanumDrive.sendTelemetryPacket(packet);
            } else {
                // We successfully completed the Action!
                return true; // ====>
            }
        }
        // The user either pressed Cancel or End:
        drive.abortActions();
        stopMotors();
        return false;
    }

    // Road Runner expects the hardware to be in different states when using high-level MecanumDrive
    // functionality vs. its lower-level tuning functionality.
    private void configureToDrive(boolean enableRoadRunnerDefaults) {
        MecanumDrive.clearDashboardTelemetry();

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

    // Measure the optical linear scale and orientation:
    void pushTuner() {
        final int DISTANCE = 96; // Test distance in inches
        configureToDrive(false); // Don't use MecanumDrive

        double oldOffsetHeading = currentParameters.params.otos.offset.h;
        double oldLinearScalar = currentParameters.params.otos.linearScalar;
        if (oldLinearScalar == 0)
            oldLinearScalar = 1.0; // Can happen on the very first run, stock Road Runner sets to zero

        Action preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, -60, 0))
                .lineToX(DISTANCE/2.0)
                .build();
        if (dialogs.drivePrompt(preview,
                "Tune OTOS's <b>linearScalar</b> and <b>offset heading</b> by pushing the robot forward in a straight line "
                + "along a field wall for exactly "+testDistance(DISTANCE)+". To start, align the robot by hand "
                + "at its starting point along a wall. "
                + "\n\nPress "+A+" when in position, "+B+" to cancel.")) {

            double distance = 0;
            double heading = 0;

            drive.opticalTracker.resetTracking();

            boolean success = false;
            while (opModeIsActive() && !gui.cancel()) {
                if (gui.accept()) {
                    success = true;
                    break; // ====>
                }

                Pose2D pose = drive.opticalTracker.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                dialogs.message("Push forward exactly "+testDistance(DISTANCE)+" along the field wall.\n\n"
                    + String.format("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h))
                    + String.format("&ensp;Effective distance: %.2f\"\n", distance)
                    + String.format("&ensp;Heading angle: %.2f\u00b0\n", Math.toDegrees(heading))
                    + "\nPress "+A+" when you've finished pushing, "+B+" to cancel");
            }

            if (success) {
                // Avoid divide-by-zeroes on aborts:
                if (distance == 0)
                    distance = 0.001;

                // Calculate the new settings and undo the corrections that the OTOS sensor
                // was applying:
                double newLinearScalar = (DISTANCE / distance) * oldLinearScalar;
                double newOffsetHeading = normalizeAngle(heading + oldOffsetHeading);

                double linearScalarChange = Math.abs((oldLinearScalar - newLinearScalar)
                        / oldLinearScalar * 100.0); // Percentage
                double headingChange = normalizeAngle(Math.abs(oldOffsetHeading - newOffsetHeading));

                if (newLinearScalar < SparkFunOTOS.MIN_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of %d\". It can't measure more than %.1f\". "
                            + "Either you didn't push straight for "+testDistance(DISTANCE)+" or something is wrong "
                            + "with the sensor. ", distance, DISTANCE, DISTANCE / SparkFunOTOS.MIN_SCALAR);
                    message += "Maybe the distance of the sensor to the tile is less than 10.0 mm? ";
                    dialogs.staticPrompt(message + "\n\nAborted, press "+A+" to continue");
                } else if (newLinearScalar > SparkFunOTOS.MAX_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of %d\". It can't measure less than %.1f\". "
                            + "Either you didn't push straight for "+testDistance(DISTANCE)+" or something is wrong "
                            + "with the sensor. ", distance, DISTANCE, DISTANCE / SparkFunOTOS.MAX_SCALAR);

                    // If the measured distance is close to zero, don't bother with the following
                    // suggestion:
                    if (newLinearScalar < 1.5) {
                        message += "Maybe the distance of the sensor to the tile is more than 10.0 mm?";
                    }
                    dialogs.staticPrompt(message + "\n\nAborted, press "+A+" to continue");
                } else {
                    if (dialogs.staticPrompt(String.format("New offset heading %.2f\u00b0 is %.2f\u00b0 off from old.\n",
                                    Math.toDegrees(newOffsetHeading), Math.toDegrees(headingChange))
                            + String.format("New linear scalar %.3f is %.1f%% off from old.\n\n",
                                    newLinearScalar, linearScalarChange)
                            + "Use these results? Press "+A+" if they look good, "+B+" to cancel.")) {

                        TuneParameters newParameters = currentParameters.createClone();
                        newParameters.params.otos.linearScalar = newLinearScalar;
                        newParameters.params.otos.offset.h = newOffsetHeading;
                        acceptParameters(newParameters);

                        // Make sure the OTOS hardware is informed of the new parameters too:
                        setOtosHardware();
                    }
                }
            }
        }
    }

    // Prompt the user for how to set the new parameters and save them to the registry:
    public void acceptParameters(TuneParameters newParameters) {
        String comparison = newParameters.compare(currentParameters, true);
        if (comparison.isEmpty()) {
            dialogs.staticPrompt("The new results match your current settings.\n\nPress "+A+" to continue.");
        } else {
            MecanumDrive.PARAMS = newParameters.params;
            currentParameters = newParameters;
            currentParameters.save();
            dialogs.staticPrompt("Double-tap the shift key in Android Studio, enter '<b>md.params</b>' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
        }
    }

    // Show all of the parameters that have been updated in this run:
    public void showUpdatedParameters() {
        String comparison = currentParameters.compare(originalParameters, true);
        if (comparison.isEmpty()) {
            dialogs.staticPrompt("There are no changes from your current settings.\n\nPress "+A+" to continue.");
        } else {
            dialogs.staticPrompt("Here are all of the parameter updates from your current run. "
                    + "Double-tap the shift key in Android Studio, enter 'md.params' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
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

            updateRotation();
            if (duration == RAMP_TIME)
                break; // ===>
        }
    }

    // Stop all motors:
    void stopMotors() {
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        sleep(333); // Give the robot a little time to actually stop
    }

    // Structure to describe the center of rotation for the robot:
    static class Circle {
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
    static Circle fitCircle(List<Point> points, double centerX, double centerY) {
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
    static Point clusterCenter(List<Point> points) {
        int size = points.size();
        if (size == 0)
            return new Point(0, 0);

        Point sum = new Point(0, 0);
        for (Point point: points) {
            sum = sum.add(point);
        }
        return new Point(sum.x / size, sum.y / size);
    }

    // Persisted state for initiateSparkFunRotation and updateSparkFunRotation:
    double previousSparkFunHeading = 0;
    double accumulatedSparkFunRotation = 0;

    // Start tracking total amount of rotation:
    double initiateSparkFunRotation() {
        accumulatedSparkFunRotation = 0;
        previousSparkFunHeading = drive.opticalTracker.getPosition().h;
        return previousSparkFunHeading;
    }

    // Call this regularly to update the tracked amount of rotation:
    void updateRotation() { updateRotationAndGetPose(); }
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
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
        Canvas canvas = packet.fieldOverlay();
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

        MecanumDrive.sendTelemetryPacket(packet);
    }

    // This is the robot spin test for calibrating the optical sensor angular scale and offset:
    void spinTuner() {
        final double REVOLUTION_COUNT = 10.0; // Number of revolutions to use
        final double SPIN_POWER = 0.5; // Speed of the revolutions

        configureToDrive(true); // Use MecanumDrive

        // Zero these settings for the purpose of this test:
        drive.opticalTracker.setOffset(new Pose2D(0, 0, 0));
        drive.opticalTracker.setAngularScalar(0);

        Action preview = drive.actionBuilder(new Pose2d(0, -60, 0))
                .strafeTo(new Vector2d(0, -48))
                .turn(2*2*Math.PI)
                .strafeTo(new Vector2d(0, -60))
                .build();
        if (dialogs.drivePrompt(preview, "Tune <b>trackWidthTicks</b>, <b>otos.angularScalar</b> and <b>otos.offset position</b>. Position the robot against a wall, then drive "
                + String.format("it out so that the robot can rotate in place %.1f times, then drive ", REVOLUTION_COUNT)
                + "the robot against the wall again."
                + "\n\nFirst, carefully drive the robot to a wall and align it so that "
                + "it's facing forward. This marks the start orientation for calibration."
                + "\n\nDrive the robot to the start position, press "+A+" when ready, "+B+" to cancel")) {

            // Let the user position the robot:
            if (dialogs.drivePrompt("Now move the robot far enough away from the wall (and any objects) so "
                    + "that it can freely rotate in place."
                    + "\n\nPress "+A+" when ready for the robot to rotate, "+B+" to cancel")) {

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
                    telemetryAdd(String.format("%.2f rotations remaining, %d samples", rotationsRemaining, points.size()));
                    telemetryAdd("\nPress "+B+" to abort.");
                    telemetryUpdate();

                    // Update for next iteration of the loop:
                    Pose2D rawPose = updateRotationAndGetPose();
                    rawPosition = new Point(rawPose.x, rawPose.y);
                } while (opModeIsActive() && !gui.cancel());

                double endTime = time();

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                if ((success) && dialogs.drivePrompt("Now drive the robot to align it at the wall in the same "
                        + "place and orientation as it started."
                        + "\n\nDrive the robot to its wall position, press "+A+" when done, "+B+" to cancel")) {

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
                    processSpinResults(clusterCenter, circle, totalMeasuredRotation, distancePerRevolution);
                }
            }
        }

        // Restore the hardware settings:
        setOtosHardware();
    }

    // Process the spin results:
    void processSpinResults(Point clusterCenter, Circle center, double totalMeasuredRotation, double distancePerRevolution) {
        double totalMeasuredCircles = totalMeasuredRotation / (2 * Math.PI);
        double integerCircles = Math.round(totalMeasuredCircles);
        double angularScalar = integerCircles / totalMeasuredCircles;

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

        String results = String.format("Sensor thinks %.2f circles were completed.\n\n", totalMeasuredCircles);
        results += String.format("Cluster center: (%.2f, %.2f)\n", clusterOffset.x, clusterOffset.y);
        results += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", rawOffset.x, rawOffset.y, center.radius);
        results += String.format("Radius-corrected position: (%.2f, %.2f)\n", offset.x, offset.y);
        results += String.format("Angular scalar: %.4f\n", angularScalar);
        results += String.format("Track width: %.2f\"\n", trackWidth);
        results += "\n";

        // Do some sanity checking on the results:
        if ((Math.abs(offset.x) > 12) || (Math.abs(offset.y) > 12)) {
            dialogs.staticPrompt(results + "The results are bad, the calculated center-of-rotation is bogus.\n\n"
                    + "Aborted, press "+A+" to continue.");
        } else if  ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
            dialogs.staticPrompt(results + "The measured number of circles is bad. Did you properly align "
                    + "the robot on the wall the same way at both the start and end of this test?\n\n"
                    + "Aborted, press "+A+" to continue.");
        } else {
            TuneParameters newSettings = currentParameters.createClone();
            newSettings.params.otos.offset.x = offset.x;
            newSettings.params.otos.offset.y = offset.y;
            newSettings.params.otos.angularScalar = angularScalar;
            newSettings.params.trackWidthTicks = trackWidthTicks;

            if (dialogs.staticPrompt(results + "Use these results? Press "+A+" if they look good, "+B+" to discard them.")) {
                acceptParameters(newSettings);

                // We changed 'trackWidthTicks' so recreate the kinematics object:
                drive.initializeKinematics();
            }
        }
    }

    /**
     * Structure for describing the best-fit-line for a set of points.
     */
    static class BestFitLine {
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

    // Helper for acceleratingStraightLineTuner that drives an accelerating straight line
    // and samples voltage and velocity:
    /** @noinspection SameParameterValue*/
    ArrayList<Point> acceleratingStraightLine(double targetDistance, double direction) {
        final double VELOCITY_EPSILON = 2.0; // Inches/s
        final double POWER_FACTOR_ADDER_PER_SECOND = 0.1;
        final double MIN_POWER_FACTOR = 0.05;
        final double MAX_POWER_FACTOR = 0.9;

        // Reset the pose every time:
        drive.setPose(zeroPose);

        ArrayList<Point> points = new ArrayList<>();
        double startTime = time();
        while (opModeIsActive() && !gui.cancel()) {
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
            telemetryAdd(String.format("Inches remaining: %.1f, power: %.1f\n", remaining, powerFactor));
            telemetryAdd("Press "+B+" to abort.");
            telemetryUpdate();
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

    // Automatically calculate the kS and kV terms of the feed-forward approximation by
    // ramping up the velocity in a straight line. We increase power by a fixed increment.
    void acceleratingStraightLineTuner() {
        final int DISTANCE = 72; // Test distance in inches

        configureToDrive(true); // Set the brakes
        Action preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, 0, 0))
                .lineToX(DISTANCE/2.0, null, new ProfileAccelConstraint(-100, 10))
                .build();
        if (dialogs.drivePrompt(preview, "Tune <b>kS</b> and <b>kV</b>. "
                + "The robot will drive forward and backward for up to " + testDistance(DISTANCE) + ". "
                + "It will start slowly but get faster and faster in each direction. "
                + "\n\nDrive the robot to a good spot, press "+A+" to start, "+B+" to cancel.")) {

            ArrayList<Point> resultHistory = new ArrayList<>(); // x is kS, y is kV
            resultHistory.add(new Point(currentParameters.params.kS, currentParameters.params.kV));

            ArrayList<Point> acceptedSamples = new ArrayList<>(); // x is velocity, y is power
            Point max = new Point(0, 0); // x is velocity max, y is power max
            while (opModeIsActive()) {
                // Clear the FTC Dashboard field:
                TelemetryPacket telemetryPacket = MecanumDrive.getTelemetryPacket(false);
                MecanumDrive.sendTelemetryPacket(telemetryPacket);

                // Do one forward and back run:
                ArrayList<Point> forwardSamples = acceleratingStraightLine(DISTANCE,  1.0);
                if (forwardSamples != null) {
                    ArrayList<Point> reverseSamples = acceleratingStraightLine(DISTANCE, -1.0);
                    if (reverseSamples != null) {
                        findMaxValues(max, forwardSamples);
                        findMaxValues(max, reverseSamples);
                        if (max.x == 0) {
                            dialogs.staticPrompt("The optical tracking sensor returned only zero velocities. "
                                    + "Is it working properly?"
                                    + "\n\nAborted, press " + A + " to continue.");
                            break; // ====>
                        } else {
                            // Draw the results to the FTC dashboard:
                            TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
                            Canvas canvas = packet.fieldOverlay();

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

                            MecanumDrive.sendTelemetryPacket(packet);

                            double kS = bestFitLine.intercept;
                            double kV = bestFitLine.slope * currentParameters.params.inPerTick;

                            StringBuilder builder = new StringBuilder("Check out the graph on FTC Dashboard! The x axis is "
                                    + String.format("velocity going up to %.1f\"/s. The y axis is ", max.x)
                                    + String.format("voltage going up to %.2fV.\n\n", max.y)
                                    + "History of results:\n\n");

                            for (Point result : resultHistory) {
                                builder.append(String.format("&ensp;kS: %.03f, kV: %.03f\n", result.x, result.y));
                            }
                            builder.append(String.format("&ensp;<b>kS: %.03f, kV: %.03f</b> (this run)\n\n", kS, kV));
                            builder.append("If this run looks good, press " + A + " to accept, " + B + " to discard it.");
                            if (dialogs.staticPrompt(builder.toString())) {
                                resultHistory.add(new Point(kS, kV));
                                acceptedSamples = combinedSamples;
                            }
                        }
                    }
                }

                if (!dialogs.drivePrompt("Drive to reposition the robot, "
                        + "press "+A+" to start another run, "+B+" to exit. "
                        + "Every additional consecutive run helps the results converge.")) {

                    // Don't ask if they want to save if they didn't get any new results!
                    if (resultHistory.size() == 1)
                        break; // ====>
                    Prompt prompt = dialogs.savePrompt();
                    if (prompt == Prompt.EXIT)
                        break; // ====>
                    if (prompt == Prompt.SAVE) {
                        // Save the newest result:
                        Point result = resultHistory.get(resultHistory.size() - 1);
                        TuneParameters newParameters = currentParameters.createClone();
                        newParameters.params.kS = result.x;
                        newParameters.params.kV = result.y;
                        acceptParameters(newParameters);
                        break; // ====>
                    }
                }
            }
        }
    }

    // Test the robot motors.
    void wheelDebugger() {
        String[] motorDescriptions = { "leftFront", "leftBack", "rightBack", "rightFront" };
        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        int i = 0;

        stopMotors();
        while (opModeIsActive() && !gui.cancel()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            String description = motorDescriptions[i];
            telemetryAdd(String.format("This tests every motor individually, now testing '%s'.\n\n", description)
                + String.format("&emsp;%s.setPower(%.2f)\n\n", description, power)
                + "Press right trigger for forward, left trigger for reverse, "+A+" for next motor, "+B+" to cancel.");
            telemetryUpdate();

            motors[i].setPower(power);
            if (gui.accept()) {
                motors[i].setPower(0);
                i += 1;
                if (i >= motors.length)
                    i = 0;
            }
        }

        drive.setPose(zeroPose);
        stopMotors();
    }

    // Drive the robot around.
    void driveTest() {
        // Choose a good initial place on this year's field to always start from. For Into the
        // Deep, the (0, 0) origin is blocked by the 'submersible':
        Pose2d startPose = new Pose2d(-48, 0, 0);

        configureToDrive(true); // Do use MecanumDrive
        drive.setPose(startPose);
        Pose2d previousPose = startPose;
        double totalDistance = 0; // Inches
        double totalRotation = 0; // Radians
        String lastSeenStatus = ""; // Most recently reported non-zero status from the OTOS

        double baselineImu = 0; // IMU heading when the baseline was set
        Pose2d baselinePose = null; // Position where the baseline was set
        while (opModeIsActive() && !gui.cancel()) {
            if (gui.leftBumper()) {
                wheelDebugger();
                drive.setPose(startPose);
                baselinePose = null;
                previousPose = startPose;
            }
            if (gui.xButton()) {
                drive.setPose(startPose);
                baselinePose = null;
                previousPose = startPose;
            }
            if (gui.yButton()) {
                baselineImu = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                baselinePose = drive.pose;
                totalDistance = 0;
                totalRotation = 0;
            }

            updateGamepadDriving();
            drive.updatePoseEstimate();

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
            if (!currentStatus.isEmpty())
                lastSeenStatus = currentStatus;

            // Now that updatePoseEstimate is called, get the new pose and track the distance
            // traveled:
            Pose2d pose = drive.pose;
            totalDistance += Math.hypot(
                    pose.position.x - previousPose.position.x,
                    pose.position.y - previousPose.position.y);
            totalRotation += Math.abs(pose.heading.toDouble() - previousPose.heading.toDouble());
            previousPose = pose;

            TelemetryPacket packet = MecanumDrive.getTelemetryPacket(true);
            String message = "Use the controller to drive the robot around.\n\n";

            if (drive.opticalTracker != null) {
                MecanumDrive.Params.Otos otos = currentParameters.params.otos;
                if ((otos.linearScalar == 0) || (otos.angularScalar == 0) ||
                    (otos.offset.x == 0) || (otos.offset.y == 0)) {

                    message += "<font color='#e84e4f'>"
                        + "WARNING: The optical sensor's parameters haven't yet been fully "
                        + "tuned, so the robot's pose and its location shown in FTC Dashboard "
                        + "will be off.</font>\n\n";
                }
            }
            message += String.format("Pose: (%.2f\", %.2f\"), %.2f\u00b0\n",
                        pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            if (currentStatus.isEmpty()) {
                if (lastSeenStatus.isEmpty())
                    message += "OTOS status: Good!\n";
                else
                    message += String.format("OTOS status: Good now, but was %s\n", lastSeenStatus);
            } else {
                message += String.format("OTOS status: %s\n", currentStatus);
            }

            if (baselinePose != null) {
                double dx = pose.position.x - baselinePose.position.x;
                double dy = pose.position.y - baselinePose.position.y;
                double otosTheta = normalizeAngle(pose.heading.toDouble() - baselinePose.heading.toDouble());
                double imuTheta = normalizeAngle(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - baselineImu);

                message += "\nError analysis if back at the baseline position:\n\n"
                    + String.format("&ensp;Offset: (%.2f\", %.2f\"), %.2f\u00b0, IMU: %.2f\u00b0\n",
                        dx, dy, Math.toDegrees(otosTheta), Math.toDegrees(imuTheta));

                if ((totalDistance != 0) && (totalRotation != 0)) {
                    double distanceError = Math.abs(dx) / totalDistance;
                    double rotationError = Math.abs(otosTheta) / totalDistance;
                    message += String.format("&ensp;Distance error: %.2f%%\n", distanceError * 100);
                    message += String.format("&ensp;Rotation error: %.3f\u00b0/inch\n", rotationError);
                }
            }

            dialogs.message(message + "\nPress left bumper to debug the wheels, "+B+" to exit, "+X+" to reset the pose, "+Y+" to set the baseline pose.");

            Canvas canvas = packet.fieldOverlay();
            if (baselinePose != null) {
                canvas.setStroke("#c0c0c0");
                Drawing.drawRobot(canvas, baselinePose);
            }
            canvas.setStroke("#3F51B5");
            Drawing.drawRobot(canvas, pose);
            MecanumDrive.sendTelemetryPacket(packet);

        }
        stopMotors();
    }

    // Compute the average of the singleton value plus the values of an array:
    double average(double value, List<Double> values) {
        for (double x: values) {
            value += x;
        }
        return value / (values.size() + 1);
    }

    // Tuner for the lateral multiplier on Mecanum drives.
    void lateralTuner() {
        final int DISTANCE = 72; // Test distance in inches
        configureToDrive(true); // Do use MecanumDrive

        Action preview = drive.actionBuilder(new Pose2d(0, -DISTANCE/2.0, 0))
                .strafeTo(new Vector2d(0, DISTANCE/2.0))
                .strafeTo(new Vector2d(0, -DISTANCE/2.0))
                .build();
        if (dialogs.drivePrompt(preview, "Tune <b>lateralInPerTick</b>. The robot will strafe left and right for "
                + testDistance(DISTANCE) + ". "
                + "\n\nDrive the robot to position, press "+A+" to start, "+B+" to cancel.")) {

            double startLateralInPerTick = currentParameters.params.lateralInPerTick;

            // Disable the PID gains so that the distance traveled isn't corrected:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.lateralGain = 0;
            testParameters.params.lateralVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            // Now recreate the Kinematics object based on the new settings:
            drive.initializeKinematics();

            // Accelerate and decelerate slowly so we don't overshoot:
            ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(-10, 15);

            ArrayList<Double> history = new ArrayList<>();
            String resultsMessage = "";
            while (opModeIsActive()) {
                // Strafe left and then right:
                drive.setPose(zeroPose);
                if (runCancelableAction(drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE), null, accelConstraint)
                        .build())) {

                    double actualDistance1 = Math.hypot(drive.pose.position.x, drive.pose.position.y);

                    drive.setPose(zeroPose);
                    if (runCancelableAction(drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, -DISTANCE), null, accelConstraint)
                            .build())) {

                        double actualDistance2 = Math.hypot(drive.pose.position.x, drive.pose.position.y);

                        double multiplier1 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance1 / DISTANCE);
                        double multiplier2 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance2 / DISTANCE);

                        if (Math.min(multiplier1, multiplier2) < 0.25) {
                            dialogs.staticPrompt("The measured distance is too low to be correct. "
                                    + "Did it not move, or is the distance sensor not working properly?"
                                    + "\n\nPress " + A + "to continue.");
                            break; // ====>
                        }

                        double multiplier = (multiplier1 + multiplier2) / 2.0; // Compute the average

                        // Compute the average of all results for the new lateralInPerTick value:
                        double newLateralInPerTick = average(multiplier, history);
                        history.add(newLateralInPerTick);

                        StringBuilder builder = new StringBuilder(String.format(
                                "The robot drove %.1f and %.1f inches, respectively.\n\n", actualDistance1, actualDistance2));
                        builder.append("Test results, with the newest last:\n\n");
                        builder.append(String.format("&ensp;lateralInPerTick: %.03f\n", startLateralInPerTick));
                        for (Double lateralInPerTick : history) {
                            builder.append(String.format("&ensp;lateralInPerTick: %.03f\n", lateralInPerTick));
                        }
                        resultsMessage = builder + "\n";

                        // Adopt the new settings:
                        testParameters.params.lateralInPerTick = newLateralInPerTick;
                        MecanumDrive.PARAMS = testParameters.params;
                        drive.initializeKinematics();
                    }
                }

                if (!dialogs.drivePrompt(resultsMessage + "Drive to reposition the robot (it may go farther this time), "
                        + "press " + A + " to start another run, " + B + " to exit. ")) {

                    Prompt prompt = dialogs.savePrompt();
                    if (prompt == Prompt.EXIT)
                        break; // ====>
                    if (prompt == Prompt.SAVE) {
                        // Don't accept 'testParameters' because that has gains set to zero!
                        TuneParameters newParameters = currentParameters.createClone();
                        newParameters.params.lateralInPerTick = history.get(history.size() - 1);
                        acceptParameters(newParameters);
                        break; // ====>
                    }
                }
            }

            // Restore the kinematics:
            MecanumDrive.PARAMS = currentParameters.params;
            drive.initializeKinematics();
        }
    }

    // Tune the kV and kA feed forward parameters:
    void interactiveFeedForwardTuner() {
        final int DISTANCE = 72; // Test distance in inches
        configureToDrive(false); // Don't use MecanumDrive

        // Disable all lateral gains so that backward and forward behavior is not affected by the
        // PID/Ramsete algorithm. It's okay for the axial and rotation gains to be either zero
        // or non-zero:
        TuneParameters testParameters = currentParameters.createClone();
        testParameters.params.lateralGain = 0;
        testParameters.params.lateralVelGain = 0;
        MecanumDrive.PARAMS = testParameters.params;

        int inputIndex = 0; // 0 means we're inputting kV, 1 means kA
        NumericInput vInput = new NumericInput(drive.PARAMS, "kV", -2, 3, 0.000001, 20);
        NumericInput aInput = new NumericInput(drive.PARAMS, "kA", -3, 4, 0, 1);

        Action preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, 0, 0))
                .lineToX(DISTANCE/2.0)
                .lineToX(-DISTANCE/2.0)
                .build();
        if (dialogs.drivePrompt(preview, "Tune <b>kV</b> and <b>kA</b> with FTC Dashboard. "
                + "The robot will drive forwards then backwards for " + testDistance(DISTANCE) + ". "
                + "Follow <u><a href='https://learnroadrunner.com/feedforward-tuning.html#tuning'>LearnRoadRunner's guide</a></u>.\n\n"
                + "Press "+A+" to start, "+B+" to cancel")) {

            // Trigger a reset the first time into the loop:
            double profileStartTime = 0; // Profile start time, in seconds
            double cycleStartTime = 0; // Cycle start time, in second (each cycle consumes 2 profiles)
            double maxVelocityFactor = 0.8;
            TimeProfile profile = null;
            boolean movingForwards = false;
            int queuedAButtons = 0;
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

            while (opModeIsActive()) {
                // Process the gamepad numeric input:
                telemetryAdd("Tune the feed forward constants:\n");
                if (inputIndex == 0) {
                    telemetryAdd(String.format("&emsp;kV: <big><big>%s</big></big>&emsp;kA: %s\n", vInput.update(), aInput.get()));
                    telemetryAdd("View the graph in FTC Dashboard and adjust "
                            + "<b>kV</b> to make the horizontal lines as close as possible in height. "
                            + "<b>vTarget</b> is green, <b>vActual</b> is blue, <i>kV = vTarget / vActual</i>.\n");
                } else {
                    telemetryAdd(String.format("&emsp;kV: %s&emsp;kA: <big><big>%s</big></big>\n", vInput.get(), aInput.update()));
                    telemetryAdd("View the graph in FTC Dashboard and adjust "
                            + "<b>kA</b> to shift <b>vActual</b> left and right so the angled lines overlap "
                            + "where the robot accelerates. Don't worry about the deceleration portions.\n");
                }

                if (gui.accept())
                    queuedAButtons++; // Let the a-button be queued up even while running
                if (gui.yButton())
                    inputIndex ^= 1; // Toggle the index

                // If there's a profile, that means we're actively moving:
                if (profile != null) {
                    telemetryAdd("Press "+B+" to cancel");
                    telemetryUpdate();

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

                        TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
                        Canvas canvas = packet.fieldOverlay();
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
                        packet.put("vActual", actualVelocity);
                        packet.put("vTarget", v.get(0));
                        putDivider(packet);
                        MecanumDrive.sendTelemetryPacket(packet);
                    }

                    // Set the motors to the appropriate values:
                    MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

                    double power = feedForward.compute(v) / drive.voltageSensor.getVoltage();
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));

                    if (gui.cancel()) {
                        // Cancel the current cycle but remain in this test:
                        stopMotors();
                        queuedAButtons = 0;
                        profile = null;
                    }
                } else {
                    telemetryAdd("Use the triggers to change velocity to lengthen or shorten the horizontal lines. "
                            +  String.format("Max velocity is <b>%.0f%%</b>.\n", maxVelocityFactor * 100.0));

                    telemetryAdd("Dpad up/down to change the value, left/right to move "
                            + "the cursor, "+Y+" to switch input, triggers to change max velocity, "
                            + A+" to run on the robot, "+X+" to exit.");
                    telemetryUpdate();

                    updateGamepadDriving();

                    if (gui.xButton()) {
                        Prompt prompt = dialogs.savePrompt();
                        if (prompt == Prompt.SAVE) {
                            // Restore the state that we zeroed to run the test:
                            testParameters.params.lateralGain = currentParameters.params.lateralGain;
                            testParameters.params.lateralVelGain = currentParameters.params.lateralVelGain;
                            acceptParameters(testParameters);
                            break; // ====>
                        } else if (prompt == Prompt.EXIT) {
                            if (dialogs.staticPrompt("Are you sure you want to exit without saving?\n\n"
                                    + "Press "+A+" to exit without saving, "+B+" to cancel."))
                                break; // ====>
                        }
                    }
                    if (gui.leftTrigger()) {
                        maxVelocityFactor = Math.max(maxVelocityFactor - 0.1, 0.2);
                        maxDuration = 0; // A different speed will have a different window size
                    }
                    if (gui.rightTrigger()) {
                        maxVelocityFactor = Math.min(maxVelocityFactor + 0.1, 1.0);
                        maxDuration = 0; // A different speed will have a different window size
                    }
                    if (queuedAButtons > 0) {
                        queuedAButtons--;
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
        MecanumDrive.clearDashboardTelemetry();
    }

    /**
     * Class to handle gamepad input of decimal numbers.
     */
    class NumericInput {
        final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
        final double ADVANCE_DELAY = 0.15; // Seconds after any repeat to repeat again

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
            if (gui.left()) {
                digit = Math.min(digit + 1, 2);
            }
            if (gui.right()) {
                digit = Math.max(digit - 1, -decimalDigits);
            }
            if ((digit > 0) && (Math.pow(10, digit) > Math.abs(value))) {
                digit--;
            }

            // Advance the value according to the thumb stick state:
            int input = gui.gamepad.dpad_up ? 1 : (gui.gamepad.dpad_down ? -1 : 0); // -1, 0 or 1
            if (input != lastInput) {
                nextAdvanceTime = time() + INITIAL_DELAY;
                lastInput = input;
                value += lastInput * Math.pow(10, digit);
            } else if (time() > nextAdvanceTime) {
                nextAdvanceTime = time() + ADVANCE_DELAY;
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

        // Get the current value without updating:
        String get() {
            return String.format(showFormat, value);
        }
    }

    // Types of interactive PiD tuners:
    enum PidTunerType { AXIAL, LATERAL, HEADING }

    // Adjust the Ramsete/PID values:
    void interactivePidTuner(PidTunerType type) {
        final int DISTANCE = 48; // Test distance in inches
        configureToDrive(true); // Do use MecanumDrive

        TuneParameters testParameters = currentParameters.createClone();
        MecanumDrive.PARAMS = testParameters.params;
        String description, gainName, velGainName;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(zeroPose);
        Action preview;
        if (type == PidTunerType.AXIAL) {
            description = "Tune the <b>axial gains</b>. The robot will drive forwards and then backwards " + testDistance(DISTANCE) + ". ";
            trajectory = trajectory.lineToX(DISTANCE).lineToX(0);
            gainName = "axialGain";
            velGainName = "axialVelGain";
            preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, 0, 0))
                    .lineToX(DISTANCE/2.0)
                    .lineToX(-DISTANCE/2.0)
                    .build();

        } else if (type == PidTunerType.LATERAL) {
            description = "Tune the <b>lateral gains</b>. The robot will strafe left and then right " + testDistance(DISTANCE) + ". ";
            trajectory = trajectory.strafeTo(new Vector2d(0, DISTANCE)).strafeTo(new Vector2d(0, 0));
            gainName = "lateralGain";
            velGainName = "lateralVelGain";
            preview = drive.actionBuilder(new Pose2d(0, -DISTANCE/2.0, 0))
                    .strafeTo(new Vector2d(0, DISTANCE/2.0))
                    .strafeTo(new Vector2d(0, -DISTANCE/2.0))
                    .build();

        } else {
            description = "Tune the <b>heading gains</b>. The robot will rotate in place 180\u00b0 counterclockwise and then clockwise. "; // Degree symbol
            trajectory = trajectory.turn(Math.PI).turn(-Math.PI);
            gainName = "headingGain";
            velGainName = "headingVelGain";
            preview = drive.actionBuilder(zeroPose)
                    .turn(Math.PI)
                    .turn(-Math.PI)
                    .build();
        }
        if (dialogs.drivePrompt(preview, description + "\n\nPress "+A+" to start, "+B+" to cancel")) {

            int inputIndex = 0;
            int queuedAButtons = 0;
            NumericInput gainInput = new NumericInput(drive.PARAMS, gainName, -1, 2, 0, 20);
            NumericInput velGainInput = new NumericInput(drive.PARAMS, velGainName, -1, 2, 0, 20);

            while (opModeIsActive()) {
                // Drive:
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
                boolean more = drive.doActionsWork(packet);

                // Update the display:
                telemetryAdd("Tune the gains:\n");
                if (inputIndex == 0) {
                    telemetryAdd(String.format("&emsp;%s: <big><big>%s</big></big>&emsp;%s: %s\n",
                            gainName, gainInput.update(), velGainName, velGainInput.get()));
                } else {
                    telemetryAdd(String.format("&emsp;%s: %s&emsp;%s: <big><big>%s</big></big>\n",
                            gainName, gainInput.get(), velGainName, velGainInput.update()));
                }

                telemetryAdd("As the gains increase, the circles should align and the measured "
                        + String.format("error should decrease. First, increase <b>%s</b> until the robot starts ", gainName)
                        + String.format("to shake. Then switch to <b>%s</b> and increase it to minimize ", velGainName)
                        + "shaking but don't overdo it. "
                        + String.format("Then switch back to <b>%s</b> and try to increase it even more.\n", gainName));

                // Compute the relevant error:
                double error;
                String errorString;
                if (type == PidTunerType.AXIAL) {
                    error = drive.pose.position.x - drive.targetPose.position.x;
                    errorString = String.format("%.2f\"", error);
                } else if (type == PidTunerType.LATERAL) {
                    error = drive.pose.position.y - drive.targetPose.position.y;
                    errorString = String.format("%.2f\"", error);
                } else {
                    error = Math.toDegrees(normalizeAngle(drive.pose.heading.toDouble()
                                                        - drive.targetPose.heading.toDouble()));
                    errorString = String.format("%.2f\u00b0", error);
                }

                packet.put("Error", error); // Make the error graphable
                putDivider(packet);

                if (gui.accept())
                    queuedAButtons++; // Let the x-button be queued up even while running
                if (gui.yButton())
                    inputIndex ^= 1; // Toggle the index

                if (more) {
                    telemetryAdd("Current error: " + errorString + ".\n");
                    telemetryAdd("Press "+B+" to cancel");
                    telemetryUpdate();

                    // Only send the packet if there's more to the trajectory, otherwise the
                    // field view will be erased once the trajectory terminates:
                    MecanumDrive.sendTelemetryPacket(packet);

                    if (gui.cancel()) {
                        // Cancel the current cycle but remain in this test:
                        drive.abortActions();
                        queuedAButtons = 0;
                    }
                } else {
                    telemetryAdd("Last error: " + errorString + ".\n");
                    telemetryAdd("Dpad up/down to change the value, left/right to move "
                        + "the cursor, "+Y+" to switch input, "+A+" to run on the robot, "+X+" to exit.");
                    telemetryUpdate();

                    updateGamepadDriving();

                    if (gui.xButton()) {
                        Prompt prompt = dialogs.savePrompt();
                        if (prompt == Prompt.SAVE) {
                            acceptParameters(testParameters);
                            break; // ====>
                        } else if (prompt == Prompt.EXIT) {
                            if (dialogs.staticPrompt("Are you sure you want to exit without saving?\n\n"
                                    + "Press "+A+" to exit without saving, "+B+" to cancel."))
                                break; // ====>
                        }
                    }

                    // If there is no more actions, let the A button start a new one.
                    if (queuedAButtons > 0) {
                        queuedAButtons--;
                        stopMotors(); // Stop the user's driving
                        drive.setPose(zeroPose);
                        if (type == PidTunerType.HEADING) {
                            // An apparent Road Runner build prevents a trajectory from being reused:
                            drive.runParallel(drive.actionBuilder(zeroPose).turn(Math.PI).turn(-Math.PI).build());
                        } else {
                            drive.runParallel(trajectory.build());
                        }
                    }
                }
            }
            drive.abortActions();
            stopMotors();

        }
        MecanumDrive.PARAMS = currentParameters.params;
    }

    // Simple verification test for 'TrackWidth':
    void rotationTest() {
        configureToDrive(true); // Do use MecanumDrive

        if (dialogs.drivePrompt("To test 'trackWidthTicks', the robot will turn in-place for two complete "
                + "rotations.\n\nPress "+A+" to start, "+B+" to cancel.")) {

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
            runCancelableAction(action);

            dialogs.staticPrompt("The robot should be facing the same direction as when it started. It it's "
                    + "not, run the spin tuner again to re-tune 'trackWidthTicks'."
                    + "\n\nPress "+A+" to continue.");

            // Restore the parameters:
            MecanumDrive.PARAMS = currentParameters.params;
        }
    }

    // Run a simple trajectory with a preview and an option to disable odometry. The message
    // can be null:
    void runTrajectory(Action action) { runTrajectory(action, null);}
    void runTrajectory(Action action, String promptMessage) {
        configureToDrive(true); // Do use MecanumDrive
        TrajectoryPreviewer preview = new TrajectoryPreviewer(action);

        boolean useOdometry = true;
        while (opModeIsActive() && !gui.cancel()) {
            String message = promptMessage;
            if (message == null)
                message = "The robot will run the trajectory shown in FTC Dashboard.";
            if (useOdometry)
                message += "\n\nRunning with normal odometry correction. Press the left bumper to "
                        + "disable so as to test how well all the non-odometry settings are tuned. "
                        + "If well tuned, the robot should drive close to the intended path. ";
            else
                message += "\n\nRunning <b>without</b> odometry correction. Press the left bumper to re-enable.";

            dialogs.message(message + "\n\nPress "+A+" to start, "+B+" to cancel, left bumper to toggle odometry");
            updateGamepadDriving();
            preview.update();

            if (gui.leftBumper()) {
                useOdometry = !useOdometry;
            }
            if (gui.accept()) {
                stopMotors();
                drive.setPose(zeroPose);
                if (useOdometry) {
                    runCancelableAction(action);
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
                    runCancelableAction(action);
                    MecanumDrive.PARAMS = currentParameters.params;
                }
                break; // ====>
            }
        }
        stopMotors();
        drive.setPose(zeroPose); // Reset the pose once they stopped
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
                + "It needs half a tile clearance on either side. "
                + "\n\nDrive the robot to a good spot, press "+A+" to start, "+B+" to cancel";
        runTrajectory(action, message);
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

    @Override
    public void runOpMode() {
        // Set the display format to use HTML:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        gui = new Gui(gamepad1);
        dialogs = new Dialogs();
        drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, zeroPose);
        currentParameters = new TuneParameters(drive);
        originalParameters = currentParameters.createClone();

        if ((drive.opticalTracker != null) &&
            ((drive.opticalTracker.getAngularUnit() != AngleUnit.RADIANS) ||
             (drive.opticalTracker.getLinearUnit() != DistanceUnit.INCH))) {
            dialogs.staticPrompt("The SparkFun OTOS must be present and configured for radians and inches.");
            return; // ====>
        }

        // Clear the Dashboard map:
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket(false);
        Canvas canvas = packet.fieldOverlay();
        canvas.fillText("Looney Tune!", -32, 0, "", 0, false);
        MecanumDrive.sendTelemetryPacket(packet);

        // Dynamically build the list of tests:
        gui.addRunnable("Drive test (motors)", this::driveTest);
        if (drive.opticalTracker != null) {
            // Basic tuners:
            gui.addRunnable("Push tuner (OTOS offset heading, linearScalar)", this::pushTuner);
            gui.addRunnable("Accelerating straight line tuner (kS and kV)", this::acceleratingStraightLineTuner,
                ()->drive.PARAMS.otos.linearScalar != 0);
            gui.addRunnable("Interactive feed forward tuner (kV and kA)", this::interactiveFeedForwardTuner,
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Lateral tuner (lateralInPerTick)", this::lateralTuner,
                    ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Spin tuner (trackWidthTicks, OTOS angularScalar, offset)", this::spinTuner,
                    ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (axialGain)", ()->interactivePidTuner(PidTunerType.AXIAL),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (lateralGain)", ()->interactivePidTuner(PidTunerType.LATERAL),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.lateralInPerTick != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (headingGain)", ()->interactivePidTuner(PidTunerType.HEADING),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.trackWidthTicks != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Completion test (overall verification)", this::completionTest,
                ()->drive.PARAMS.axialGain != 0 && drive.PARAMS.lateralGain != 0 && drive.PARAMS.headingGain != 0);

            // Extras:
            gui.addRunnable("Extras::Show accumulated parameter changes", this::showUpdatedParameters);
            gui.addRunnable("Extras::Rotation test (verify trackWidthTicks)", this::rotationTest,
                    ()->drive.PARAMS.trackWidthTicks != 0);

            // Examples:
            gui.addRunnable("Examples::Spline", this::splineExample);
            gui.addRunnable("Examples::LineTo/Turn example", this::lineToTurnExample);
        }

        // Remind the user to press Start on the Driver Station, then press A on the gamepad.
        // We require the latter because enabling the gamepad on the DS after it's been booted
        // causes an A press to be sent to the app, and we don't want that to accidentally
        // invoke a menu option:
        dialogs.message("<big><big><big><big><big><big><big><big><b>Press \u25B6");
        waitForStart();
        dialogs.staticPrompt("<big><big><big><big><big><big><big><b>Press Gamepad "+A+" or "+B);

        while (opModeIsActive()) {
            telemetryAdd(gui.update());
            telemetryUpdate();
        }
    }
}