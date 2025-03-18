/**
 * Loony Tune is a parameters tuner for robots using Road Runner with Mecanum drives and the
 * SparkFun Optical Tracking Odometry Sensor.
 */

package org.firstinspires.ftc.team417.roadrunner;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import static org.swerverobotics.ftc.GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED;
import static org.swerverobotics.ftc.GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED;
import static org.swerverobotics.ftc.GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
import static org.swerverobotics.ftc.GoBildaPinpointDriver.DeviceStatus.NOT_READY;
import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;
import android.util.Log;

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
import com.qualcomm.robotcore.util.RobotLog;
import com.wilyworks.common.WilyWorks;

import static java.lang.System.out;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.swerverobotics.ftc.GoBildaPinpointDriver;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Class for debugging assistance.
 * @noinspection unused
 */
class Debug {
    // Code assertions are very valuable in software development but Java's built-in Debug.assertion()
    // causes FTC robots to immediately reboot when the assertion fires. Logcat will show
    // the assertion, but that's very rude behavior. This version, in contrast, spews a sticky
    // red message to the Driver Station in addition to logging the error on LogCat.
    public static void assertion(boolean assertion) {
        if (!assertion) {
            String stackTraceString;
            try {
                throw new Exception("Assertion");
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                stackTraceString = sw.toString();
            }

            // Use regex to get the line number and file name of the code that fired the assertion:
            Pattern pattern = Pattern.compile("\\(.*\\).*\\n.*\\((.*):(.*)\\)");
            Matcher matcher = pattern.matcher(stackTraceString);
            String message;
            if (matcher.find()) {
                message = String.format("Debug.assertion: %s, line %s", matcher.group(1), matcher.group(2));
            } else {
                message = "Debug.assertion, check Logcat!";
            }
            RobotLog.addGlobalWarningMessage(message);

            // Log as an error to Logcat too:
            Log.e("System.out", "Assertion failure:\n" + stackTraceString);
        }
    }

    // Send a sticky red error message to the Driver Station:
    public static void error(String message) {
        RobotLog.addGlobalWarningMessage("Debug.error: " + message);
        Log.e("System.out", "Error: " + message);
    }
}

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
    final double TELEMETRY_PACKET_THROTTLE = 0.1; // Update Road Runner at this interval, in seconds
    public enum Background { BLANK, GRID, FIELD } // Rendering options

    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed
    final private Telemetry telemetry; // Driver Station telemetry object
    private String welcomeMessage; // Welcome message to show only on Driver Station, if any
    public Gamepad gamepad; // Gamepad reference
    private String messageCopy = ""; // Copy of the most recently shown message
    private List<CanvasOp> canvasOpsCopy; // Copy of the most recently shown field canvas
    double lastTelemetryPacketTime; // Time of sending the last telemetry packet

    // The following are null when not in an active begin/end bracket:
    private StringBuilder message; // Message that is begin built
    private Canvas canvas; // Canvas to render
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
        canvas(Background.BLANK); // Draws the field
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
    boolean home() { return buttonPress(gamepad.guide, 12); }
    boolean start() { return buttonPress(gamepad.start, 13); }
    boolean back() { return buttonPress(gamepad.back, 14); }

    // Set the message to be shown constantly on the Driver Station screen (at least until they
    // press Start):
    /** @noinspection SameParameterValue*/
    void setWelcomeMessage(String message) {
        welcomeMessage = message;
    }

    // Begin a UI update. The optional 'showField' parameter dictates whether draw the field or
    // show blank when updating the field:
    void begin() {
        Debug.assertion(packet == null);
        Debug.assertion(message == null);
        Debug.assertion(canvas == null);

        packet = new TelemetryPacket(false); // We'll draw our own field, thank you
        message = new StringBuilder();

        // Disable the welcome message if they've pressed the gamepad's 'start'"
        if (start()) {
            welcomeMessage = null;
        }
    }

    // Return true if a begin/end bracket is active:
    boolean inBracket() {
        return packet != null;
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

    // Get a canvas to render on, initialized to the specified background:
    Canvas canvas(Background background) {
        Debug.assertion(canvas == null);
        Debug.assertion(packet != null);

        canvas = packet.fieldOverlay();
        canvas.setRotation(Math.toRadians(-90));
        if (background == Background.BLANK) {
            canvas.setFill("#e0e0e0");
            canvas.fillRect(-72, -72, 144, 144);
        } else if (background == Background.GRID) {
            canvas.setFill("#e0e0e0");
            canvas.fillRect(-72, -72, 144, 144);
            canvas.drawGrid(0, 0, 144, 144, 7, 7);
        } else {
            canvas.drawImage("/dash/into-the-deep.png", 0, 0, 144, 144,
                    Math.toRadians(90), 0, 144, true);
            canvas.drawGrid(0, 0, 144, 144, 7, 7);

            // Fade field and grid to white by drawing transparent white over it:
            canvas.setAlpha(0.8);
            canvas.setFill("#ffffff");
            canvas.fillRect(-72, -72, 144, 144);
            canvas.setAlpha(1.0);
        }
        return canvas;
    }

    // Clear the field outside of a begin/end bracket:
    /** @noinspection SameParameterValue*/
    void clearField(Background background) {
        begin();
        canvas(background);
        end();
    }

    // Discard the current canvas:
    void abortCanvas() {
        canvas = null;
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

        // Throttle telemetry packets because FTC Dashboard has lousy rate control:
        if (LoonyTune.time() - lastTelemetryPacketTime > TELEMETRY_PACKET_THROTTLE) {
            lastTelemetryPacketTime = LoonyTune.time();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Prepare for the next begin():
        message = null;
        packet = null;
        canvas = null;
    }

    // Put a sticky FTC Dashboard telemetry value:
    void put(String key, Object value) {
        packet.put(key, value);
    }

    // Put a sticky divider string in the telemetry:
    void putDivider() {
        StringBuilder divider = new StringBuilder();
        for (int i = 0; i < 5; i++)
            //noinspection UnnecessaryUnicodeEscape
            divider.append("\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af");
        packet.put(divider.toString(), "");
    }

    // Get rid of any sticky variables that have accumulated in FTC Dashboard's telemetry:
    static void clearDashboardTelemetry() {
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

    // Get the tuning parameters from the current MecanumDrive object but the 'passed' state
    // from the saved state, if any:
    public TuneParameters(MecanumDrive drive) { this(drive, null); }
    public TuneParameters(MecanumDrive drive, TuneParameters savedParameters) {
        params = drive.PARAMS;
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
        Gson gson = new Gson();
        String json = gson.toJson(this);
        writeDataFile(json);
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
    void compareBooleans(String parameter, boolean oldValue, boolean newValue) {
        String oldString = Boolean.toString(oldValue);
        String newString = Boolean.toString(newValue);
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

    // It's not simple to read a file into a string on Java:
    static String readDataFile() {
        StringBuilder string = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(LoonyTune.FILE_NAME))) {
            String line;
            while ((line = reader.readLine()) != null) {
                string.append(line);
                string.append("\n");
            }
        } catch (IOException e) {
            return "";
        }
        return string.toString();
    }

    // Write to our data file. It's not simple to write a string to a file on Java.
    static void writeDataFile(String string) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(LoonyTune.FILE_NAME))) {
            writer.write(string);
        } catch (IOException e) {
            if (!WilyWorks.isSimulating)
                throw new RuntimeException(e);
        }
    }

    // Delete the data file.
    static void deleteDataFile() {
        //noinspection ResultOfMethodCallIgnored
        new File(LoonyTune.FILE_NAME).delete();
    }

    // Validate that the settings are valid and apply to the current robot:
    static TuneParameters getSavedParameters() {
        // Load the saved settings from the preferences database:
        Gson gson = new Gson();
        String json = readDataFile();
        TuneParameters savedParameters = gson.fromJson(json, TuneParameters.class);

        if ((savedParameters == null) || (savedParameters.params == null)) {
            out.println("LoonyTune: Couldn't load saved settings!");
            return null; // No saved settings were found
        }
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
        compare("otos.linearScalar", "%.4f", oldSettings.params.otos.linearScalar, params.otos.linearScalar);
        compare("otos.angularScalar", "%.4f", oldSettings.params.otos.angularScalar, params.otos.angularScalar);
        compare("pinpoint.ticksPerMm", "%.3f", oldSettings.params.pinpoint.ticksPerMm, params.pinpoint.ticksPerMm);
        compareBooleans("pinpoint.xReversed", oldSettings.params.pinpoint.xReversed, params.pinpoint.xReversed);
        compareBooleans("pinpoint.yReversed", oldSettings.params.pinpoint.yReversed, params.pinpoint.yReversed);
        compare("pinpoint.xOffset", "%.1f", oldSettings.params.pinpoint.xOffset, params.pinpoint.xOffset);
        compare("pinpoint.yOffset", "%.1f", oldSettings.params.pinpoint.yOffset, params.pinpoint.yOffset);

        // The following aren't really tuned values so don't compare them:
        //
        // 'compare("maxWheelVel", "%.2f", oldSettings.params.maxWheelVel, params.maxWheelVel);'
        // 'compare("minProfileAccel", "%.2f", oldSettings.params.minProfileAccel, params.minProfileAccel);'
        // 'compare("maxProfileAccel", "%.2f", oldSettings.params.maxProfileAccel, params.maxProfileAccel);'
        // 'compare("maxAngVel", "%.2f", oldSettings.params.maxAngVel, params.maxAngVel);'
        // 'compare("maxAngAccel", "%.2f", oldSettings.params.maxAngAccel, params.maxAngAccel);'
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

    // Update loop for the menu.
    String update() {
        StringBuilder output = new StringBuilder();
        String footer = "";

        // Add a header with submenu names:
        if (menuStack.size() <= 1) {
            output.append("Press " + LoonyTune.DPAD_UP_DOWN + " to navigate, " + LoonyTune.A + " to select.\n\n");
        } else {
            output.append("<big><b>");
            for (int i = 1; i < menuStack.size(); i++) {
                if (i > 1)
                    output.append("\u00b7");
                output.append(menuStack.get(i).description);
            }
            output.append("...</b></big>\n\n");
            footer = "\nPress " + LoonyTune.GUIDE + " for previous menu.";
        }

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
                output.append("<span style='background: " + LoonyTune.HIGHLIGHT_COLOR
                        + "'>" + bullet + " " + widget.string() + "</span>\n");
            }
        }

        Widget widget = menu.widgets.get(menu.current);
        if (io.home()) {
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
                    Io.clearDashboardTelemetry();
                }
            }
        }

        output.append(footer);
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
@TeleOp(name="Loony Tune", group="Utility")
public class LoonyTune extends LinearOpMode {
    @SuppressLint("SdCardPath")
    static final String FILE_NAME = "/sdcard/loony_tune.dat";
    static final String HIGHLIGHT_COLOR = "#9090c0"; // Used to be #88285a
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

    // Types of interactive PiD tuners:
    enum PidTunerType { AXIAL, LATERAL, HEADING, ALL }

    // Menu widgets for each of the tuners:
    enum Tuner {
        NONE(0),

        WHEEL_TEST(1),
        PUSH(2),
        ACCELERATING(3),
        FEED_FORWARD(4),
        SPIN(5),
        TRACKING_TEST(6),
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
    boolean usePinpoint; // True if using Pinpoint odometry, false if using OTOS
    int nextRetuneIndex = Tuner.COUNT.index; // Next tuner to run when re-tuning
    ArrayList<Menu.Widget> unlockables = new ArrayList<>(); // Widgets runnable when everything unlocked

    // Tests and tuners:
    PushTuner pushTuner = new PushTuner();
    SpinTuner spinTuner = new SpinTuner();
    AcceleratingStraightLineTuner acceleratingTuner = new AcceleratingStraightLineTuner();
    InteractiveFeedForwardTuner feedForwardTuner = new InteractiveFeedForwardTuner();
    LateralMultiplierTuner lateralMultiplierTuner = new LateralMultiplierTuner();
    InteractivePidTuner pidTuner = new InteractivePidTuner();

    // Constants:
    final Pose2d zeroPose = new Pose2d(0, 0, 0);

    // Get the current heading from the tracking sensor:
    double getSensorHeading() {
        if (usePinpoint) {
            drive.pinpointDriver.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            return drive.pinpointDriver.getHeading();
        } else {
            return drive.otosDriver.getPosition().h;
        }
    }

    // Get the current velocity from the tracking sensor. Note that this is field-relative
    // for both sensor types.
    Pose2D getSensorVelocity() {
        if (usePinpoint) {
            drive.pinpointDriver.update();
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D velocity
                    = drive.pinpointDriver.getVelocity();
            return new Pose2D(
                    velocity.getX(DistanceUnit.INCH),
                    velocity.getY(DistanceUnit.INCH),
                    velocity.getHeading(AngleUnit.RADIANS));
        } else {
            return drive.otosDriver.getVelocity();
        }
    }

    // Get the current position from the tracking sensor:
    Pose2D getSensorPosition() {
        if (usePinpoint) {
            drive.pinpointDriver.update();
            org.firstinspires.ftc.robotcore.external.navigation.Pose2D position
                    = drive.pinpointDriver.getPosition();
            return new Pose2D(
                    position.getX(DistanceUnit.INCH),
                    position.getY(DistanceUnit.INCH),
                    position.getHeading(AngleUnit.RADIANS));
        } else {
            return drive.otosDriver.getPosition();
        }
    }

    // Add tuners to the menu:
    void addTuner(Tuner tuner, Runnable runnable, String description) {
        widgets[tuner.index] = menu.addRunnable(description, runnable);
    }

    // Add unlockable to the menu:
    void addUnlockable(Runnable runnable, String description) {
        unlockables.add(menu.addRunnable(description, runnable));
    }



    // Update which tuners need to be disabled and which need to be run:
    void updateTunerDependencies(Tuner completedTuner) {
        MecanumDrive.Params params = currentParameters.params;
        MecanumDrive.Params.Pinpoint pinpoint = params.pinpoint;
        MecanumDrive.Params.Otos otos = params.otos;

        boolean pushFailure;
        boolean spinFailure;
        if (usePinpoint) {
            pushFailure = (pinpoint.ticksPerMm == 0);
            spinFailure = (pinpoint.xOffset == 0 || pinpoint.yOffset == 0);
        } else {
            pushFailure = (otos.linearScalar == 0 || otos.offset.h == 0);
            spinFailure = (params.trackWidthTicks == 0 || otos.angularScalar == 0 || otos.offset.x == 0 || otos.offset.y == 0);
        }

        Tuner firstFailure;
        if (!currentParameters.passedWheelTest)
            firstFailure = Tuner.WHEEL_TEST;
        else if (pushFailure)
            firstFailure = Tuner.PUSH;
        else if (params.kS == 0 || params.kV == 0)
            firstFailure = Tuner.ACCELERATING;
        else if (params.kA == 0)
            firstFailure = Tuner.FEED_FORWARD;
        else if (spinFailure)
            firstFailure = Tuner.SPIN;
        else if (!currentParameters.passedTrackingTest)
            firstFailure = Tuner.TRACKING_TEST;
        else if (params.lateralInPerTick == 0 || params.lateralInPerTick == 1)
            firstFailure = Tuner.LATERAL_MULTIPLIER;
        else if (params.axialGain == 0)
            firstFailure = Tuner.AXIAL_GAIN;
        else if (params.lateralGain == 0)
            firstFailure = Tuner.LATERAL_GAIN;
        else if (params.headingGain == 0)
            firstFailure = Tuner.HEADING_GAIN;
        else if (!currentParameters.passedCompletionTest)
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

        // Enable retuning after the push test:
        widgets[Tuner.RETUNE.index].isEnabled = firstFailure.index > Tuner.PUSH.index;

        // Enable unlockables when all tests are passed:
        for (Menu.Widget widget: unlockables) {
            widget.isEnabled = (firstFailure == Tuner.COUNT);
        }
    }

    // Check if the robot code setting the MecanumDrive configuration parameters is up to date
    // with the last results from tuning:
    /** @noinspection BusyWait*/
    static public void verifyCodeMatchesTuneResults(MecanumDrive drive, Telemetry telemetry, Gamepad gamepad) {
        // There's no point in complaining about mismatches when running under the simulator:
        if (WilyWorks.isSimulating)
            return; // ====>

        TuneParameters currentSettings = new TuneParameters(drive, null);
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
                        + "Studio, enter 'md.Params' to jump to the MecanumDrive Params constructor, "
                        + "then update as follows:");
                telemetry.addLine();
                telemetry.addLine(comparison);
                telemetry.addLine("Please update your code and restart now. Or, to proceed anyway and "
                        + "delete the Loony Tune results, triple-tap the START button (to the left of the X) on the gamepad.");
                telemetry.update();

                // Wait for a triple-tap of the button:
                for (int i = 0; i < 3; i++) {
                    try {
                        while (!gamepad.start)
                            Thread.sleep(1);
                        while (gamepad.start)
                            Thread.sleep(1);
                    } catch (InterruptedException e) {
                        return; // Don't save if STOP has been pressed
                    }
                }

                // If we reached this point, the user has chosen to ignore the last tuning results.
                // Delete the data file entirely:
                TuneParameters.deleteDataFile();
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
            Canvas canvas = io.canvas(Io.Background.GRID);
            canvas.setFill("#a0a0a0");
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
     * Class that encapsulates waiting on a user response.
     */
    class Poll {
        // Drive the robot, and wait for either an A/B button press. If ok (A) is pressed,
        // return success. If cancel (B) is pressed, return failure. The robot CAN be driven while
        // waiting.
        boolean okCancelWithDriving() {
            boolean success = false;
            while (!isStopRequested() && !io.cancel()) {
                io.redraw();
                updateGamepadDriving();
                if (io.ok()) {
                    success = true;
                    break;
                }
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
                io.redraw();
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
        static final String QUESTION_ICON = "<big>\u2753</big> ";
        static final String INFORMATION_ICON = "<big><big>\ud83d\udec8</big></big> ";
        static final String CRITICAL_ICON = "<big>\u274c</big> ";
        static final String WARNING_ICON = "<big>\u26a0\ufe0f</big> ";

        void warning(String format, Object... args) {
            Io.clearDashboardTelemetry();
            stopMotors();
            io.message(WARNING_ICON + format + "\n\nPress " + A + " to continue.", args);
            poll.ok();
        }
        void warning(String message) {
            Io.clearDashboardTelemetry();
            stopMotors();
            io.message(WARNING_ICON + message + "\n\nPress " + A + " to continue.");
            poll.ok();
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
        // Get a preview of the trajectory's path:
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        drive.runParallel(action);
        while (opModeIsActive() && !io.cancel()) {
            io.begin();
            io.canvas(Io.Background.GRID).getOperations().addAll(previewCanvas.getOperations());
            io.out(header + "Press "+B+" to cancel and stop the robot.");
            boolean more = drive.doActionsWork(io.packet);
            if (!more) {
                // We successfully completed the Action! Toss the current (incomplete) canvas:
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
            out.printf("LoonyTune: raw stick: %.2f, shaped: %.2f, power: %.2f, signum: %.2f\n", stickValue, result, power, Math.signum(stickValue));
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
            io.message("Double-tap the shift key in Android Studio, enter '<b>md.Params</b>' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
            poll.ok();
        }
    }

    // Set the hardware to the current parameters:
    public void initializeTrackerHardware() {
        drive.initializeOtosDriver();
        drive.initializePinpointDriver();
    }

    // Return true if the optical tracker hardware is responding:
    boolean isOpticalTrackerResponsive() {
        // Send a new offset to the hardware:
        drive.otosDriver.setPosition(new Pose2D(12, 34, 0));

        // We need to wait for at least an entire OTOS quantum (2.4ms) before reading back:
        sleep(5);
        Pose2D read = drive.otosDriver.getPosition();

        // Return success if we read back about the same as what we wrote in (accounting for
        // slight jitter movement on the robot):
        return Math.round(read.x) == 12 && Math.round(read.y) == 34;
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
    void stopMotors() {
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
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

    /**
     * This class provides a framework for tuners and tests with multiple screens. It provides
     * common code to handle the UI to transition screens, results history, and everything
     * related to exiting the test.
     */
    class Screens {
        final String HEADER_FORMAT = "<span style='background:%s'>Screen %d of %d: %s</span>\n\n";

        Tuner tuner; // Tuner type, can be NONE
        public final String[] screens; // Name for each screen except for the Exit screen
        public int index; // Currently active screen
        public boolean switched; // True if the screen was just switched, false if not
        public String header = ""; // Header to be drawn at the top of the screen
        public String buttons = ""; // Buttons message describing what the bumper buttons do
        public ArrayList<Result> history = new ArrayList<>(); // History of results
        public int count; // Count of screens including the exit screen
        final private Consumer<Boolean> testPassFail; // Completion callback
        final private BiConsumer<TuneParameters, TuneParameters> syncParams; // Sync callback for parameters

        // 'testParameters' is used to represent the parameters being tested, carefully
        // ignoring those parameters which are being overwritten just for this test:
        /** @noinspection FieldMayBeFinal*/
        private TuneParameters testParameters = currentParameters.createClone();

        // This class handles the saving of results on behalf of the caller when the user exits
        // the set of screens. There are 4 types of results supported. All will automatically
        // save the state if it's changed and the user requests to save on exit:
        //
        //   1. Tuning results supplied as Result objects from caller via registerResult().
        //   2. Tuning of the active params relative to the official current settings. For this,
        //      the caller supplies a syncParams callback to copy the appropriate fields from
        //      the active params to the official current params (necessary as these tests often
        //      temporarily override other params state which shouldn't be saved).
        //   3. Test where the user is asked if it succeeded or not. In this case, the caller
        //      supplies a testPassFail callback to get informed of the results.
        //   4. There are no results. In this case, testPassFail and syncParams are null, and
        //      registerResult() is never called.
        Screens(Tuner tuner, String[] screens) { this(tuner, screens, null, null); }
        Screens(Tuner tuner, String[] screens, Consumer<Boolean> testPassFail) { this(tuner, screens, testPassFail, null); }
        Screens(Tuner tuner, String[] screens, BiConsumer<TuneParameters, TuneParameters> syncParams) { this(tuner, screens, null, syncParams); }
        Screens(Tuner tuner, String[] screens, Consumer<Boolean> testPassFail, BiConsumer<TuneParameters, TuneParameters> syncParams) {
            this.tuner = tuner;
            this.screens = screens;
            this.count = screens.length + 1;
            this.testPassFail = testPassFail;
            this.syncParams = syncParams;
        }

        // Add a result to the history:
        void registerResult(Result result) {
            history.add(result);
        }

        // Handle all of the loop bookkeeping for screens controlled by the bumpers. Check the
        // gamepad input and update the status string as well as the buttons string:
        //
        // Returns true if the caller should keep looping and call again; false if the caller
        // should be done and exit.
        /** @noinspection BooleanMethodIsAlwaysInverted*/
        boolean update() {
            Debug.assertion(!io.inBracket());

            // Loop here to handle the exit screen updates. Everything in the exit screen is the
            // responsibility of this class.
            while (!isStopRequested()) {
                int oldIndex = index;
                if (io.leftBumper()) { // Go to previous screen
                    index = Math.max(index - 1, 0);
                }
                if (io.rightBumper()) { // Go to next screen
                    index = Math.min(index + 1, count - 1);
                }
                if (io.home()) { // Exit
                    if (syncTestParameters().isEmpty())
                        return false; // ====> No parameters changes, so immediately tell caller to exit

                    // There are pending changes:
                    io.message(Dialog.WARNING_ICON + "You have unsaved results, are you "
                            + "sure you want to exit?\n"
                            + "\n"
                            + "Press " + A + " to exit without saving, " + B + " to cancel.");
                    if (poll.okCancel())
                        return false; // ====> User said okay, so exit without saving
                }

                switched = (index != oldIndex); // Callers can check this variable
                if (switched) {
                    Io.clearDashboardTelemetry();
                }
                if (index == 0) {
                    buttons = RIGHT_BUMPER + " (bumper button above right trigger) for the "
                            + "next screen, " + GUIDE + " (big center button) "
                            + "to exit";
                } else if (index < count - 1) {
                    buttons = RIGHT_BUMPER + "/" + LEFT_BUMPER + " for the next/previous screen";
                } else{
                    buttons = LEFT_BUMPER + " for the previous screen";
                }
                if (index < count -1 ) {
                    // A screen other than the exit screen is active so update the header
                    // and return control to the caller:
                    header = String.format(HEADER_FORMAT,
                            HIGHLIGHT_COLOR, index + 1, count, screens[index]);
                    return true; // ====>
                }

                // The exit screen is active. Do an iteration of its processing:
                header = String.format(HEADER_FORMAT, HIGHLIGHT_COLOR, index + 1, count, "Exit");
                if (!updateExitScreen())
                    return false; // ====>
            }
            return false;
        }

        // Sync our copy of the test parameters with the actual state and return a string
        // describing the state that was changed:
        String syncTestParameters() {
            if (syncParams != null) {
                // Sync just the relevant fields from the drive's current parameters into our
                // official test parameters record:
                syncParams.accept(testParameters, new TuneParameters(drive));
            } else if (history.size() > 1) {
                // Take the last result in the history and apply it to our test parameters:
                history.get(history.size() - 1).applyTo(testParameters);
            }
            return testParameters.compare(currentParameters, true);
        }

        // Handles all of the exit screen logic. Returns true if the caller should keep calling,
        // false if the caller should be done:
        boolean updateExitScreen() {
            io.begin();
            if ((testPassFail == null) && (history.isEmpty()) && (syncParams == null)) { // Examples case
                io.out(header + "Press " + A + " to exit.");
                io.end();
                //noinspection RedundantIfStatement
                if (io.ok()) {
                    return false; // ====>
                }
            } else if (testPassFail != null) { // Test case
                io.out(header + "Press " + A + " if everything passed, " + B + " if there was "
                        + "a failure, or " + buttons + ".");
                io.end();
                if (io.ok()) {
                    testPassFail.accept(true);
                    currentParameters.save();
                    updateTunerDependencies(tuner);
                    return false; // ====>
                }
                if (io.cancel()) {
                    testPassFail.accept(false);
                    currentParameters.save();
                    updateTunerDependencies(tuner);
                    return false; // ====>
                }
            } else { // Tuner case
                String changes = syncTestParameters();
                if (changes.isEmpty()) {
                    io.out(header + "No configuration has changed, press " + A + " to exit, "
                            + buttons + ".");
                    io.end();
                    if (io.ok())
                        return false; // ====>
                } else {
                    io.out(HEADER_FORMAT, HIGHLIGHT_COLOR, count, count, "Save and exit");
                    io.out("Current configuration changes:\n\n" + changes + "\n");
                    io.out(A + " to save these results, " + B + " to discard and exit, ");
                    io.out(buttons + ".");
                    io.end();

                    if (io.ok()) {
                        acceptParameters(testParameters);
                        updateTunerDependencies(tuner);
                        return false; // ====>
                    }
                }
                if (io.cancel()) {
                    if (changes.isEmpty())
                        return false; // ====>

                    io.message(Dialog.WARNING_ICON + "Are you sure you want to discard your results?\n\n"
                            + A + " to discard, " + B + " to cancel.");
                    return !poll.okCancel();
                }
            }
            return true; // Keep calling
        }

        // Show the history of results. The first result will be tagged as the original, and the
        // final result will be bolded and tagged as the newest.
        void showHistory(Io io) {
            // The first entry is always the original settings:
            if (history.size() > 1) {
                io.out("Result history for %s:\n\n", history.get(0).getHeader());
                for (int i = 0; i < history.size(); i++) {
                    Result result = history.get(i);
                    io.out("&ensp;<b>%d:</b> %s", i, result.getValues());
                    if (i == 0) {
                        io.out(" (original)");
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
            middle = "<span style='background: " + HIGHLIGHT_COLOR + "'>" + middle + "</span>";

            // Blink the underline every half second:
            if ((((int) (time() * 2)) & 1) != 0) {
                middle = "<u>" + middle + "</u>";
            }

            return prefix + middle + suffix;
        }
    }

    // Run a simple trajectory with a preview and an option to disable odometry. The description
    // can be null. We take a lambda Action supplier rather than a simple Action because we
    // need to invoke the Action multiple times, and Road Runner can't do that.
    void runTrajectory(Supplier<Action> action) { runTrajectory(action, null, null);}
    void runTrajectory(Supplier<Action> action, String description, String clearance) {
        configureToDrive(true); // Do use MecanumDrive
        TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, action.get());
        Screens screens = new Screens(Tuner.NONE, new String[] {"Preview", "Test", "Optional experiment"});

        int runCount = 0;
        while (opModeIsActive()) {
            if (!screens.update())
                break; // ===>

            io.begin();
            io.out(screens.header);
            if (screens.index == 0) {
                previewer.update();
                updateGamepadDriving();
                if (description == null) {
                    io.out("The robot will run the trajectory previewed in FTC Dashboard.\n\n");
                } else {
                    io.out(description + "\n\n");
                }
                io.out("Press " + screens.buttons + ".");
                io.end();
            } else if (screens.index == 1) { // Run screen
                updateGamepadDriving();
                if (runCount > 0) {
                    io.out("Max gain error: %.2f\", %.2f\u00b0\n"
                                    + "End gain error: %.2f\", %.2f\u00b0\n\n",
                            drive.maxLinearGainError, Math.toDegrees(drive.maxHeadingGainError),
                            drive.lastLinearGainError, Math.toDegrees(drive.lastHeadingGainError));
                }
                io.out("Press " + A + " to start the robot");
                if (clearance != null)
                    io.out(" " + clearance);
                io.out(", " + screens.buttons + ".");
                io.end();

                if (io.ok()) {
                    runCount++;
                    stopMotors();
                    drive.setPose(zeroPose);
                    drive.maxLinearGainError = 0; // Reset for our new run
                    drive.maxHeadingGainError = 0;
                    runCancelableAction(screens.header, action.get());
                }
            } else if (screens.index == 2) { // Experiment screen
                updateGamepadDriving();
                io.out("This screen is for trouble shooting, skip this if your robot works well.\n\n");
                io.out("This runs the trajectory with the usual odometry correction disabled. "
                        + "This tests how well the non-odometry settings have been tuned. If well "
                        + "tuned, the robot should drive close to the intended path.\n\n");
                if (runCount > 0) {
                    io.out("Max gain error: %.2f\", %.2f\u00b0\n"
                                    + "End gain error: %.2f\", %.2f\u00b0\n\n",
                            drive.maxLinearGainError, Math.toDegrees(drive.maxHeadingGainError),
                            drive.lastLinearGainError, Math.toDegrees(drive.lastHeadingGainError));
                }
                io.out("Press " + A + " to start the robot, " + screens.buttons + ".");
                io.end();

                if (io.ok()) {
                    runCount++;
                    stopMotors();
                    drive.setPose(zeroPose);
                    drive.maxLinearGainError = 0; // Reset for our new run
                    drive.maxHeadingGainError = 0;

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
                    runCancelableAction("", action.get());
                    MecanumDrive.PARAMS = currentParameters.params;
                }
            }
        }
    }

    // All tuner results are derived from this Result class:
    abstract static class Result {
        // Get the names of the result fields:
        abstract public String getHeader();

        // Convert the values to a one-line descriptive string:
        abstract public String getValues();

        // Apply the changes associated with these results to the tuned parameters structure:
        abstract public void applyTo(TuneParameters parameters);
    }

    //**********************************************************************************************

    // Verify wheel correctness on the robot by driving around and testing each wheel individually.
    void wheelTest() {
        configureToDrive(true); // Do use MecanumDrive

        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        String[] motorNames = { "leftFront", "leftBack", "rightBack", "rightFront" };
        Screens screens = new Screens(Tuner.WHEEL_TEST, new String[] {
                "Preview",                      // 0
                "Test 'leftFront' wheel",       // 1
                "Test 'leftBack' wheel",        // 2
                "Test 'rightBack' wheel",       // 3
                "Test 'rightFront' wheel",      // 4
                "Test all wheels by driving"},  // 5
                (passed)->currentParameters.passedWheelTest = passed);

        while (opModeIsActive()) {
            if (!screens.update())
                break; // ====>

            io.begin();
            io.out(screens.header);
            if (screens.index == 0) {
                io.out("This test validates that every wheel is configured correctly. "
                        + "In the next screens, you'll test every wheel individually, then drive "
                        + "the robot.\n"
                        + "\n"
                        + "Press " + screens.buttons + ".");
                io.end();
            } else if (screens.index < 5) { // Individual wheel screen
                int motor = screens.index - 1;
                double power = gamepad1.right_trigger - gamepad1.left_trigger;
                motors[motor].setPower(power);
                DcMotorSimple.Direction direction =  motors[motor].getDirection();
                String motorName = motorNames[motor];

                io.out("Use the triggers to power this wheel:\n\n");
                io.out("&emsp;<b>%s.setPower(%.2f)</b> ", motorName, power);
                if (power > 0) {
                    io.out("// Should turn forward");
                } else if (power < 0) {
                    io.out("// Should turn backward");
                }
                io.out("\n\n"
                        + "The right trigger should turn the wheel forward; the left trigger should "
                        + "turn it backward. If it turns in the wrong direction, double-tap the shift "
                        + "key in Android Studio, enter <b>'MecanumDrive.configure'</b>, and ");
                if (direction == DcMotorSimple.Direction.FORWARD) {
                    io.out("add a call to <b>'%s.setDirection( DcMotorEx.Direction.REVERSE)'</b>.", motorName);
                } else {
                    io.out("disable the line <b>'%s.setDirection( DcMotorEx.Direction.REVERSE);'</b>.", motorName);
                }
                io.out("\n\nPress "+RIGHT_TRIGGER+" to rotate wheel forward, "+LEFT_TRIGGER+" for reverse, "+ screens.buttons + ".");
                io.end();
            } else if (screens.index == 5) { // All wheels screen
                updateGamepadDriving();
                io.out("Test the robot by driving it around. Left stick controls movement, "
                        + "right stick controls rotation."
                        + "\n\n"
                        + "If it strafes in the wrong direction (e.g., left when you want right), or "
                        + "if it drives weird even though it passed the previous steps, the Mecanum "
                        + "wheels may be mounted wrong. When looking from above, the direction "
                        + "of the rollers on the 4 wheels should form an 'X'.");
                io.out("\n\nSticks to drive, " + screens.buttons + ".");
                io.end();
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
        String lastSeenStatus = ""; // Most recently reported non-zero status from the sensor
        double lastSeenTime = 0; // Time at which lastSeenStatus was set
        double goldenStartTime = 0; // Start time of the golden pose test, in seconds

        Pose2d baselinePose = null; // Position where the baseline was set
        double maxLinearSpeed = 0; // Max linear speed seen, inches/s
        double maxRotationalSpeed = 0; // Max rotational speed seen, radians/s

        Screens screens = new Screens(Tuner.TRACKING_TEST,
                new String[]{"Preview", "Free drive", "Measure error", "Stats"},
                (passed)->currentParameters.passedTrackingTest = passed);
        while (opModeIsActive()) {
            if (!screens.update())
                break; // ====>

            io.begin();
            io.out(screens.header);

            // Update our guess for the robot location:
            PoseVelocity2d velocity = drive.updatePoseEstimate();

            // Query the OTOS for any problems:
            String currentStatus = "";
            if (usePinpoint) {
                // Note that the Pinpoint driver's update() was called by updatePoseEstimate().
                GoBildaPinpointDriver.DeviceStatus status = drive.pinpointDriver.getDeviceStatus();
                if (status == GoBildaPinpointDriver.DeviceStatus.READY)
                    currentStatus = ""; // Good result!
                else if (status == GoBildaPinpointDriver.DeviceStatus.NOT_READY)
                    currentStatus = "not ready";
                else if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING)
                    currentStatus = "calibrating";
                else if (status == FAULT_X_POD_NOT_DETECTED)
                    currentStatus = "X pod not detected";
                else if (status == FAULT_Y_POD_NOT_DETECTED)
                    currentStatus = "Y pod not detected";
                else if (status == FAULT_NO_PODS_DETECTED)
                    currentStatus = "no pods detected";
                else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY)
                    currentStatus = "IMU runaway";
                else
                    currentStatus = "Unknown error";

                int loopTime = drive.pinpointDriver.getLoopTime();
                if ((loopTime < 500) || (loopTime > 1100)) {
                    if (!currentStatus.isEmpty())
                        currentStatus += ", ";
                    currentStatus += String.format("Bad loop time %d", loopTime);
                }
            } else {
                SparkFunOTOS.Status status = drive.otosDriver.getStatus();
                if (status.errorLsm)
                    currentStatus += "errorLsm ";
                if (status.errorPaa)
                    currentStatus += "errorPaa ";
                if (status.warnOpticalTracking)
                    currentStatus += "warnOpticalTracking ";
                if (status.warnTiltAngle)
                    currentStatus += "warnTileAngle ";
            }

            if (!currentStatus.isEmpty()) {
                lastSeenStatus = currentStatus;
                lastSeenTime = time();
            }

            // Calculate some statistics about the pose results:
            double linearSpeed = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);
            double rotationalSpeed = Math.abs(velocity.angVel);
            maxLinearSpeed = Math.max(maxLinearSpeed, linearSpeed);
            maxRotationalSpeed = Math.max(maxRotationalSpeed, rotationalSpeed);

            if (screens.index == 0) { // Preview
                io.canvas(Io.Background.BLANK); // Clear the field
                io.out("At this point, pose estimation should fully work. This test "
                        + "verifies its correctness."
                        + "\n\n"
                        + "In the first screen, drive around and view the real-time pose estimate via "
                        + "FTC Dashboard to visually verify that the tracking performs well. In "
                        + "the subsequent screen, you can quantify the accuracy. "
                        + "The final screen has statistics on the performance of the tracking "
                        + "and the robot."
                        + "\n\n"
                        + "Press " + screens.buttons + ".");
                io.end();
            } else if (screens.index == 1) { // Free drive screen
                updateGamepadDriving();

                Canvas canvas = io.canvas(Io.Background.GRID);
                canvas.setStroke("#3F51B5");
                Drawing.drawRobot(canvas, drive.pose);

                io.out("Drive around and look for two things:\n"
                        + "\n"
                        + "\u2022 Does the robot shown in the field view correctly track "
                        + "the actual movement? (The next screen will do exact error measurements "
                        + "so just look for approximate correctness here.)\n"
                        + "\u2022 When the robot rotates in place using only the right stick, the "
                        + "on-screen robot shouldn't move its <i>(x, y)</i> position at all. Does it?\n"
                        + "\u2022 Does a full 360° end with correct alignment?\n"
                        + "\n");
                io.out("Pose: (%.2f\", %.2f\"), %.2f\u00b0\n",
                        drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.toDouble()));
                io.out("Velocity: (%.1f\"/s, %.1f\"/s), %.1f\u00b0/s\n\n",
                        velocity.linearVel.x, velocity.linearVel.y, Math.toDegrees(velocity.angVel));
                io.out("Sticks to drive, press " + X + " to reset the pose, " + screens.buttons + ".");
                io.end();

                if (io.x()) {
                    drive.setPose(zeroPose);
                }
            } else if (screens.index == 2) { // Measure error screen
                updateGamepadDriving();

                Canvas canvas = io.canvas(Io.Background.FIELD); // Draw the real actual field
                canvas.setStroke("#ffd700"); // Gold
                Drawing.drawRobot(canvas, homePose);

                if (baselinePose == null) {
                    io.out("To start measuring error, press "+X+" when the robot is physically "
                            + "positioned in the corner as indicated by the golden pose on the "
                            + "field view.\n\n");
                    io.out("Press "+X+" when at the golden pose, "+ screens.buttons+".");
                } else {
                    // Get the new pose and track the distance traveled:
                    Pose2d pose = drive.pose;
                    totalDistance += Math.hypot(
                            pose.position.x - previousPose.position.x,
                            pose.position.y - previousPose.position.y);
                    totalRotation += Math.abs(pose.heading.toDouble() - previousPose.heading.toDouble());
                    previousPose = pose;

                    canvas.setStroke("#3F51B5"); // Blueish
                    Drawing.drawRobot(canvas, pose); // Pose estimation

                    io.out("Drive around and then physically return the robot back to the "
                            + "golden pose corner location and stop.\n");
                    if (linearSpeed < 1.0) {
                        io.out("\nThese results are valid <b><i>only</b></i> when the robot "
                            + "is back in the same golden pose physical location:\n\n");

                        double dx = pose.position.x - baselinePose.position.x;
                        double dy = pose.position.y - baselinePose.position.y;
                        double otosTheta = normalizeAngle(pose.heading.toDouble() - baselinePose.heading.toDouble());

                        io.out("&ensp;Sensor error: (%.2f\", %.2f\"), %.2f\u00b0\n", dx, dy, Math.toDegrees(otosTheta));

                        if ((totalDistance != 0) && (totalRotation != 0)) {
                            double distancePercent = 100 * Math.abs(dx) / totalDistance;
                            double exactMinutes = (time() - goldenStartTime) / 60.0;

                            // Round the minutes so that the rotation error updates every 6 seconds
                            // rather than constantly:
                            double roundedMinutes = Math.round(exactMinutes * 10.0) / 10.0;
                            double degreesPerMinute = (roundedMinutes == 0) ? 0 :
                                    Math.abs(Math.toDegrees(otosTheta)) / roundedMinutes;
                            io.out("&ensp;Positional error: <b>%.2f</b>%%\n", distancePercent);
                            io.out("&ensp;Rotational error: <b>%.2f</b>\u00b0/minute\n", degreesPerMinute);
                            io.out("\nGood error results are less than 1% positional and 1\u00b0/minute rotational.\n");
                        }
                    }

                    io.out("\nPress "+X+" to restart when back at the golden pose, "
                            + Y + " to forget the golden pose, " + screens.buttons + ".");
                }
                io.end();

                if (io.x()) { // Set golden pose at home
                    drive.setPose(homePose);
                    previousPose = homePose;
                    baselinePose = drive.pose;
                    totalDistance = 0;
                    totalRotation = 0;
                    maxLinearSpeed = 0;
                    maxRotationalSpeed = 0;
                    goldenStartTime = time();
                }
                if (io.y()) { // Clear the baseline
                    baselinePose = null;
                }

            } else if (screens.index == 3) { // Stats screen
                updateGamepadDriving();

                Canvas canvas = io.canvas(Io.Background.GRID);
                canvas.setStroke("#3F51B5");
                Drawing.drawRobot(canvas, drive.pose);

                io.put(" ", "Dashboard can graph this section :");
                io.put("Linear speed", linearSpeed);
                io.put("Rotational speed", Math.toDegrees(rotationalSpeed));
                io.putDivider();

                if (currentStatus.isEmpty()) {
                    if (lastSeenStatus.isEmpty())
                        io.out("Sensor error: None!\n");
                    else {
                        double minutesAgo = (time() - lastSeenTime) / 60.0;
                        if (minutesAgo < 0.083) { // Within last 5 seconds
                            io.out("Sensor error: <font color='#cc0202'>%s</font>\n", lastSeenStatus);
                        } else {
                            io.out("Sensor error (%.1f minutes ago): %s\n", minutesAgo, lastSeenStatus);
                        }
                    }
                } else {
                    io.out("Sensor error: %s\n", currentStatus);
                }

                if (usePinpoint) {
                    io.out("Latest sensor loop time: %d\n", drive.pinpointDriver.getLoopTime());
                }

                io.out("Max velocities: %.1f \"/s, %.1f\u00b0/s\n",
                        maxLinearSpeed, Math.toDegrees(maxRotationalSpeed));

                io.out("\nPress " + X + " to reset pose, " + Y + " to reset stats, " + screens.buttons + ".");
                io.end();

                if (io.x()) {
                    drive.setPose(zeroPose);
                }
                if (io.y()) {
                    maxLinearSpeed = 0;
                    maxRotationalSpeed = 0;
                }

            }
        }
    }

    /**
     * Class to encapsulate push-tuner logic for all devices.
     */
    class PushTuner {
        final int FORWARD_DISTANCE = 96; // Forward test distance in inches
        final int LEFT_DISTANCE = 24; // Leftwards test distance in inches (only used for Pinpoint)

        // goBilda's odometry pods are 13.3 and 19.9 ticks-per-mm which works out to
        // 336.9 and 505.3 ticks-per-inch. The REV thru-bore encoder is 1891.9 inches-
        // per-tick using the OpenOdometry design.
        final int MIN_TICKS_PER_INCH = 100;

        // Structure used to encapsulate an OTOS result from push tuning:
        class OtosPushResult extends Result {
            double linearScalar;
            double offsetHeading;

            public OtosPushResult(double linearScalar, double offsetHeading) {
                this.linearScalar = linearScalar;
                this.offsetHeading = offsetHeading;
            }

            @Override
            public String getHeader() {
                return ("<b>linearScalar</b>, <b>offset heading</b>");
            }

            @Override
            public String getValues() {
                return String.format("%.4f, %.2f\u00b0", linearScalar, Math.toDegrees(offsetHeading));
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.otos.linearScalar = linearScalar;
                parameters.params.otos.offset.h = offsetHeading;
            }
        }

        // Structure used to encapsulate a result from push tuning:
        class PinpointPushResult extends Result {
            double ticksPerMm;
            boolean xReversed;
            boolean yReversed;

            public PinpointPushResult(double ticksPerMm, boolean xReversed, boolean yReversed) {
                this.ticksPerMm = ticksPerMm;
                this.xReversed = xReversed;
                this.yReversed = yReversed;
            }

            @Override
            public String getHeader() {
                return ("<b>ticksPerMm</b>, <b>xReversed</b>, <b>yReversed</b>");
            }

            @Override
            public String getValues() {
                return String.format("%.3f, %s, %s", ticksPerMm, xReversed, yReversed);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.pinpoint.ticksPerMm = ticksPerMm;
                parameters.params.pinpoint.xReversed = xReversed;
                parameters.params.pinpoint.yReversed = yReversed;
            }
        }

        // Do the measuring work for OTOS devices. Returns the result once done; will be null if
        // there was an apparent error.
        OtosPushResult measureOtos(String header, double oldLinearScalar, double oldOffsetHeading) {
            double distance = 0;
            double heading = 0;

            // Reset the optical tracker to the origin:
            drive.otosDriver.resetTracking();

            while (!io.ok()) {
                // Use the optical tracker directory instead of using Road Runner's pose because we
                // haven't calibrated enough yet for the latter to be correct!
                Pose2D pose = drive.otosDriver.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                io.begin();
                io.out(header);
                io.out("Push forward exactly " + testDistance(FORWARD_DISTANCE) + " along the field wall.\n\n");
                io.out("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h));
                io.out("&ensp;Effective distance: %.2f\"\n", distance);

                // The heading angle will jump wildly at the very start so don't show any values until
                // there's enough distance that the angle becomes more reliable:
                io.out("&ensp;Heading angle: ");
                if (distance > 24) {
                    double currentHeading = normalizeAngle(heading + oldOffsetHeading);
                    io.out("%.2f\u00b0\n", Math.toDegrees(currentHeading)); // Degree symbol
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
            double newLinearScalar = (FORWARD_DISTANCE / distance) * oldLinearScalar;
            double newOffsetHeading = normalizeAngle(heading + oldOffsetHeading);

            if (newLinearScalar < SparkFunOTOS.MIN_SCALAR) {
                io.begin();
                io.out(Dialog.WARNING_ICON + "The measured distance of %.1f\" is not close enough to the expected distance " +
                                "of %d\". It can't measure more than %.1f\". ",
                        distance, FORWARD_DISTANCE, FORWARD_DISTANCE / SparkFunOTOS.MIN_SCALAR);
                io.out("Either you didn't push straight for " + testDistance(FORWARD_DISTANCE) + " or " +
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
                        distance, FORWARD_DISTANCE, FORWARD_DISTANCE / SparkFunOTOS.MAX_SCALAR);
                io.out("Either you didn't push straight for " + testDistance(FORWARD_DISTANCE) + " or " +
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
            return new OtosPushResult(newLinearScalar, newOffsetHeading);
        }

        // Measure the optical linear scale and orientation.
        void tuneOtos() {
            configureToDrive(false); // Don't use MecanumDrive

            double oldLinearScalar = currentParameters.params.otos.linearScalar;
            if (oldLinearScalar == 0)
                oldLinearScalar = 1.0; // Can happen on the very first run, stock Road Runner sets to zero
            double oldOffsetHeading = currentParameters.params.otos.offset.h;

            Action preview = drive.actionBuilder(new Pose2d(-FORWARD_DISTANCE / 2.0, -60, 0))
                    .lineToX(FORWARD_DISTANCE / 2.0)
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            Screens screens = new Screens(Tuner.PUSH, new String[]{"Preview", "Measure"});
            screens.registerResult(new OtosPushResult(
                    currentParameters.params.otos.linearScalar,
                    currentParameters.params.otos.offset.h));
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen
                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("You'll push the robot forward in a straight line along a field wall for "
                            + "exactly " + testDistance(FORWARD_DISTANCE) + ". This will measure the following:\n"
                            + "\n"
                            + "\u2022 <b>linearScalar</b> accounts for the height of the OTOS sensor from "
                            + "the surface of the field when measuring distance.\n"
                            + "\u2022 <b>offset heading</b> corrects the robot's heading to account for the "
                            + "direction that the sensor is mounted on the robot.\n"
                            + "\n"
                            + "Press " + screens.buttons + ".");
                    io.end();
                } else if (screens.index == 1) { // Measure screen
                    updateGamepadDriving();
                    io.canvas(Io.Background.BLANK); // Clear the field
                    screens.showHistory(io); // Show measurement history and advise when done

                    io.out("To start a measurement, align the robot by hand to its starting point "
                            + "aligned to a field wall, with room ahead for " + testDistance(FORWARD_DISTANCE) + ".");
                    io.out("\n\n");
                    io.out("Press " + A + " when in position, " + screens.buttons + ".");
                    io.end();

                    if (io.ok()) {
                        OtosPushResult result = measureOtos(screens.header, oldLinearScalar, oldOffsetHeading);
                        if (result != null)
                            screens.registerResult(result);
                    }
                }
            }

            // Set/restore the hardware settings:
            initializeTrackerHardware();
        }

        // Push either forward or left, as requested by the caller:
        Point pushPinpoint(int inches, String header, String message) {
            DecimalFormat formatter = new DecimalFormat("#,###");
            drive.pinpointDriver.update();
            int xEncoderStart = drive.pinpointDriver.getEncoderX();
            int yEncoderStart = drive.pinpointDriver.getEncoderY();
            while (!io.ok()) {
                drive.pinpointDriver.update();
                int xTicks = drive.pinpointDriver.getEncoderX() - xEncoderStart;
                int yTicks = drive.pinpointDriver.getEncoderY() - yEncoderStart;

                io.begin();
                io.out(header);
                io.out(message + "\n\n");
                io.out("&ensp;X encoder: %s ticks\n", formatter.format(xTicks));
                io.out("&ensp;Y encoder: %s ticks\n", formatter.format(yTicks));

                io.out("\n");
                io.out("Press " + A + " when you've finished pushing, " + B + " to cancel.");
                io.end();

                if (io.cancel() || isStopRequested())
                    return null; // ====>
            }
            int xTicks = drive.pinpointDriver.getEncoderX() - xEncoderStart;
            int yTicks = drive.pinpointDriver.getEncoderY() - yEncoderStart;

            if (Math.max(Math.abs(xTicks), Math.abs(yTicks)) < inches * MIN_TICKS_PER_INCH) {
                dialog.warning("The measured number of ticks was too low. " +
                        "Check your odometry wheels (ensure good contact with the " +
                        "field) and the wiring (make sure there are good connections) and restart.");
                return null; // ====>
            }
            return new Point(xTicks, yTicks);
        }

        // Do the measuring work for Pinpoint devices. Returns the result once done; will be null if
        // there was an apparent error.
        PinpointPushResult measurePinpoint(String header) {
            Point forward = pushPinpoint(FORWARD_DISTANCE, header,
                    "Push forward exactly " + testDistance(FORWARD_DISTANCE) + " along the field wall. " +
                    "Try to be as precise as possible.");
            if (forward == null)
                return null; // ===>

            io.message(header + "Now position the robot so that it can be pushed about " +
                    testDistance(LEFT_DISTANCE) + " to its left.\n\n" +
                    "Press " + A + " to continue, " + B + " to cancel.");
            if (!poll.okCancelWithDriving())
                return null; // ====>

            Point left = pushPinpoint(LEFT_DISTANCE, header,
                    "Push left approximately " + testDistance(LEFT_DISTANCE) + ". This " +
                    "does not have to be exact.");
            if (left == null)
                return null; // ===>

            // Check if the X and Y results might be swapped:
            boolean inconsistent = false;

            if ((Math.abs(forward.y) > Math.abs(forward.x)) &&
                (Math.abs(left.x) > Math.abs(left.y))) {

                if ((0.2 * Math.abs(forward.y) < Math.abs(forward.x)) ||
                    (0.2 * Math.abs(left.x) < Math.abs(left.y))) {
                    inconsistent = true;
                } else {
                    dialog.warning("It looks like the X and Y encoder results are swapped. " +
                            "Physically unplug the X and Y encoder wires from the Pinpoint computer, " +
                            "then plug them in again but swapped. Restart once that's done.");
                    return null; // ====>
                }
            } else {
                if ((0.2 * Math.abs(forward.x) < Math.abs(forward.y)) ||
                    (0.2 * Math.abs(left.y) < Math.abs(left.x))) {
                    inconsistent = true;
                }
            }

            double forwardTicksPerMm = Math.abs(forward.x) / (FORWARD_DISTANCE * 25.4);
            double leftTicksPerMm = Math.abs(left.y) / (LEFT_DISTANCE * 25.4);

            if ((leftTicksPerMm < 0.67 * forwardTicksPerMm) || (leftTicksPerMm) > 1.5 * forwardTicksPerMm) {
                inconsistent = true;
            }

            if (inconsistent) {
                dialog.warning("The forward and left tick counts are inconsistent. " +
                        "Check your odometry wheels (ensure good contact with the " +
                        "field) and the wiring (make sure there are good connections) and restart.");
                return null; // ====>
            }

            // Use the forward ticks-per-mm as that result will be more accurate given the longer
            // measurement:
            return new PinpointPushResult(forwardTicksPerMm, forward.x < 0, left.y < 0);
        }

        // Measure the Pinpoint linear scale and orientation.
        void tunePinpoint() {
            configureToDrive(false); // Don't use MecanumDrive

            double positionY = -60;
            Action preview = drive.actionBuilder(new Pose2d(-FORWARD_DISTANCE / 2.0, positionY, 0))
                    .lineToX(FORWARD_DISTANCE / 2.0)
                    .strafeTo(new Vector2d(FORWARD_DISTANCE / 2.0, positionY + LEFT_DISTANCE))
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);
            Screens screens = new Screens(Tuner.PUSH, new String[]{"Preview", "Measure"});
            screens.registerResult(new PinpointPushResult(
                    currentParameters.params.pinpoint.ticksPerMm,
                    currentParameters.params.pinpoint.xReversed,
                    currentParameters.params.pinpoint.yReversed));
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen
                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("You'll push the robot forward in a straight line along a field wall for "
                            + "exactly " + testDistance(FORWARD_DISTANCE) + ", then push it left "
                            + "for " + testDistance(LEFT_DISTANCE) + ". This will measure the following:\n"
                            + "\n"
                            + "\u2022 <b>ticksPerMm</b> is the number of ticks each encoder measures "
                            + "per millimeter traveled.\n"
                            + "\u2022 <b>xReversed</b> and <b>yReversed</b> indicate if the respective "
                            + "encoder results need to be reversed.\n"
                            + "\n"
                            + "Press " + screens.buttons + ".");
                    io.end();
                } else if (screens.index == 1) { // Measure screen
                    updateGamepadDriving();
                    io.canvas(Io.Background.BLANK); // Clear the field
                    screens.showHistory(io); // Show measurement history and advise when done

                    io.out("To start a measurement, align the robot by hand to its starting point "
                            + "aligned to a field wall, with room ahead for " + testDistance(FORWARD_DISTANCE) + ".");
                    io.out("\n\n");
                    io.out("Press " + A + " when in position, " + screens.buttons + ".");
                    io.end();

                    if (io.ok()) {
                        PinpointPushResult result = measurePinpoint(screens.header);
                        if (result != null)
                            screens.registerResult(result);
                    }
                }
            }
        }

        // Tune the appropriate sensor:
        void tune() {
            if (usePinpoint)
                tunePinpoint();
            else
                tuneOtos();
        }
    }

    /**
     * Class to encapsulate all Spin Tuner logic.
     */
    class SpinTuner {
        // Struct to represent OTOS Spin Tuner results:
        class OtosSpinResult extends Result {
            double trackWidthTicks;
            double otosAngularScalar;
            double otosOffsetX;
            double otosOffsetY;

            public OtosSpinResult(double trackWidthTicks, double otosAngularScalar, double otosOffsetX, double otosOffsetY) {
                this.trackWidthTicks = trackWidthTicks;
                this.otosAngularScalar = otosAngularScalar;
                this.otosOffsetX = otosOffsetX;
                this.otosOffsetY = otosOffsetY;
            }

            @Override
            public String getHeader() {
                return "<b>trackWidthTicks</b>, <b>angularScalar</b>, <b>(x, y) offset</b>";
            }

            @Override
            public String getValues() {
                return String.format("%.2f, %.4f, (%.3f\", %.3f\")", trackWidthTicks, otosAngularScalar, otosOffsetX, otosOffsetY);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.trackWidthTicks = trackWidthTicks;
                parameters.params.otos.angularScalar = otosAngularScalar;
                parameters.params.otos.offset.x = otosOffsetX;
                parameters.params.otos.offset.y = otosOffsetY;
            }
        }

        // Struct to represent Pinpoint Spin Tuner results:
        class PinpointSpinResult extends Result {
            double trackWidthTicks;
            double xOffset;
            double yOffset; // Sensor offset, in millimeters

            public PinpointSpinResult(double trackWidthTicks, double xOffset, double yOffset) {
                this.trackWidthTicks = trackWidthTicks;
                this.xOffset = xOffset;
                this.yOffset = yOffset;
            }

            @Override
            public String getHeader() {
                return "<b>trackWidthTicks</b>, <b>(x, y) offset</b>";
            }

            @Override
            public String getValues() {
                return String.format("%.2f, (%.1fmm, %.1fmm)", trackWidthTicks, xOffset, yOffset);
            }

            @Override
            public void applyTo(TuneParameters parameters) {
                parameters.params.trackWidthTicks = trackWidthTicks;
                parameters.params.pinpoint.xOffset = xOffset;
                parameters.params.pinpoint.yOffset = yOffset;
            }
        }

        final double REVOLUTION_COUNT = 10.0; // Number of revolutions to use
        final double SPIN_POWER = 0.5; // Speed of the revolutions

        // Detail about the most recent result:
        String detail;

        // Persisted state for initiateSparkFunRotation and updateSparkFunRotation:
        double previousAccumulationHeading;
        double accumulatedRotation;

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
        void initiateRotationAccumulation() {
            accumulatedRotation = 0;
            previousAccumulationHeading = getSensorHeading();
        }

        // Call this regularly to update the tracked amount of rotation:
        void updateRotation() {
            double newAccumulationHeading = getSensorHeading();
            accumulatedRotation += normalizeAngle(newAccumulationHeading - previousAccumulationHeading);
            previousAccumulationHeading = newAccumulationHeading;
        }

        // 'updateRotationAndGetOtosPose()' is the same as 'updateRotation()' except that it
        // additionally atomically returns the current pose (a performance benefit by avoiding
        // a second call to the expensive 'getPosition()').
        Pose2D updateRotationAndGetOtosPose() {
            Pose2D newPose = drive.otosDriver.getPosition();
            accumulatedRotation += normalizeAngle(newPose.h - previousAccumulationHeading);
            previousAccumulationHeading = newPose.h;
            return newPose;
        }

        // Draw the spin sample points, plus the optional best-fit circle, on FTC Dashboard:
        void drawSpinPoints(List<Point> points, Circle circle) {
            io.begin();
            Canvas canvas = io.canvas(Io.Background.GRID);
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
                if (duration == RAMP_TIME)
                    break; // ===>
            }
        }

        // Process the OTOS spin results:
        OtosSpinResult processOtos(Point clusterCenter, Circle center, double totalMeasuredRotation, double distancePerRevolution, double wallHeadingDelta) {
            double measuredCircles = totalMeasuredRotation / (2 * Math.PI);

            // If wallHeadingDelta is positive 30 degrees and we did 10 circles, that means that
            // the gyro undershot by a factor of 3570 / 3600. Adjust the gyro's angularScalar
            // constant to account for that:
            double angularScalar = (totalMeasuredRotation - wallHeadingDelta) / totalMeasuredRotation;

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
            detail = String.format("Sensor thinks %.2f circles were completed.\n\n", measuredCircles);
            detail += String.format("Cluster center: (%.2f, %.2f)\n", clusterOffset.x, clusterOffset.y);
            detail += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", rawOffset.x, rawOffset.y, center.radius);
            detail += String.format("Radius-corrected position: (%.2f, %.2f)\n", offset.x, offset.y);
            detail += String.format("Angular delta: %.2f\u00b0, resulting scalar: %.4f\n\n", Math.toDegrees(wallHeadingDelta), angularScalar);
            detail += String.format("Track width: %.2f\"\n", trackWidth);
            detail += "\n";

            // Do some sanity checking on the results:
            if ((Math.abs(offset.x) > 12) || (Math.abs(offset.y) > 12)) {
                dialog.warning("The calculated OTOS offset to the center of rotation "
                        + "is (%.2f, %.2f) and is clearly bogus.", offset.x, offset.y);
                return null; // ====>
            } else if ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
                dialog.warning("The calculated OTOS angular scalar of %.4f is "
                        + "out of valid range. Did you properly align the robot on the wall the same "
                        + "way at both the start and end of this test?", angularScalar);
                return null; // ====>
            }

            return new OtosSpinResult(trackWidthTicks, angularScalar, clusterOffset.x, clusterOffset.y);
        }

        // Process the Pinpoint spin results:
        PinpointSpinResult processPinpoint(double totalMeasuredRotation, double xTicks, double yTicks, double distancePerRevolution, double wallHeadingDelta) {
            double measuredCircles = totalMeasuredRotation / (2 * Math.PI);

            // If wallHeadingDelta is positive 30 degrees and we did 10 circles, that means that
            // the gyro undershot by a factor of 3570 / 3600. Adjust the gyro's angularScalar
            // constant to account for that:
            double yawScalar = (totalMeasuredRotation - wallHeadingDelta) / totalMeasuredRotation;

            // Once the reversed settings are applied, the X wheel increases when moving forward,
            // the Y wheel increases when moving to the left:
            if (drive.PARAMS.pinpoint.xReversed)
                xTicks = -xTicks;
            if (drive.PARAMS.pinpoint.yReversed)
                yTicks = -yTicks;

            // We turned counter-clockwise so the X wheel result needs to be negated while the Y
            // wheel result does not:
            double xOffset = -xTicks / (currentParameters.params.pinpoint.ticksPerMm * REVOLUTION_COUNT * 2 * Math.PI);
            double yOffset =  yTicks / (currentParameters.params.pinpoint.ticksPerMm * REVOLUTION_COUNT * 2 * Math.PI);

            // 'Track width' is really the radius of the circle needed to make a complete rotation:
            double trackWidth = distancePerRevolution / (2 * Math.PI);
            double trackWidthTicks = trackWidth / drive.PARAMS.inPerTick;

            // Remember detail about these results:
            detail = String.format("Sensor thinks %.2f circles were completed.\n\n", measuredCircles);
            detail += String.format("Location offset: (%.1fmm, %.1fmm)\n", xOffset, yOffset);
            detail += String.format("Yaw delta: %.2f\u00b0, resulting scalar: %.4f\n\n", Math.toDegrees(wallHeadingDelta), yawScalar);

            return new PinpointSpinResult(trackWidthTicks, xOffset, yOffset);
        }

        // Do the Spin measurement step for OTOS:
        OtosSpinResult measureOtos(String header) {
            double wallStartHeading = getSensorHeading();

            // Let the user position the robot:
            io.message(header + "Now drive the robot far enough away from the wall (and any objects) so "
                    + "that it can freely rotate in place."
                    + "\n\nPress "+A+" when ready for the robot to rotate, "+B+" to cancel.");

            if (poll.okCancelWithDriving()) {
                LinkedList<Point> points = new LinkedList<>();

                // Start spinning and then start tracking the accumulated rotation:
                rampMotorsSpin(drive, SPIN_POWER);
                initiateRotationAccumulation();

                // Prepare for calculating how far the drive wheels have traveled:
                double voltageSum = 0;
                int voltageSamples = 0;
                double startTime = time();

                // Do some preparation for termination:
                final double terminationRotation = REVOLUTION_COUNT * 2 * Math.PI;
                double farthestDistance = 0;
                Point farthestPoint = new Point(0, 0);

                Point originPosition = new Point(0, 0); // The origin of the start of every circle
                double nextCircleRotation = 0;

                Pose2D offsetStartPosition = updateRotationAndGetOtosPose();
                double offsetStartHeading = offsetStartPosition.h;
                Point rawPosition = new Point(offsetStartPosition.x, offsetStartPosition.y);

                // Now do all of the full-speed spins:
                boolean success = false;
                do {
                    // Sample the voltage to compute an average later:
                    voltageSum += drive.voltageSensor.getVoltage();
                    voltageSamples++;

                    // Check if we're at the start of a new circle:
                    if (accumulatedRotation >= nextCircleRotation) {
                        // Remember the raw position as the start of the new circle:
                        originPosition = rawPosition;
                        nextCircleRotation += 2 * Math.PI;
                    }

                    // Now that we've potentially done the last adjustment, see if we're all done:
                    double rotationsRemaining = (terminationRotation - accumulatedRotation) / (2 * Math.PI);
                    if (rotationsRemaining <= 0) {
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

                    io.begin();
                    io.out(header);
                    io.out("%.2f rotations remaining, %d samples", rotationsRemaining, points.size());
                    io.out("\n\nPress "+B+" to abort.");
                    io.end();

                    // Update for next iteration of the loop:
                    Pose2D rawPose = updateRotationAndGetOtosPose();
                    rawPosition = new Point(rawPose.x, rawPose.y);
                } while (opModeIsActive() && !io.cancel());

                double endTime = time();

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                io.message("Now drive the robot to snugly align it against the wall in the "
                        + "same orientation as it started."
                        + "\n\nDrive the robot to its wall position, press " + A + " when done, " + B + " to cancel.");
                if ((success) && poll.okCancelWithDriving()) {
                    double wallHeadingDelta = normalizeAngle(getSensorHeading() - wallStartHeading);

                    Point clusterCenter = clusterCenter(points);
                    Circle circle = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

                    // Draw results with the fitted circle:
                    drawSpinPoints(points, circle);

                    double averageVoltage = voltageSum / voltageSamples;
                    double averageWheelVelocity = (SPIN_POWER * averageVoltage - drive.PARAMS.kS) /
                            (drive.PARAMS.kV / drive.PARAMS.inPerTick); // Velocity in inches per second

                    double distancePerRevolution = averageWheelVelocity * (endTime - startTime) / REVOLUTION_COUNT;
                    return processOtos(clusterCenter, circle, accumulatedRotation, distancePerRevolution, wallHeadingDelta);
                }
            }
            return null;
        }

        // Do the Spin measurement step for the Pinpoint sensor:
        PinpointSpinResult measurePinpoint(String header) {
            double wallStartHeading = getSensorHeading();

            // Let the user position the robot:
            io.message("Now drive the robot far enough away from the wall (and any objects) so "
                    + "that it can freely rotate in place."
                    + "\n\nPress "+A+" when ready for the robot to rotate, "+B+" to cancel.");

            if (poll.okCancelWithDriving()) {
                // Start spinning and then start tracking the accumulated rotation:
                rampMotorsSpin(drive, SPIN_POWER);
                initiateRotationAccumulation();

                // Prepare for calculating how far the drive wheels have traveled:
                double voltageSum = 0;
                int voltageSamples = 0;
                double startTime = time();

                // Do some preparation for termination:
                final double terminationRotation = REVOLUTION_COUNT * 2 * Math.PI;

                drive.pinpointDriver.update();
                double xEncoderStart = drive.pinpointDriver.getEncoderX();
                double yEncoderStart = drive.pinpointDriver.getEncoderY();

                // Now do all of the full-speed spins:
                boolean success = false;
                do {
                    // Sample the voltage to compute an average later:
                    voltageSum += drive.voltageSensor.getVoltage();
                    voltageSamples++;

                    double rotationsRemaining = (terminationRotation - accumulatedRotation) / (2 * Math.PI);
                    if (rotationsRemaining <= 0) {
                        success = true;
                        break; // ====>
                    }

                    io.begin();
                    io.out(header);
                    io.out("%.2f rotations remaining", rotationsRemaining);
                    io.out("\n\nPress "+B+" to abort.");
                    io.end();

                    // Update for next iteration of the loop:
                    updateRotation();
                } while (opModeIsActive() && !io.cancel());

                double endTime = time();

                drive.pinpointDriver.update();
                double xTicks = drive.pinpointDriver.getEncoderX() - xEncoderStart;
                double yTicks = drive.pinpointDriver.getEncoderY() - yEncoderStart;

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                io.message("Now drive the robot to snugly align it against the wall in the "
                        + "same orientation as it started."
                        + "\n\nDrive the robot to its wall position, press " + A + " when done, " + B + " to cancel");
                if ((success) && poll.okCancelWithDriving()) {
                    double wallHeadingDelta = normalizeAngle(getSensorHeading() - wallStartHeading);

                    double averageVoltage = voltageSum / voltageSamples;
                    double averageWheelVelocity = (SPIN_POWER * averageVoltage - drive.PARAMS.kS) /
                            (drive.PARAMS.kV / drive.PARAMS.inPerTick); // Velocity in inches per second

                    double distancePerRevolution = averageWheelVelocity * (endTime - startTime) / REVOLUTION_COUNT;
                    return processPinpoint(accumulatedRotation, xTicks, yTicks, distancePerRevolution, wallHeadingDelta);
                }
            }
            return null;
        }

        // This is the robot spin test for calibrating the optical sensor angular scale and offset.
        //
        // This has a dependency on knowing kS and kV (for purposes of calculating wheel velocity
        // to determine distance traveled for 'trackWidthTicks').
        void tune() {
            configureToDrive(true); // Use MecanumDrive

            Action preview = drive.actionBuilder(new Pose2d(0, -60, 0))
                    .strafeTo(new Vector2d(0, -48))
                    .turn(2 * 2 * Math.PI)
                    .strafeTo(new Vector2d(0, -60))
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);

            Screens screens = new Screens(Tuner.SPIN, new String[]{"Preview", "Measure", "Detail"});

            if (usePinpoint) {
                screens.registerResult(new PinpointSpinResult(
                        currentParameters.params.trackWidthTicks,
                        currentParameters.params.pinpoint.xOffset,
                        currentParameters.params.pinpoint.yOffset));
            } else {
                // Zero these settings for the purpose of this test:
                drive.otosDriver.setOffset(new Pose2D(0, 0, 0));
                drive.otosDriver.setAngularScalar(1.0);

                screens.registerResult(new OtosSpinResult(
                        currentParameters.params.trackWidthTicks,
                        currentParameters.params.otos.angularScalar,
                        currentParameters.params.otos.offset.x,
                        currentParameters.params.otos.offset.y));
            }
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("You'll position the robot against a wall, then drive it out so that the robot "
                            + "can rotate in place %.1f times, then drive the robot against the wall "
                            + "again. This will measure the following:\n\n", REVOLUTION_COUNT);
                    if (usePinpoint) {
                        io.out("\u2022 <b>trackWidthTicks</b> accounts for how far the drive wheels travel when "
                                + "the robot turns 360\u00b0.\n"

                                + "\u2022 <b>xOffset</b>, <b>yOffset</b> are the offsets "
                                + "of the respective X and Y encoder pods from the center of "
                                + "rotation.\n");

                    } else {
                        io.out("\u2022 <b>trackWidthTicks</b> accounts for how far the drive wheels travel when "
                                + "the robot turns 360\u00b0.\n"

                                + "\u2022 <b>otos.angularScalar</b> is the calibration factor to apply to make "
                                + "the OTOS gyro more accurate.\n"

                                + "\u2022 <b>otos.offset position</b> is the location on the robot where the "
                                + "OTOS sensor is mounted, relative to the center of rotation.\n");
                    }
                    io.out("\nPress "+ screens.buttons+".");
                    io.end();

                } else if (screens.index == 1) { // Measure screen

                    updateGamepadDriving(); // Let the user drive
                    io.canvas(Io.Background.GRID); // Clear the field
                    screens.showHistory(io); // Show measurement history and advise when done
                    io.out("To start a measurement, carefully drive the robot so it's snugly "
                            + "aligned against a wall, facing forward. This marks the start orientation for "
                            + "calibration."
                            + "\n\n"
                            + "Press "+A+" to start once the robot is snug against a wall, "+ screens.buttons+".");
                    io.end();
                    if (io.ok()) {
                        Result result;
                        if (usePinpoint) {
                            result = measurePinpoint(screens.header);
                        } else {
                            result = measureOtos(screens.header);
                        }
                        if (result != null)
                            screens.registerResult(result);
                    }

                } else if (screens.index == 2) { // Details screen

                    if (screens.history.size() <= 1) {
                        io.out("This will contain detail about the most recent result.\n\n");
                    } else {
                        io.out("Here is detail about the most recent result. Feel free to ignore.\n\n");
                        io.out(detail);
                    }
                    io.out("Press " + screens.buttons + ".");
                    io.end();

                }
            }

            // Set/restore the hardware settings:
            initializeTrackerHardware();
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
            public String getHeader() {
                return "<b>kS</b>, <b>kV</b>";
            }

            @Override
            public String getValues() {
                return String.format("%.3f, %.3f", kS, kV);
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

        // Ramp the motors up or down to or from the target spin speed. Turns counter-clockwise
        // when provided a positive power value.
        /** @noinspection SameParameterValue*/
        void rampMotors(MecanumDrive drive, double targetPower) {
            final double RAMP_TIME = 0.25; // Seconds
            double startPower = drive.rightFront.getPower();
            double deltaPower = targetPower - startPower;
            double startTime = time();
            while (opModeIsActive()) {
                double duration = Math.min(time() - startTime, RAMP_TIME);
                double power = (duration / RAMP_TIME) * deltaPower + startPower;
                drive.rightFront.setPower(power);
                drive.rightBack.setPower(power);
                drive.leftFront.setPower(power);
                drive.leftBack.setPower(power);
                if (duration == RAMP_TIME)
                    break; // ===>
            }
        }

        // Helper that drives an accelerating straight line and samples voltage and velocity:
        /** @noinspection SameParameterValue*/
        ArrayList<Point> driveLine(String header, double direction) {
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

                Pose2D velocityVector = getSensorVelocity();
                double velocity = Math.hypot(velocityVector.x, velocityVector.y);

                // Discard zero velocities that will happen when the provided power isn't
                // enough yet to overcome static friction:
                if (velocity > VELOCITY_EPSILON) {
                    double power = powerFactor * drive.voltageSensor.getVoltage();
                    points.add(new Point(velocity, power));
                }
                Pose2D position = getSensorPosition();
                double distance = Math.hypot(position.x, position.y);

                // We're done if we've reach the maximum target voltage or gone far enough:
                if ((powerFactor > MAX_POWER_FACTOR) || (distance > DISTANCE)) {
                    if (Math.abs(position.x) < 0.9 * DISTANCE) {
                        dialog.warning("Odometry is inconsistent with movement direction. "
                                + "Rerun first tuners.");
                        return null; // ====>
                    }
                    rampMotors(drive, 0);
                    sleep(500); // Sleep a bit to quiesce
                    return points; // ====>
                }

                double remaining = Math.max(0, DISTANCE - distance);
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
            // Do one forward and back run:
            ArrayList<Point> forwardSamples = driveLine(header, 1.0);
            if (forwardSamples != null) {
                ArrayList<Point> reverseSamples = driveLine(header, -1.0);
                if (reverseSamples != null) {
                    findMaxValues(max, forwardSamples);
                    findMaxValues(max, reverseSamples);
                    if (max.x == 0) {
                        dialog.warning("The optical tracking sensor returned only zero velocities. "
                                + "Is it working properly?");
                        return null; // ====>
                    } else {
                        // Draw the results to the FTC dashboard:
                        io.begin();
                        Canvas canvas = io.canvas(Io.Background.BLANK);

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
                        io.end(); // Send the canvas results

                        double kS = bestFitLine.intercept;
                        double kV = bestFitLine.slope * currentParameters.params.inPerTick;

                        graphExplanation = String.format("The graph in the Field view shows "
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
        void tune() {
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

            Screens screens = new Screens(Tuner.ACCELERATING, new String[]{"Preview", "Measure"});
            screens.registerResult(new AcceleratingResult(
                    currentParameters.params.kS, currentParameters.params.kV));
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen

                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out("The robot will drive forward and backward for "
                            + testDistance(DISTANCE) + ". It will start slowly but get faster and "
                            + "faster in each direction. This will measure the following:\n");
                    io.out("\n"
                            + "\u2022 <b>kS</b> is the motor voltage required to overcome static "
                            + "friction and make the robot move when it's stopped. It's the "
                            + "intercept on the velocity/voltage graph.\n"
                            + "\u2022 A motor's velocity is proportional to the voltage applied, and "
                            + "<b>kV</b> is the ratio of motor voltage to robot velocity. It's the "
                            + "slope of the velocity/voltage graph.\n"
                            + "\n"
                            + "Press " + screens.buttons + ".");
                    io.end();

                } else if (screens.index == 1) { // Measure screen
                    updateGamepadDriving(); // Let the user drive
                    if (screens.switched)
                        io.canvas(Io.Background.BLANK);

                    io.out(graphExplanation);
                    screens.showHistory(io); // Show measurement history and advise when done

                    io.out("To start a measurement, drive the robot to a spot that is clear "
                            + "in front for " + testDistance(DISTANCE) + ".\n"
                            + "\n"
                            + "Press " + A + " to start the robot (ensure " + clearanceDistance(DISTANCE)
                            + " of clearance ahead), " + screens.buttons + ".");
                    io.end();
                    if (io.ok()) {
                        AcceleratingResult result = measure(screens.header);
                        if (result != null)
                            screens.registerResult(result);
                    }
                }
            }
        }
    }

    /**
     * Class to encapsulate all Interface Feed Forward Tuner logic.
     */
    class InteractiveFeedForwardTuner {
        final int DISTANCE = 72; // Test distance in inches
        final String TARGET_COLOR = "#4CAF50"; // Green
        final String ACTUAL_HIGHLIGHT_COLOR = "#3F51B5"; // Blue
        final String ACTUAL_LOWLIGHT_COLOR = "#808080"; // Grey
        final double GRAPH_THROTTLE = 0.1; // Only update the graph every 0.1 seconds

        // Allocate a repository for all of our velocity samples:
        class Sample {
            final double time; // Seconds
            final double target; // Target velocity, inches/s
            final double actual; // Actual velocity, inches/s
            final boolean isHighlight; // True if this sample should be highlighted

            public Sample(double time, double target, double actual, boolean isHighlight) {
                this.time = time; this.target = target; this.actual = actual; this.isHighlight = isHighlight;
            }
        }

        double runStartTime; // Start time of the run
        double profileStartTime; // Start time of the profile cycle, there are two in every run
        double maxTargetVelocity; // Maximum target velocity, known ahead of time
        double maxActualVelocity; // Maximum actual velocity, accumulated from measurements
        boolean movingForwards;
        double maxDuration; // Max run duration, accumulated across runs
        double lastGraphTime; // Last time the graph was updated, in seconds
        LinkedList<Sample> samples; // The accumulated samples

        // Start a run of the robot:
        TimeProfile startRun(double maxVelocityFactor) {
            stopMotors(); // Stop the user's driving
            movingForwards = true;
            runStartTime = time();
            profileStartTime = time();
            maxTargetVelocity = MecanumDrive.PARAMS.maxWheelVel * maxVelocityFactor;
            maxActualVelocity = 0;
            samples = new LinkedList<>();

            // Reset the start position on every cycle. This ensures that getVelocity().x
            // is the appropriate velocity to read, and it resets after we reposition
            // the robot:
            drive.setPose(new Pose2d(-DISTANCE / 2.0, 0, 0));
            return new TimeProfile(constantProfile(
                    DISTANCE, 0.0,
                    maxTargetVelocity,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel).baseProfile);
        }

        // Execute one iteration of the run logic. Returns the TimeProfile if it's still active,
        // or null if the profile is complete.
        TimeProfile run(TimeProfile profile, boolean highlightKv) {
            double time = time();
            double elapsedTime = time - profileStartTime;
            if (elapsedTime > profile.duration) {
                if (movingForwards) {
                    movingForwards = false;
                    profileStartTime = time;
                } else {
                    if (maxActualVelocity < 0.5 * maxTargetVelocity) {
                        dialog.warning("Unexpectedly low robot velocity. Examine robot or redo "
                                + "first tuners.");
                    }
                    return null; // ====> All done!
                }
            }

            DualNum<Time> dualVelocity = profile.get(elapsedTime).drop(1);
            if (!movingForwards) {
                dualVelocity = dualVelocity.unaryMinus();
            }

            // Calculate the new sample and log it:
            Pose2D velocityPose = getSensorVelocity();
            double targetVelocity = dualVelocity.get(0);
            double targetAcceleration = dualVelocity.get(1);
            double actualVelocity = Math.signum(velocityPose.x) * Math.hypot(velocityPose.x, velocityPose.y);
            maxActualVelocity = Math.max(maxActualVelocity, Math.abs(actualVelocity));
            maxDuration = Math.max(maxDuration, time - runStartTime);

            // Do some sanity checking:
            if (Math.abs(velocityPose.y) > 0.3 * maxTargetVelocity) {
                dialog.warning("Odometry results are inconsistent with movement. "
                        + "Re-run first tuners.");
                return null; // ====>
            }

            // Highlight the flat tops and bottoms if tuning kV, and the acceleration and
            // deceleration (but not regenerative) portions if tuning kA:
            boolean highlight;
            if (highlightKv) {
                highlight = Math.abs(targetVelocity) == maxTargetVelocity;
            } else {
                highlight = (((targetAcceleration >= 0) && (targetVelocity >= 0))
                        || ((targetAcceleration <= 0) && (targetVelocity <= 0)))
                        && (Math.abs(targetVelocity) < maxTargetVelocity);
            }
            samples.addLast(new Sample(time, targetVelocity, actualVelocity, highlight));

            // Throttle the Dashboard updates so that it doesn't get overwhelmed as it
            // has very bad rate control:
            if (time - lastGraphTime > GRAPH_THROTTLE) {
                lastGraphTime = time;

                io.begin(); // Canvas update
                Canvas canvas = io.canvas(Io.Background.GRID);
                canvas.setTranslation(0, 72);
                double maxGraphVelocity = Math.max(maxActualVelocity, maxTargetVelocity);
                double xScale = (maxDuration == 0) ? 1 : 144 / maxDuration;
                double yScale = (maxGraphVelocity == 0) ? 1 : 72 / maxGraphVelocity;

                // For efficiency, do a first pass to find the length of every sequence of
                // highlight/no-highlight samples:
                ArrayList<Integer> sequences = new ArrayList<>();
                int sequenceCount = 0;
                boolean sequenceIsHighlight = samples.get(0).isHighlight;
                for (Sample sample: samples) {
                    if (sample.isHighlight == sequenceIsHighlight) {
                        sequenceCount++;
                    } else {
                        sequences.add(sequenceCount);
                        sequenceIsHighlight = sample.isHighlight;
                        sequenceCount = 1;
                    }
                }
                if (sequenceCount != 0)
                    sequences.add(sequenceCount);

                // Now render every sequence of polylines:
                int sequenceStart = 0;
                for (int count: sequences) {
                    double[] xPoints = new double[count];
                    double[] yTargets = new double[count];
                    double[] yActuals = new double[count];

                    boolean isHighlight = samples.get(sequenceStart).isHighlight;
                    for (int i = 0; i < count; i++) {
                        Sample sample = samples.get(sequenceStart + i);
                        xPoints[i] = (sample.time - (time - maxDuration)) * xScale;
                        yTargets[i] = sample.target * yScale;
                        yActuals[i] = sample.actual * yScale;
                    }
                    sequenceStart += count;

                    canvas.setStroke(TARGET_COLOR);
                    canvas.strokePolyline(xPoints, yTargets);
                    canvas.setStroke(isHighlight ? ACTUAL_HIGHLIGHT_COLOR : ACTUAL_LOWLIGHT_COLOR);
                    canvas.strokePolyline(xPoints, yActuals);
                }
                io.end();
            }

            // Set the motors to the appropriate values:
            MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

            double power = feedForward.compute(dualVelocity) / drive.voltageSensor.getVoltage();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));
            return profile;
        }

        // Sync the relevant tune parameters for this test:
        void sync(TuneParameters destination, TuneParameters source) {
            destination.params.kV = source.params.kV;
            destination.params.kA = source.params.kA;
        }

        // Tune the kV and kA feed forward parameters:
        void tune() {
            configureToDrive(false); // Don't use MecanumDrive

            // Trigger a reset the first time into the loop:
            profileStartTime = 0; // Profile start time, in seconds
            runStartTime = 0; // Cycle start time, in second (each cycle consumes 2 profiles)
            movingForwards = false;
            maxDuration = 4; // Maximum graph duration, in seconds
            lastGraphTime = 0;
            samples = new LinkedList<>();

            double maxVelocityFactor = 0.7;
            int qeuedStarts = 0;
            TimeProfile profile = null;

            // Disable all axial gains so that backward and forward behavior is not affected by the
            // PID/Ramsete algorithm. It's okay for the lateral and rotation gains to be either zero
            // or non-zero:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.axialGain = 0;
            testParameters.params.axialVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            NumericInput vInput = new NumericInput(drive.PARAMS, "kV", -2, 3, 0.000001, 20);
            NumericInput aInput = new NumericInput(drive.PARAMS, "kA", -3, 4, 0, 1);

            Action preview = drive.actionBuilder(new Pose2d(-DISTANCE/2.0, 0, 0))
                    .lineToX(DISTANCE/2.0)
                    .lineToX(-DISTANCE/2.0)
                    .build();
            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);

            Screens screens = new Screens(Tuner.FEED_FORWARD,
                    new String[]{"Preview", "Adjust kV", "Adjust kA"},
                    this::sync);
            while (opModeIsActive()) {
                // Note we don't do screens.update() here because we restrict when it's called
                if (screens.index == 0) { // Preview screen
                    if (!screens.update())
                        break; // ====>

                    io.begin();
                    io.out(screens.header);
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
                            + "Press " + screens.buttons + ".");
                    io.end();
                } else { // kV and kA tuning screens
                    io.begin();
                    io.out(screens.header);
                    if (screens.switched) {
                        io.canvas(Io.Background.BLANK); // Clear the field
                    }

                    if (screens.index == 1) {
                        io.out("&emsp;kV: <big><big>%s</big></big>\n", vInput.update());
                        io.out("&emsp;Max velocity is <b>%.0f%%</b>.\n\n", maxVelocityFactor * 100.0);
                        io.out("Once started, view the graph in FTC Dashboard's field view and adjust "
                                + "<b>kV</b> to make the horizontal lines as close as possible in height. "
                                + "<b>vTarget</b> is green, <b>vActual</b> is blue, <i>kV = vTarget / vActual</i>. ");
                    } else {
                        io.out("&emsp;kA: <big><big>%s</big></big>\n", aInput.update());
                        io.out("&emsp;Max velocity is <b>%.0f%%</b>.\n\n", maxVelocityFactor * 100.0);
                        io.out("Once started, view the graph in FTC Dashboard's field view and adjust "
                                + "<b>kA</b> to shift <b>vActual</b> left and right so the angled lines overlap "
                                + "where the robot accelerates. It's too much if the horizontal line gets overshot. "
                                + "Don't worry about the gray sloping portions if they bulge out. ");
                    }
                    io.out(DPAD_UP_DOWN + " to change value, " + DPAD_LEFT_RIGHT + " to move "
                            + "the cursor.\n\n");

                    if (io.ok())
                        qeuedStarts++; // Let starts be queued up even while running

                    // If there's a profile, that means we're actively moving:
                    if (profile != null) {
                        io.out("Press " + B + " to cancel and stop the robot.\n");
                        io.end();

                        // Run the processing logic for this tuner:
                        profile = run(profile, screens.index == 1);
                        if (io.cancel()) {
                            // Cancel the current cycle but remain in this test:
                            stopMotors();
                            qeuedStarts = 0;
                            profile = null;
                        }
                    } else {
                        io.out("Changing max velocity via " + TRIGGERS + " will lengthen or "
                                + "shorten the horizontal lines.\n\n");
                        io.out(A + " to start the robot (ensure " + clearanceDistance(DISTANCE)
                                + " of forward clearance), " + screens.buttons + ".");
                        io.end();

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
                            profile = startRun(maxVelocityFactor);
                        }

                        if (!screens.update()) // Update the screen manager
                            break; // ====>
                    }
                }
            }

            // We're done, undo any temporary state we set:
            MecanumDrive.PARAMS = currentParameters.params;
            stopMotors();
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
            public String getHeader() {
                return "<b>lateralInPerTick</b>";
            }

            @Override
            public String getValues() {
                return String.format("%.3f", lateralInPerTick);
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
            ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(-15, 25);

            // Strafe left and then right:
            Pose2d startPose1 = new Pose2d(0, -DISTANCE / 2.0, 0);
            drive.setPose(startPose1);
            if (runCancelableAction(header, drive.actionBuilder(startPose1)
                    .strafeTo(new Vector2d(0, DISTANCE / 2.0), null, accelConstraint)
                    .build())) {

                sleep(500); // Quiesce between runs

                double actualDistance1 = drive.pose.position.y - startPose1.position.y;
                double error1 = Math.abs(drive.pose.position.x - startPose1.position.x);

                Pose2d startPose2 = new Pose2d(0, DISTANCE / 2.0, 0);
                drive.setPose(startPose2);
                if (runCancelableAction(header, drive.actionBuilder(startPose2)
                        .strafeTo(new Vector2d(0, -DISTANCE / 2.0), null, accelConstraint)
                        .build())) {

                    double actualDistance2 = startPose2.position.y - drive.pose.position.y;
                    double error2 = Math.abs(startPose2.position.x - drive.pose.position.x);

                    double multiplier1 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance1 / DISTANCE);
                    double multiplier2 = MecanumDrive.PARAMS.lateralInPerTick * (actualDistance2 / DISTANCE);

                    if ((error1 > 0.4 * DISTANCE) || (error2 > 0.4 * DISTANCE)) {
                        dialog.warning("Odometry results are inconsistent with movement. "
                                + "Re-run first tuners");
                        return null; // ====>
                    }

                    if (Math.min(multiplier1, multiplier2) < 0.25) {
                        dialog.warning("The measured distance is too low to be correct. "
                                + "Is the odometry sensor not working properly?");
                        return null; // ====>
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
        void tune() {
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

            Screens screens = new Screens(Tuner.LATERAL_MULTIPLIER, new String[]{"Preview", "Measure"});
            screens.registerResult(new LateralResult(currentParameters.params.lateralInPerTick));
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen

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
                            + "Press " + screens.buttons + ".");
                    io.end();

                } else if (screens.index == 1) { // Measure screen

                    if (screens.switched)
                        io.canvas(Io.Background.BLANK); // Clear the field
                    updateGamepadDriving(); // Let the user drive

                    // io.out(latestResult);
                    screens.showHistory(io);

                    io.out("Press " + A + " to start the robot (ensure " + clearanceDistance(DISTANCE)
                            + " of clearance to the left), " + screens.buttons + ".");
                    io.end();
                    if (io.ok()) {
                        LateralResult result = measure(screens.header, testParameters, screens.history);
                        if (result != null)
                            screens.registerResult(result);
                    }

                }
            }

            // Restore the kinematics:
            MecanumDrive.PARAMS = currentParameters.params;
            drive.initializeKinematics();
        }
    }

    /**
     * Class to encapsulate all interactive PID tuning logic.
     */
    class InteractivePidTuner {
        final int DISTANCE = 48; // Test distance in inches
        String errorSummary; // String describing the current amount of error
        double maxAxialError; // Maximum error accumulated over the current robot run
        double maxLateralError;
        double maxHeadingError;

        // Run the tuning update:
        boolean run(PidTunerType type) {
            // Execute the trajectory:
            boolean more = drive.doActionsWork(io.packet);
            if (!more)
                io.abortCanvas(); // Do this to keep the last frame shown

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

                errorSummary = "Max gain error: <b>";
                if (type == PidTunerType.AXIAL)
                    errorSummary += String.format("%.2f\"", maxAxialError);
                else if (type == PidTunerType.LATERAL)
                    errorSummary += String.format("%.2f\"", maxLateralError);
                else if (type == PidTunerType.HEADING)
                    errorSummary += String.format("%.2f\u00b0", Math.toDegrees(maxHeadingError));
                else
                    errorSummary += String.format("%.2f\", %.2f\", %.2f\u00b0",
                            maxAxialError, maxLateralError, Math.toDegrees(maxHeadingError));
                errorSummary += "</b>";

                if (!more) {
                    errorSummary += (type == PidTunerType.ALL) ? "\n" : ", ";
                    errorSummary += "End error: <b>";
                    if (type == PidTunerType.AXIAL)
                        errorSummary += String.format("%.2f\"", axialError);
                    else if (type == PidTunerType.LATERAL)
                        errorSummary += String.format("%.2f\"", lateralError);
                    else if (type == PidTunerType.HEADING)
                        errorSummary += String.format("%.2f\u00b0", Math.toDegrees(headingError));
                    else
                        errorSummary += String.format("%.2f\", %.2f\", %.2f\u00b0",
                                axialError, lateralError, Math.toDegrees(headingError));
                    errorSummary += "</b>";
                }
                errorSummary += "\n\n";
            }
            return more;
        }

        // Copy the relevant fields for this test when saving the state:
        void sync(PidTunerType type, TuneParameters destination, TuneParameters source) {
            if (type == PidTunerType.AXIAL) {
                destination.params.axialGain = source.params.axialGain;
                destination.params.axialVelGain = source.params.axialVelGain;
            } else if (type == PidTunerType.LATERAL) {
                destination.params.lateralGain = source.params.lateralGain;
                destination.params.lateralVelGain = source.params.lateralVelGain;
            } else if (type == PidTunerType.HEADING) {
                destination.params.headingGain = source.params.headingGain;
                destination.params.headingVelGain = source.params.headingVelGain;
            } else { // All case
                destination.params.axialGain = source.params.axialGain;
                destination.params.axialVelGain = source.params.axialVelGain;
                destination.params.lateralGain = source.params.lateralGain;
                destination.params.lateralVelGain = source.params.lateralVelGain;
                destination.params.headingGain = source.params.headingGain;
                destination.params.headingVelGain = source.params.headingVelGain;
            }
        }

        void tune(PidTunerType type) {
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
                        + "Tune these gains to reduce the forward/backward error between target and actual position:\n"
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
                        + "Tune these gains to reduce the lateral error between target and actual positions:\n"
                        + "\n"
                        + "\u2022 <b>lateralGain</b> sets the magnitude of response to the error. "
                        + "A higher value more aggressively corrects but can cause overshoot.\n"
                        + "\u2022 <b>lateralVelGain</b> is a damper and can reduce overshoot and oscillation.\n";
                clearance = "ensure " + clearanceDistance(DISTANCE) + " clearance to the left";
                adjective = "laterally ";
                gainNames = new String[] { "lateralGain", "lateralVelGain" };
                tuner = Tuner.LATERAL_GAIN;
                trajectory = trajectory.strafeTo(new Vector2d(0, DISTANCE)).strafeTo(new Vector2d(0, 0));
                preview = drive.actionBuilder(new Pose2d(0, -DISTANCE / 2.0, 0))
                        .strafeTo(new Vector2d(0, DISTANCE / 2.0))
                        .strafeTo(new Vector2d(0, -DISTANCE / 2.0))
                        .build();

            } else if (type == PidTunerType.HEADING) {
                overview = "The robot will rotate in place 180\u00b0. Tune these gains to reduce the "
                        + "error between target and actual headings:\n"
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
            screenNames.add("Preview");
            for (String gainName: gainNames) {
                screenNames.add(String.format("Tune %s", gainName));
                inputs.add(new NumericInput(drive.PARAMS, gainName, -1, 2, 0, 20));
            }

            TrajectoryPreviewer previewer = new TrajectoryPreviewer(io, preview);

            int queuedStarts = 0;
            Screens screens = new Screens(tuner, screenNames.toArray(new String[0]), (a, b)->sync(type, a, b));
            while (opModeIsActive()) {
                if (!screens.update())
                    break; // ====>

                io.begin();
                io.out(screens.header);
                if (screens.index == 0) { // Preview screen
                    previewer.update(); // Animate the trajectory preview
                    updateGamepadDriving(); // Let the user drive
                    io.out(overview);
                    io.out("\n"
                            + "These are essentially the <b>P</b> and <b>D</b> (<i>proportional</i> and "
                            + "<i>differential</i>) terms for a PID control system.\n"
                            + "\n"
                            + "Press " + screens.buttons + ".");
                    io.end();
                } else { // Tune screens
                    if (screens.switched)
                        errorSummary = "";

                    io.canvas(Io.Background.GRID); // Clear the field
                    int index = screens.index - 1;
                    NumericInput input = inputs.get(index);
                    io.out("&emsp;%s: <big><big>%s</big></big>\n", input.fieldName, input.update());

                    if ((index & 1) == 0) { // Tuning a proportional gain
                        io.out("\n"+ errorSummary);
                        io.out("Increase the gain to make the circles %scoincident and to minimize "
                                + "the maximum and final error. ", adjective);
                        io.out("Green is target, blue is actual. ");
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
                            + " to move the cursor.\n\n");

                    if (io.start())
                        queuedStarts++;

                    // Continue the trajectory, if any:
                    if (run(type)) {
                        io.out("Press " + B + " to cancel and stop the robot.");
                        io.end();
                        if (io.cancel()) {
                            // Cancel the current cycle but remain in this test:
                            drive.abortActions();
                            queuedStarts = 0;
                        }
                    } else {
                        updateGamepadDriving(); // Let the user drive

                        io.out("Press "+ A + " to start the robot (%s), %s.", clearance, screens.buttons);
                        io.end();

                        if (io.ok())
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
        String message = "The robot will drive forward 48 inches using a spline as previewed "
                + "in the field view. "
                + "It needs half a tile clearance on either side. ";
        String clearance = "(ensure " + clearanceDistance(48) + " clearance in front, half a "
                + "tile on either side)";
        runTrajectory(()->drive.actionBuilder(zeroPose)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(-60))
                .splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(180)), Math.toRadians(60))
                .endTrajectory()
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-0.0001)), Math.toRadians(-180))
                .build(), message, clearance);

        currentParameters.passedCompletionTest = true;
        currentParameters.save();
        updateTunerDependencies(Tuner.COMPLETION_TEST);
    }

    // Simple verification test for 'TrackWidth':
    void rotationTest() {
        configureToDrive(true); // Do use MecanumDrive

        io.message("To test <b>trackWidthTicks</b>, the robot will turn in-place for two complete "
                + "rotations.\n\nPress "+A+" to start the robot, "+B+" to cancel.");
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

            io.message("The robot should be facing close to the same direction as when it started. It it's "
                    + "not, run the spin tuner again to re-tune <b>trackWidthTicks</b>."
                    + "\n\nPress "+A+" to continue.");
            poll.ok();

            // Restore the parameters:
            MecanumDrive.PARAMS = currentParameters.params;
        }
    }

    // Examples:
    void splineExample() {
        runTrajectory(()->drive.actionBuilder(zeroPose)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build());
    }

    void lineToTurnExample() {
        runTrajectory(()->drive.actionBuilder(zeroPose)
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
        runTrajectory(()->drive.actionBuilder(zeroPose)
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
                    + "Double-tap the shift key in Android Studio, enter 'md.Params' to jump to the "
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
        drive.otosDriver.getVersionInfo(hwVersion, fwVersion);
        io.message("SparkFun OTOS hardware version: %d.%d, firmware version: %d.%d\n\n"
                        + "Press "+A+" to continue.",
                hwVersion.major, hwVersion.minor, fwVersion.major, fwVersion.minor);
        poll.ok();
    }

    // Show the goBilda Pinpoint version:
    public void pinpointVersionDialog() {
        io.message("goBilda Pinpoint hardware version: 0x%x, yaw scalar: %.5f\n\n"
                        + "Press "+A+" to continue.",
                drive.pinpointDriver.getDeviceVersion(), drive.pinpointDriver.getYawScalar());
        poll.ok();
    }

    public void retuneDialog() {
        io.message(Dialog.QUESTION_ICON + "Do you want to re-tune your robot ("
                + "maybe you did a big hardware change)? "
                + "This will walk you through the re-tuning step-by-step. It will also show your "
                + "new tuning results compared to your previous results.\n"
                + "\n"
                + "Press "+A+" to re-tune, "+B+" to cancel.");
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
        usePinpoint = drive.pinpointDriver != null;
        currentParameters = new TuneParameters(drive, TuneParameters.getSavedParameters());
        originalParameters = currentParameters.createClone();

        // Wait for the start button to be pressed:
        while (!isStarted()) {
            io.begin();
            io.out("<big><big><big><big><big><big><b><font color='%s'>Loony Tune!</font></b></big></big></big></big></big></big>\n", HIGHLIGHT_COLOR);
            io.out("<big><big>By Swerve Robotics, Woodinville</big></big>\n");
            io.out("<big><big><big><b><font color='%s'>Tap \u25B6 to begin", HIGHLIGHT_COLOR);
            Canvas canvas = io.canvas(Io.Background.BLANK);
            canvas.setFill(HIGHLIGHT_COLOR);
            canvas.fillText("Loony Tune!", -30, 8, "", 0, false);
            io.end();
        }

        if ((drive.otosDriver == null) && (drive.pinpointDriver == null)) {
            dialog.warning("Either a SparkFun OTOS or goBilda Pinpoint sensor must be initialized. ");
            return; // ====>
        }

        if (usePinpoint) {
            drive.pinpointDriver.update(); // Call update() to read the status register
            GoBildaPinpointDriver.DeviceStatus status = drive.pinpointDriver.getDeviceStatus();

            if (status == NOT_READY) {
                dialog.warning("The Pinpoint computer isn't responding. Check your I2C wiring.");
                return; // ====>
            }

            if ((status == FAULT_X_POD_NOT_DETECTED) ||
                (status == FAULT_Y_POD_NOT_DETECTED) ||
                (status == FAULT_NO_PODS_DETECTED)) {
                dialog.warning("The Pinpoint computer indicates that pods aren't connected. " +
                        "Check your encoder wiring.");
                return; // ====>
            }
        } else {
            if (!isOpticalTrackerResponsive()) {
                dialog.warning("The SparkFun OTOS sensor is not responding. Check your wiring.");
                return; // ====>
            }
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
                    + "Station, press the Start button to the left of " + X + ".)</font></small>");
            io.message("<big><big><big><font color='%s'>Press %s to begin tuning</font></big></big></big>\n"
                    + "<big>Make sure you can see the field view in FTC Dashboard.", HIGHLIGHT_COLOR, A);
            while (!isStopRequested() && !io.ok())
                io.redraw();
        }

        String pushDescription, spinDescription;
        if (usePinpoint) {
            pushDescription = "Push tuner (Pinpoint ticksPerMm, x/yReversed)";
            spinDescription = "Spin tuner (trackWidthTicks, Pinpoint x/y offset)";
        } else {
            pushDescription = "Push tuner (OTOS offset heading, linearScalar)";
            spinDescription = "Spin tuner (trackWidthTicks, OTOS x/y offset, angularScalar)";
        }

        // Dynamically build the list of tests:
        addTuner(Tuner.WHEEL_TEST,          this::wheelTest,                          "Wheel test (wheels, motors verification)");
        addTuner(Tuner.PUSH,                pushTuner::tune,                          pushDescription);
        addTuner(Tuner.ACCELERATING,        acceleratingTuner::tune,                  "Accelerating straight line tuner (kS, kV)");
        addTuner(Tuner.FEED_FORWARD,        feedForwardTuner::tune,                   "Interactive feed forward tuner (kV, kA)");
        addTuner(Tuner.SPIN,                spinTuner::tune,                          spinDescription);
        addTuner(Tuner.TRACKING_TEST,       this::trackingTest,                       "Tracking test (sensor verification)");
        addTuner(Tuner.LATERAL_MULTIPLIER,  lateralMultiplierTuner::tune,             "Lateral tuner (lateralInPerTick)");
        addTuner(Tuner.AXIAL_GAIN,          ()-> pidTuner.tune(PidTunerType.AXIAL),   "Interactive PiD tuner (axial gains)");
        addTuner(Tuner.LATERAL_GAIN,        ()-> pidTuner.tune(PidTunerType.LATERAL), "Interactive PiD tuner (lateral gains)");
        addTuner(Tuner.HEADING_GAIN,        ()-> pidTuner.tune(PidTunerType.HEADING), "Interactive PiD tuner (heading gains)");
        addTuner(Tuner.COMPLETION_TEST,     this::completionTest,                     "Completion test (overall verification)");
        addTuner(Tuner.RETUNE,              this::retuneDialog,                       "Re-tune");

        menu.addRunnable("More::Show accumulated parameter changes", this::updatedParametersDialog);
        if (drive.otosDriver != null)
            menu.addRunnable("More::Show SparkFun OTOS version", this::otosVersionDialog);
        if (drive.pinpointDriver != null)
            menu.addRunnable("More::Show goBilda Pinpoint version", this::pinpointVersionDialog);

        // Set the initial enable/disable status for the core widgets:
        updateTunerDependencies(Tuner.NONE);

        // Add more options if tuning is complete:
        addUnlockable(() -> pidTuner.tune(PidTunerType.ALL), "More::Interactive PiD tuner (all gains)");
        addUnlockable(this::rotationTest,                    "More::Rotation test (verify trackWidthTicks)");
        addUnlockable(this::splineExample,                   "Examples::Spline");
        addUnlockable(this::lineToTurnExample,               "Examples::LineTo/Turn example");
        addUnlockable(this::lineWithRotationExample,         "Examples::Line with rotation");

        // Run the menu loop:
        while (opModeIsActive()) {
            io.message(menu.update());
        }
    }
}