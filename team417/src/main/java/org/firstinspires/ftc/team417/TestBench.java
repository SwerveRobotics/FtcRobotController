/// Test Bench for First Tech Challenge.
///
/// Copyright James Goossen.
package org.firstinspires.ftc.team417;

import static java.lang.System.nanoTime;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.regex.Pattern;

/// Helper class for doing UI to the Driver Station.
@SuppressWarnings("unused") class Ui {
    static final String HIGHLIGHT_COLOR = "#9090c0";

    Telemetry telemetry;
    StringBuilder buffer;
    Gamepad gamepad;

    public Ui(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.buffer = new StringBuilder();
        Html.initialize(telemetry);
        // Change the update interval from 250ms to 50ms for a more responsive UI:
        telemetry.setMsTransmissionInterval(50);
    }

    /// Time, in seconds.
    public static double time() {
        return nanoTime() * 1e-9;
    }
    /// Output without a newline:
    public void out(String format, Object... args) {
        buffer.append(String.format(Locale.ROOT, format, args));
    }
    /// Output with a newline:
    public void line(String format, Object... args) {
        out(format + "\n", args);
    }
    /// Clear the output buffer.
    public void clear() {
        buffer = new StringBuilder();
    }
    /// Send the accumulated output to the Driver Station:
    public void send() {
        telemetry.addLine(buffer.toString());
        telemetry.update();
        buffer = new StringBuilder();
    }

    /// Convert a string into a red error message.
    public String error(String string, Object... args) {
        return Html.color("#DC3545", String.format(Locale.ROOT, string, args));
    }

    /// Style a string for displaying gamepad control names.
    public String buttonName(String button) {
        return Html.background("#404040", button);
    }

    /// Use a type of markdown to describe gamepad buttons for nicer presentation. A string of
    /// "Press Y" will get transformed into "Press 🅨" if using an Xbox gamepad, and "Press ▲"
    /// if using a Playstation controller, for example.
    public String markdown(String format, Object... args) {
        String text = String.format(Locale.ROOT, format, args);
        final Map<String, String> xboxReplacements = Map.ofEntries(
                Map.entry("A", "\uD83C\uDD50"), // Round A 🅐
                Map.entry("B", "\ud83c\udd51"), // Round B 🅑
                Map.entry("X", "\ud83c\udd67"), // Round X 🅧
                Map.entry("Y", "\ud83c\udd68")  // Round Y 🅨
        );
        final Map<String, String> ps4Replacements = Map.ofEntries(
                Map.entry("A", "✕"), // Cross
                Map.entry("B", "⭘"), // Circle
                Map.entry("X", "■"), // Square
                Map.entry("Y", "▲")  // Triangle
        );
        final Map<String, String> commonReplacements = Map.ofEntries(
                Map.entry("LS", buttonName("LS")),
                Map.entry("RS", buttonName("RS")),
                Map.entry("LT", buttonName("LT")),
                Map.entry("RT", buttonName("RT")),
                Map.entry("LB", buttonName("LB")),
                Map.entry("RB", buttonName("RB")),
                Map.entry("dpad", buttonName("DPAD")),
                Map.entry("PLAY", Html.color("#05BD05", "▶")), // Start on the RC
                Map.entry("STOP", Html.color("#C94F4F", "■")) // Stop on the RC
        );

        Map<String, String> abxyReplacements = xboxReplacements;
        if ((gamepad.type == Gamepad.Type.SONY_PS4) ||
                (gamepad.type == Gamepad.Type.SONY_PS4_SUPPORTED_BY_KERNEL)) {
            abxyReplacements = ps4Replacements;
        }
        // Replace whole words. Note that \b denotes a word boundary in regex:
        for (Map.Entry<String, String> entry : abxyReplacements.entrySet()) {
            text = text.replaceAll("\\b" + Pattern.quote(entry.getKey()) + "\\b", entry.getValue());
        }
        for (Map.Entry<String, String> entry : commonReplacements.entrySet()) {
            text = text.replaceAll("\\b" + Pattern.quote(entry.getKey()) + "\\b", entry.getValue());
        }
        return text;
    }

    /// Structure for registering menu options.
    private static class MenuOption {
        String description; // What to show for this menu item
        Runnable runnable; // What to run when this menu item is selected
        public MenuOption(String description, Runnable runnable) {
            this.description = description;
            this.runnable = runnable;
        }
    }

    /// Class to handle a scrollable list of options.
    public class Menu {
        static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
        static final double REPEAT_DELAY = 0.15; // Seconds after any repeat to repeat again

        ArrayList<MenuOption> options = new ArrayList<>(); // The list of options (can be empty)
        int current; // The currently highlighted option index
        int scrollLine; // The lowest line in the menu list that's guaranteed to be visible on the DS
        double nextAdvanceTime; // Time, relative to time(), at which an auto-repeat happens

        /// Show and process input for a menu.
        ///
        /// @param scrollLine The lowest line in the menu list that's guaranteed to be visible on
        /// the Driver Station.
        public Menu(int scrollLine) {
            this.scrollLine = scrollLine;
        }
        public Menu() { this(8); }

        /// Register an option for the menu. When the option is selected, the runnable will be run.
        public void add(String description, Runnable runnable) {
            options.add(new MenuOption(description, runnable));
        }

        /// Call this in a loop to process menu input and draw the menu via telemetry. Uses DPAD
        /// up and down to scroll and A to select. The caller is responsible for subsequently
        /// calling [#send()].
        public void update() {
            // The list can be so long that it's not all visible on the screen at once, so we
            // scroll the list. Unfortunately, we can't be sure how many lines are visible
            // because some top lines can be reserved by the OS to give error messages, and
            // because the DS can be in either landscape or portrait mode, and we have no good
            // way to determine either. So we just scroll the top of the list, reserving the
            // top line for an arrow-up indicator.
            //
            // This method assumes that menu lines don't exceed the width of the display (which
            // we can't really tell, either), but we try to construct our options to fit on one line.
            int firstDisplayLine = 0;
            if (current > scrollLine) {
                firstDisplayLine = current - scrollLine + 1;
                line("&nbsp;&nbsp;▲"); // "black up-pointing triangle"
            }
            for (int i = firstDisplayLine; i < options.size(); i++) {
                if (i == current) {
                    line(Html.background(HIGHLIGHT_COLOR,
                            "◆ " + options.get(i).description)); // Solid diamond
                } else {
                    line("◇ " + options.get(i).description); // Hollow diamond
                }
            }
            int advance = 0;
            if (gamepad.dpadUpWasPressed()) {
                advance = -1;
                nextAdvanceTime = time() + INITIAL_DELAY;
            }
            if (gamepad.dpadDownWasPressed()) {
                advance = 1;
                nextAdvanceTime = time() + INITIAL_DELAY;
            }
            // Automatically repeat if held long enough:
            if ((gamepad.dpad_up) && (time() > nextAdvanceTime)) {
                advance = -1;
                nextAdvanceTime = time() + REPEAT_DELAY;
            }
            if ((gamepad.dpad_down) && (time() > nextAdvanceTime)) {
                advance = 1;
                nextAdvanceTime = time() + REPEAT_DELAY;
            }
            current = Math.max(0, Math.min(options.size() - 1, current + advance));

            // If A is pressed, run the selected option:
            if (gamepad.aWasPressed()) {
                clear();
                options.get(current).runnable.run();
            }
        }
    }

    /// This class contains helpers for using HTML with FTC Driver Station telemetry.
    public static class Html {
        /// Showing a less-than or greater-than sign requires special encodings when HTML is enabled:
        public final static String LESS_THAN = "&lt;"; // String to show  a "<"
        public final static String GREATER_THAN = "&gt;"; // String to show a ">"

        /// Enable the telemetry display for HTML and optionally monospace.
        public static void initialize(Telemetry telemetry) { initialize(telemetry, false); }
        public static void initialize(Telemetry telemetry, boolean monospace) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            if (monospace) {
                telemetry.addLine("<tt>");
            }
        }

        /// Repeat the string for the specified count.
        private static String repeat(int count, String string) {
            StringBuilder result = new StringBuilder();
            for (int i = 0; i < count; i++) {
                result.append(string);
            }
            return result.toString();
        }

        /// Set the foreground font color for a string. Color must be in the format "#00ff00".
        public static String color(String color, String string) {
            return "<font color='" + color + "'>" + string + "</font>";
        }

        /// Set the background color for a string. Color must be in the format "#ff0000".
        public static String background(String backgroundColor, String string) {
            return "<span style='background: " + backgroundColor + "'>" + string + "</span>";
        }

        /// Set the foreground and background colors for a string. Colors must be in the format "#0000ff".
        public static String colors(String foregroundColor, String backgroundColor, String string) {
            return "<span style='color: " + foregroundColor + "; background: " + backgroundColor + "'>" + string + "</span>";
        }

        /// Make a string big according to the specified factor: 1.25^factor times bigger.
        public static String big(int factor, String string) {
            return repeat(factor, "<big>") + string + repeat(factor, "</big>");
        }

        /// Make a string smaller according to the specified factor: 0.8^factor times smaller.
        public static String small(int factor, String string) {
            return repeat(factor, "<small>") + string + repeat(factor, "</small>");
        }

        /// Leading spaces on a line will be trimmed unless this is used:
        public static String spaces(int count) {
            return repeat(count / 4, "&emsp;") + repeat(count % 4, "&nbsp;");
        }

        /// One-liners:
        public static String bold(String string) { return "<b>" + string + "</b>"; }
        public static String italic(String string) { return "<i>" + string + "</i>"; }
        public static String monospace(String string) { return "<tt>" + string + "</tt>"; }
        public static String underline(String string) { return "<u>" + string + "</u>"; }
        public static String superscript(String string) { return "<sup>" + string + "</sup>"; }
        public static String subscript(String string) { return "<sub>" + string + "</sub>"; }
        public static String strikethrough(String string) { return "<del>" + string + "</del>"; }
    }
}

/// OpMode class for running tests. Add @Disabled to remove it from the DS's opMode list.
@TeleOp(name="Test Bench", group="Utility")
public class TestBench extends LinearOpMode {
    boolean isCameraTestMode; // True if testing cameras before Start; false if testing other devices
    Ui ui; // Class for outputting UI to the Driver Station
    TestDescriptor testDescriptor; // Descriptor of currently executing test

    /// Structure for registering tests.
    static class Test {
        Class<?> klass;
        Consumer<HardwareDevice> test;
        public Test(Class<?> klass, Consumer<HardwareDevice> test) {
            this.klass = klass;
            this.test = test;
        }
    }

    /// Structure for describing tests.
    static class TestDescriptor {
        String deviceName; // User's name for the device
        String className; // Friendly name of the device object's class
        HardwareDevice hardwareDevice; // The device object
        Consumer<HardwareDevice> testMethod; // The method that does the test
        public TestDescriptor(String deviceName, String className, HardwareDevice hardwareDevice, Consumer<HardwareDevice> testMethod) {
            this.deviceName = deviceName;
            this.className = className;
            this.hardwareDevice = hardwareDevice;
            this.testMethod = testMethod;
        }
        String getClassName() { return className; }
        String getDeviceName() { return deviceName; }
    }

    /// Give more precision at slow stick input speeds.
    double shapeStick(double input){
        return (Math.pow(input, 3) + input) / 2;
    }

    /// Input a double value from the gamepad's right thumbstick. It scales between the specified
    /// minimum and maximum values.
    double previousTime;
    double INPUT_RATE = 1.0/2; // 2 seconds at full speed to span entire range
    double stickValue(double stickInput, double oldValue, double min, double max) {
        double time = Ui.time();
        double deltaT = Math.min(time - previousTime, 0.03); // Max delta-t of 30ms
        double deltaValue = (max - min) * INPUT_RATE * deltaT;
        previousTime = time;
        return Math.max(min, Math.min(max, oldValue - shapeStick(stickInput) * deltaValue));
    }

    /// The camera test mode is active until Start is pressed; the mode for all other devices
    /// is active once Start is pressed and goes until Stop is pressed.
    boolean isModeActive() {
        // Camera tests end when Start is pressed; other tests end when Stop is pressed:
        return (isCameraTestMode) ? !isStarted() : !isStopRequested();
    }

    /// Every test has to call this in their loop. It allows the user to terminate the test by
    /// pressing the 'B' button. It returns false when the test should terminate. `markdownPrompt'
    /// is an optional prompt text that will have ", B to exit" appended to it. If no string is
    /// given, a default of "\nB to exit" will be used.
    boolean prompt() { return prompt(""); }
    boolean prompt(String markdownPrompt) {
        String text = ui.markdown((markdownPrompt.isEmpty() ? "\n" : "\n" + markdownPrompt + ", ") + "B to exit");

        ui.out(text); // Add the prompt text
        ui.send(); // Send all output to the driver station
        if (!isModeActive() || gamepad1.bWasPressed()) {
            return false;
        }
        String gray = "#808080";
        ui.line(Ui.Html.big(2, Ui.Html.bold("\"%s\"")), testDescriptor.deviceName);
        ui.line(Ui.Html.color(gray, "Description: %s"), testDescriptor.hardwareDevice.getDeviceName());
        ui.line(Ui.Html.color(gray, "Connection: %s"), testDescriptor.hardwareDevice.getConnectionInfo());

        return true;
    }

    /// Build and run the menu from the list of test descriptors.
    void deviceMenu(List<TestDescriptor> testList, String header) {
        // Build the menu:
        Ui.Menu menu = ui.new Menu();
        for (TestDescriptor descriptor : testList) {
            String text = String.format(Locale.ROOT, "%s: \"<b>%s</b>\"", descriptor.className, descriptor.deviceName);
            menu.add(text, () -> {
                testDescriptor = descriptor; // Set so that test has access to the descriptor
                descriptor.testMethod.accept(descriptor.hardwareDevice); // Run the test when selected
            });
        }

        // Run the menu loop:
        while (isModeActive()) {
            ui.line(header + "\n");
            menu.update();
            ui.send();
            sleep(25);
        }
    }

    /// Entry point for our opMode.
    @Override public void runOpMode() {
        ui = new Ui(telemetry, gamepad1);

        // Banner to show while initializing:
        String banner = Ui.Html.big(5, Ui.Html.color(Ui.HIGHLIGHT_COLOR,
                Ui.Html.bold("Test Bench!\n"))) + Ui.Html.big(2, "By Loonybot\n\n");

        // Initializing *all* hardware devices in the configuration can take a long time,
        // especially when Sidekick is running. Inform the user as we pre-initialize every device
        // in our test list. Devices outside our list will be initialized later when we enumerate
        // all devices via getAll(HardwareDevice.class).
        for (Test test: TESTS) {
            ui.line(banner + "Initializing %s...", test.klass.getSimpleName());
            ui.send();
            hardwareMap.tryGet(test.klass, test.klass.getSimpleName()); // Populate emulator
            hardwareMap.getAll(test.klass); // Initialize the hardware
        }

        final HashSet<String> ignoredClassNames = new HashSet<>(Arrays.asList(
                "LynxAnalogInputController",
                "LynxDcMotorController",
                "LynxDigitalChannelController",
                "LynxServoController",
                "LynxUsbDeviceDelegate"
        ));

        ui.line(banner + "Initializing everything else...");
        ui.send();

        // Query the hardwareMap for all registered devices, instantiate them, and create
        // corresponding test entries:
        List<TestDescriptor> testList = new LinkedList<>();
        for (HardwareDevice device: hardwareMap.getAll(HardwareDevice.class)) {
            String classSimpleName = device.getClass().getSimpleName();
            // Truncate the name at "$" to remove decorators:
            if (classSimpleName.contains("$")) {
                classSimpleName = classSimpleName.substring(0, classSimpleName.indexOf("$"));
            }
            if (ignoredClassNames.contains(classSimpleName))
                continue; // ====> Ignore this class

            // Determine the configuration name. getNamesOf() can return 0 or more than one name:
            Set<String> names = hardwareMap.getNamesOf(device);
            StringBuilder name = new StringBuilder();
            for (String string: names) {
                if (name.length() > 0)
                    name.append(", ");
                name.append(string);
            }
            if (name.length() == 0) {
                name = new StringBuilder("???");
            }

            // Find and add a test for this device type, with the default of a generic test:
            TestDescriptor testDescriptor = new TestDescriptor(name.toString(), classSimpleName,
                    device, this::testGeneric);
            for (Test test: TESTS) {
                if (test.klass.isAssignableFrom(device.getClass())) {
                    testDescriptor = new TestDescriptor(name.toString(), test.klass.getSimpleName(),
                            device, test.test);
                }
            }
            testList.add(testDescriptor);

            // CRServos annoyingly default to a power of -1. Set it to zero here.
            if (device instanceof CRServo) {
                ((CRServo) device).setPower(0);
            }
        }

        // Major sort on class name, then minor sort on device name:
        testList.sort(Comparator.comparing(TestDescriptor::getClassName).thenComparing(TestDescriptor::getDeviceName));

        // Split the list into two: one list has cameras, the other list everything else:
        List<TestDescriptor> cameraList = new LinkedList<>();
        List<TestDescriptor> otherList = new LinkedList<>();
        for (TestDescriptor testDescriptor : testList) {
            if (testDescriptor.className.contains("WebcamName")) {
                cameraList.add(testDescriptor);
            } else {
                otherList.add(testDescriptor);
            }
        }

        if (cameraList.isEmpty()) {
            ui.line(ui.markdown("Tap PLAY on the left to begin."));
            ui.send();
            waitForStart();
        } else {
            isCameraTestMode = true;
            String header = ui.markdown("Before pressing PLAY, test your webcam. " +
                    "Press A to select your camera, dpad to navigate.\n\n" +
                    "Tap PLAY on the left when ready to test other devices.");
            deviceMenu(cameraList, header);
        }

        isCameraTestMode = false;
        String header = ui.markdown("dpad to navigate, A to select. Tap STOP on the left to quit.");
        deviceMenu(otherList, header);

        ui.line("Test Bench is done!");
        ui.send();

        // If using Sidekick, let it know that we're done so that its thread view is accurate.
        // We do this via reflection so that it won't crash for people who don't use Sidekick.
        try {
            Class<?> skClass = Class.forName("com.loonybot.sidekick.Sk");
            skClass.getMethod("endOpMode").invoke(null);
        } catch (Exception ignored) {}
    }

    /// This method is for devices that don't have a specific test written for them.
    void testGeneric(HardwareDevice device) {
        do {
            ui.line("Sorry, no test exists for %s. Please add one!", testDescriptor.className);
        } while (prompt());
    }

    /// Test the built-in IMU.
    void testIMU(HardwareDevice device) {
        IMU imu = (IMU) device;
        do {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            ui.line("Yaw: " + Ui.Html.big(2, "%.2f\u00b0") +
                            ", Pitch: " + Ui.Html.big(2, "%.2f\u00b0") +
                            ", Roll: " + Ui.Html.big(2, "%.2f\u00b0"),
                    angles.getYaw(AngleUnit.DEGREES),
                    angles.getPitch(AngleUnit.DEGREES),
                    angles.getRoll(AngleUnit.DEGREES));
        } while (prompt());
    }

    /// Test the voltage module.
    void testVoltage(HardwareDevice device) {
        VoltageSensor voltage = (VoltageSensor) device;
        do {
            ui.line(Ui.Html.big(3, "Voltage: %.2f"), voltage.getVoltage());
        } while (prompt());
    }

    /// Test the Lynx module.
    void testLynxModule(HardwareDevice device) {
        LynxModule module = (LynxModule) device;
        do {
            ui.line("Current: %.2f mA", module.getCurrent(CurrentUnit.MILLIAMPS));
            ui.line("GPIO bus current: %.2f mA", module.getGpioBusCurrent(CurrentUnit.MILLIAMPS));
            ui.line("I2C bus current: %.2f mA", module.getI2cBusCurrent(CurrentUnit.MILLIAMPS));
            ui.line("Input (battery) voltage: %.2f V", module.getInputVoltage(VoltageUnit.VOLTS));
            ui.line("Auxiliary (5V) voltage: %.2f V", module.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            ui.line("Module temperature: %.1f F", module.getTemperature(TempUnit.FARENHEIT));
        } while (prompt());
    }

    /// Test a motor.
    void testMotor(HardwareDevice device) {
        DcMotorEx motor = (DcMotorEx) device;
        double power = motor.getPower();
        String encoderStatus = "";
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        do {
            ui.line(Ui.Html.big(3, "Power: %.2f"), power);
            power = stickValue(gamepad1.right_stick_y, power, -1, 1);
            if (gamepad1.xWasPressed())
                power = 0;
            motor.setPower(power);

            int currentTicks = motor.getCurrentPosition();
            double velocity = motor.getVelocity();
            if (power != 0) {
                if ((currentTicks == 0) && (velocity == 0)) {
                    encoderStatus = "No encoder detected.";
                } else if (((velocity < 0) && (power > 0)) || ((velocity > 0) && (power < 0))) {
                    encoderStatus = ui.error("ERROR: Encoder turns opposite of motor; is motor wiring wrong?");
                } else {
                    encoderStatus = "Encoder detected.";
                }
            }
            if (!encoderStatus.isEmpty()) {
                ui.line(encoderStatus);
                if ((currentTicks != 0) || (velocity != 0)) {
                    ui.line("Position: %d", currentTicks);
                    ui.line("Velocity: %.0f", velocity);
                }
            }
        } while (prompt("RS for power, X to stop"));
    }

    /// Test a Continuous Rotation servo.
    void testCRServo(HardwareDevice device) {
        CRServo crServo = (CRServo) device;
        double power = crServo.getPower();
        do {
            ui.line(Ui.Html.big(3, "Power: %.2f"), power);
            power = stickValue(gamepad1.right_stick_y, power, -1, 1);
            if (gamepad1.xWasPressed())
                power = 0;
            crServo.setPower(power);
        } while (prompt("RS for power, X to stop"));
    }

    /// Test a servo.
    void testServo(HardwareDevice device) {
        Servo servo = (Servo) device;
        double position = servo.getPosition();
        boolean enabled = false;
        do {
            ui.line(Ui.Html.big(3, "Position: %.2f"), position);
            position = stickValue(gamepad1.right_stick_y, position, 0, 1);
            if (enabled)
                servo.setPosition(position);
            else
                ui.line(ui.markdown("\nA to activate servo. Be prepared for the servo to jump to its position!"));
            if (gamepad1.aWasPressed())
                enabled = true;
        } while (prompt("RS for position"));
    }

    /// Test a distance sensor.
    void testDistance(HardwareDevice device) {
        DistanceSensor distance = (DistanceSensor) device;
        do {
            ui.line(Ui.Html.big(2, "Distance: %.2fcm"), distance.getDistance(DistanceUnit.CM));
        } while (prompt());
    }

    /// Digital channels are configurable for input and output.
    void testDigitalChannel(HardwareDevice device) {
        DigitalChannel channel = (DigitalChannel) device;
        DigitalChannel.Mode mode = channel.getMode();
        boolean outputValue = true;

        String promptText;
        do {
            if (mode == DigitalChannel.Mode.INPUT) {
                ui.line(Ui.Html.big(2, "Input: %s"), channel.getState());
                promptText = "Y to switch output mode";
            } else {
                ui.line(Ui.Html.big(2, "Output: %s"), outputValue);
                promptText = "A to toggle value, Y to switch output mode";
                if (gamepad1.aWasPressed())
                    outputValue = !outputValue;
            }
            if (gamepad1.yWasPressed()) {
                mode = (mode == DigitalChannel.Mode.INPUT)
                        ? DigitalChannel.Mode.OUTPUT
                        : DigitalChannel.Mode.INPUT;
                channel.setMode(mode);
            }
        } while (prompt(promptText));
    }

    /// Test cameras.
    void testCamera(HardwareDevice device) {
        WebcamName camera = (WebcamName) device;
        // Once the Vision Portal is created, the user can test the camera using the Driver Station.
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera);
        do {
            if (camera.isWebcam() && !camera.isAttached()) {
                ui.line(ui.error("\nERROR: The camera is not properly attached!"));
            } else {
                ui.out(ui.markdown("• Press A to see what resolutions the camera supports. This will crash the "));
                ui.out("app but the error message will list the supported resolutions.\n\n");
                ui.out("• Press %s in the upper right of this screen and select 'Camera Stream' ",
                        Ui.Html.big(1, "<b>⋮</b>"));
                ui.out("to see a camera view. Tap the screen to update. ");
                ui.out("<b>IMPORTANT</b>: When done, select 'Camera Stream' again to return here.\n");
                if (gamepad1.aWasPressed()) {
                    visionPortal.close();
                    // In a bit of questionable API design, the only way to determine the camera's
                    // supported resolutions is to build a vision portal with a bogus resolution,
                    // resulting in an exception whose message describes the supported resolutions.
                    // There is no way to catch the exception because it occurs asynchronously on a
                    // different thread, so doing this will crash our app.
                    visionPortal = new VisionPortal.Builder()
                            .setCamera(camera)
                            .setCameraResolution(new Size(217, 314))
                            .build();
                }
            }
        } while (prompt());
        visionPortal.close();
    }

    /// Test the Control Hub's analog input.
    void testAnalogInput(HardwareDevice device) {
        AnalogInput input = (AnalogInput) device;
        do {
            ui.line("Max voltage: %.2f", input.getMaxVoltage());
            ui.line(Ui.Html.big(2, "Voltage: %.2f"), input.getVoltage());
        } while (prompt());
    }

    /// Test the SparkFun Optical Tracking Odometry Sensor.
    void testOtos(HardwareDevice device) {
        SparkFunOTOS otos = (SparkFunOTOS) device;
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();

        do {
            otos.getVersionInfo(hwVersion, fwVersion);
            ui.line("Hardware version: %d.%d, firmware version: %d.%d",
                    hwVersion.major, hwVersion.minor, fwVersion.major, fwVersion.minor);
            ui.line("Is connected: %s", otos.isConnected());

            SparkFunOTOS.Status status = otos.getStatus();
            ui.line("Tilt angle warning: %s", status.warnTiltAngle);

            SparkFunOTOS.Pose2D pose = otos.getPosition();
            ui.line("x: %.2f\", y: %.2f\", heading: %.2f°", pose.x, pose.y, pose.h);
            ui.line(Ui.Html.big(2, "Status: %s"), otos.selfTest() ? "Good" : "Bad");
        } while (prompt());
    }

    /// Test the GoBilda Pinpoint odometry computer.
    void testPinpoint(HardwareDevice device) {
        GoBildaPinpointDriver pinpoint = (GoBildaPinpointDriver) device;
        int xOr = 0;
        int yOr = 0;
        do {
            pinpoint.update();
            int xEncoder = pinpoint.getEncoderX();
            int yEncoder = pinpoint.getEncoderY();
            xOr |= xEncoder;
            yOr |= yEncoder;

            ui.line("X encoder: %d, Y encoder: %d", xEncoder, yEncoder);
            if (xOr == 0 || yOr == 0) {
                ui.line(ui.error("Turn both pod wheels manually to verify wiring. " +
                        "The encoder values shouldn't stay at zero."));
            }

            int loopTime = pinpoint.getLoopTime();
            double frequency = pinpoint.getFrequency();
            ui.line("Loop time: %d, frequency: %.1f", loopTime, frequency);

            // The GoBilda driver code says to contact tech support if the following
            // conditions are consistently seen:
            if ((loopTime < 500) || (loopTime > 1100)) {
                ui.line(ui.error("Bad loop time, contact tech@gobilda.com"));
            }
            if ((frequency < 900) || (frequency > 2000)) {
                ui.line(ui.error("Bad frequency, contact tech@gobilda.com"));
            }

            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY)
                ui.line(Ui.Html.big(0, "Reported status: Good"));
            else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ)
                ui.line(Ui.Html.big(0, "Reported status: Ok") + "(bad read)");
            else {
                String error = "Unknown error";
                switch (status) {
                    case NOT_READY:
                        error = "Not ready";
                        break;
                    case CALIBRATING:
                        error = "Calibrating";
                        break;
                    case FAULT_X_POD_NOT_DETECTED:
                        error = "X pod not detected";
                        break;
                    case FAULT_Y_POD_NOT_DETECTED:
                        error = "Y pod not detected";
                        break;
                    case FAULT_NO_PODS_DETECTED:
                        error = "No pods detected";
                        break;
                    case FAULT_IMU_RUNAWAY:
                        error = "IMU runaway";
                        break;
                }
                ui.line(ui.error("Status error: " + error));
            }
        } while (prompt());
    }

    /// Test a color sensor.
    void testNormalizedColorSensor(HardwareDevice device) {
        final float[] hsv = new float[3];
        NormalizedColorSensor sensor = (NormalizedColorSensor) device;
        double gain = sensor.getGain();
        do {
            gain = stickValue(gamepad1.right_stick_y, gain, 1, 255);
            ui.line("Gain: %.2f", gain);
            sensor.setGain((float) gain);

            NormalizedRGBA rgba = sensor.getNormalizedColors();
            Color.colorToHSV(rgba.toColor(), hsv);
            String color = String.format(Locale.ROOT, "#%06x", rgba.toColor() & 0xffffff); // Color in hex
            ui.line("Color: %s", Ui.Html.big(3, Ui.Html.color(color, "\u25a0"))); // Box
            ui.line("Normalized ARGB: (%.2f, %.2f, %.2f)", rgba.red, rgba.green, rgba.blue);
            ui.line("HSV: (%.2f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);

            if (sensor instanceof DistanceSensor) {
                ui.line("Distance: %.2f\"", ((DistanceSensor) sensor).getDistance(DistanceUnit.INCH));
            }
        } while (prompt("RS to adjust gain"));
    }

    /// Test an LED.
    void testLED(HardwareDevice device) {
        LED led = (LED) device;
        boolean enable = true;
        do {
            ui.line("Enable: %s", enable);
            if (gamepad1.aWasPressed())
                enable = !enable;
            led.enable(enable);
        } while (prompt("A to toggle enable"));
    }

    /// Test a Limelight 3A.
    void testLimelight(HardwareDevice device) {
        Limelight3A limelight = (Limelight3A) device;
        if (limelight.isConnected()) {
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
            limelight.pipelineSwitch(0); // Switch to pipeline number 0
        }

        do {
            if (!limelight.isConnected()) {
                ui.line(ui.error("\nERROR: The limelight is not properly connected!"));
            } else {
                limelight.getLatestResult();

                LLStatus status = limelight.getStatus();
                ui.line("Connected: %s", limelight.isConnected());
                ui.line("Name: %s", status.getName());
                ui.line("Temperature: %.1f°", status.getTemp());
            }
        } while (prompt());
    }

    /// Register your test here. Note that the order can be important if a device supports
    /// multiple device objects (e.g., many color sensors support both NormalizedColorSensor
    /// and DistanceSensor). The first qualifying test will be used.
    final Test[] TESTS = {
            new Test(WebcamName.class, this::testCamera),
            new Test(Limelight3A.class, this::testLimelight),
            new Test(IMU.class, this::testIMU),
            new Test(VoltageSensor.class, this::testVoltage),
            new Test(LynxModule.class, this::testLynxModule),
            new Test(CRServo.class, this::testCRServo),
            new Test(Servo.class, this::testServo),
            new Test(DcMotor.class, this::testMotor),
            new Test(NormalizedColorSensor.class, this::testNormalizedColorSensor),
            new Test(DistanceSensor.class, this::testDistance),
            new Test(DigitalChannel.class, this::testDigitalChannel),
            new Test(AnalogInput.class, this::testAnalogInput),
            new Test(SparkFunOTOS.class, this::testOtos),
            new Test(GoBildaPinpointDriver.class, this::testPinpoint),
            new Test(LED.class, this::testLED),
    };
}
