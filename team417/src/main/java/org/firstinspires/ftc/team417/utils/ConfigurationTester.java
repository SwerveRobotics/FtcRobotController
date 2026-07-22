package org.firstinspires.ftc.team417.utils;

import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.regex.Pattern;

// Helpers for using HTML with the FTC Driver Station.
/** @noinspection unused*/
class Html {
    // Showing a less-than or greater-than sign requires special encodings when HTML is enabled:
    public final static String LESS_THAN = "&lt;"; // String to show  a "<"
    public final static String GREATER_THAN = "&gt;"; // String to show a ">"

    // Enable the display on telemetry for HTML.
    public static void initialize(Telemetry telemetry) { initialize(telemetry, false); }
    public static void initialize(Telemetry telemetry, boolean monospace) {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        if (monospace) {
            telemetry.addLine("<tt>");
        }
    }

    // Repeat the string for the specified count.
    private static String repeat(int count, String string) {
        StringBuilder result = new StringBuilder();
        for (int i = 0; i < count; i++) {
            result.append(string);
        }
        return result.toString();
    }

    // Set the foreground font color for a string. Color must be in the format "#00ff00".
    public static String color(String color, String string) {
        return "<font color='" + color + "'>" + string + "</font>";
    }

    // Set the background color for a string. Color must be in the format "#ff0000".
    public static String background(String backgroundColor, String string) {
        return "<span style='background: " + backgroundColor + "'>" + string + "</span>";
    }

    // Set the foreground and background colors for a string. Colors must be in the format "#0000ff".
    public static String colors(String foregroundColor, String backgroundColor, String string) {
        return "<span style='color: " + foregroundColor + "; background: " + backgroundColor + "'>" + string + "</span>";
    }

    // Make a string big according to the specified count: 1.25^count times bigger.
    public static String big(int count, String string) {
        return repeat(count, "<big>") + string + repeat(count, "</big>");
    }

    // Make a string smaller according to the specified count: 0.8^count times smaller.
    public static String small(int count, String string) {
        return repeat(count, "<small>") + string + repeat(count, "</small>");
    }

    // Leading spaces on a line will be trimmed unless this is used:
    public static String spaces(int count) {
        return repeat(count / 4, "&emsp;") + repeat(count % 4, "&nbsp;");
    }

    // One-liners:
    public static String bold(String string) { return "<b>" + string + "</b>"; }
    public static String italic(String string) { return "<i>" + string + "</i>"; }
    public static String monospace(String string) { return "<tt>" + string + "</tt>"; }
    public static String underline(String string) { return "<u>" + string + "</u>"; }
    public static String superscript(String string) { return "<sup>" + string + "</sup>"; }
    public static String subscript(String string) { return "<sub>" + string + "</sub>"; }
    public static String strikethrough(String string) { return "<del>" + string + "</del>"; }
}

@TeleOp(name="Configuration Tester", group="Utility")
@SuppressLint("DefaultLocale")
public class ConfigurationTester extends LinearOpMode {
    double nextAdvanceTime; // Time, relative to time(), at which an auto-repeat happens

    // Show and process input for a menu.
    //
    // header - a text string that is drawn before the menu list.
    // scrollLine - the lowest line in the menu list that's guaranteed to be visible on the DS.
    // options - the menu list.
    // current - the currently highlighted option index.
    // topmost - true if the B button won't cancel; false if the B button can cancel and return -1
    /** @noinspection SameParameterValue*/
    int menu(String header, int scrollLine, List<String> options, int current, boolean topmost) {
        final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
        final double REPEAT_DELAY = 0.15; // Seconds after any repeat to repeat again
        while (isActive()) {
            // If a gamepad1 hasn't been connected before, the user will have to press A
            // while holding start. That can result in phantom A button presses, so account
            // for that here:
            if (gamepad1.start) {
                gamepad1.aWasPressed();
                gamepad1.bWasPressed();
                continue;
            }
            if (header != null) {
                ui.line(header);
            }

            // The list can be so long that it's not all visible on the screen at once, so we
            // scroll the list. Unfortunately, we can't be sure how many lines are visible
            // because some top lines can be reserved by the OS to give error messages, and
            // because the DS can be in either landscape or portrait mode, and we have no good
            // way to determine either. So we just scroll the top of the list, reserving the
            // top line for an arrow-up indicator.
            //
            // This method assumes that menu lines don't exceed the width of the display (which
            // we can't really tell, either), but that should be true almost always here.
            int firstDisplayLine = 0;
            if (current > scrollLine) {
                firstDisplayLine = current - scrollLine + 1;
                ui.line("&nbsp;&nbsp;\u25b2"); // "black up-pointing triangle"
            }
            for (int i = firstDisplayLine; i < options.size(); i++) {
                if (i == current) {
                    ui.line(Html.background(Ui.HIGHLIGHT_COLOR,
                            "\u25c6 " + options.get(i))); // Solid circle
                } else {
                    ui.line("\u25c7 " + options.get(i)); // Hollow circle
                }
            }
            ui.update();
            int advance = 0;
            if (gamepad1.dpadUpWasPressed()) {
                advance = -1;
                nextAdvanceTime = time() + INITIAL_DELAY;
            }
            if (gamepad1.dpadDownWasPressed()) {
                advance = 1;
                nextAdvanceTime = time() + INITIAL_DELAY;
            }
            // Automatically repeat if held long enough:
            if ((gamepad1.dpad_up) && (time() > nextAdvanceTime)) {
                advance = -1;
                nextAdvanceTime = time() + REPEAT_DELAY;
            }
            if ((gamepad1.dpad_down) && (time() > nextAdvanceTime)) {
                advance = 1;
                nextAdvanceTime = time() + REPEAT_DELAY;
            }
            current = Math.max(0, Math.min(options.size() - 1, current + advance));

            if (gamepad1.bWasPressed() && !topmost) // Cancel
                return -1;
            if (gamepad1.aWasPressed()) // Select
                return current;
        }
        return topmost ? 0 : -1;
    }

    // This class is for registering tests.
    static class Test {
        Class<?> klass;
        Consumer<HardwareDevice> test;
        public Test(Class<?> klass, Consumer<HardwareDevice> test) {
            this.klass = klass;
            this.test = test;
        }
    }

    // Give more precision at slow speeds:
    double shapeStick(double input){
        return (Math.pow(input, 3) + input) / 2;
    }

    // This routine inputs a double value from the gamepad's right thumbstick. It scales between
    // the specified minimum and maximum values.
    double previousTime;
    double INPUT_RATE = 1.0/2; // 2 seconds at full speed to span entire range
    double stickValue(double stickInput, double oldValue, double min, double max) {
        double time = time();
        double deltaT = Math.min(time - previousTime, 0.03); // Max delta-t of 30ms
        double deltaValue = (max - min) * INPUT_RATE * deltaT;
        previousTime = time;
        return Math.max(min, Math.min(max, oldValue - shapeStick(stickInput) * deltaValue));
    }

    // Time, in seconds:
    static double time() {
        return nanoTime() * 1e-9;
    }

    // Unusually, we do all our work before Start is pressed, rather than after. That's so that
    // we can support camera previews, which the Driver Station can only do before Start is
    // pressed.
    boolean isActive() {
        return !this.isStopRequested() && !isStarted();
    }

    // Convert a string into a red error message.
    static String error(String string, Object... args) {
        return Html.color("#DC3545", String.format(string, args));
    }

    // Style for displaying gamepad control names.
    static String buttonName(String button) {
        return Html.background("#404040", button);
    }

    String format(String format, Object... args) {
        String text = String.format(format, args);
        // Replace A, B, X Y with the appropriate Xbox or PS4 symbols:
        final Map<String, String> xboxReplacements = Map.ofEntries(
                Map.entry("A", "\ud83c\udd50"), // Round A
                Map.entry("B", "\ud83c\udd51"), // Round B
                Map.entry("X", "\ud83c\udd67"), // Round X
                Map.entry("Y", "\ud83c\udd68")  // Round Y
        );
        final Map<String, String> ps4Replacements = Map.ofEntries(
                Map.entry("A", "\u2715"), // Cross
                Map.entry("B", "\u2b58"), // Circle
                Map.entry("X", "\u2b1C"), // Square
                Map.entry("Y", "\u25b2")  // Triangle
        );
        final Map<String, String> commonReplacements = Map.ofEntries(
                Map.entry("LS", buttonName("LS")),
                Map.entry("RS", buttonName("RS")),
                Map.entry("LT", buttonName("LT")),
                Map.entry("RT", buttonName("RT")),
                Map.entry("LB", buttonName("LB")),
                Map.entry("RB", buttonName("RB")),
                Map.entry("dpad", buttonName("DPAD"))
        );

        Map<String, String> abxyReplacements = xboxReplacements;
        if ((gamepad1.type == Gamepad.Type.SONY_PS4) ||
                (gamepad1.type == Gamepad.Type.SONY_PS4_SUPPORTED_BY_KERNEL)) {
            abxyReplacements = ps4Replacements;
        }
        for (Map.Entry<String, String> entry : abxyReplacements.entrySet()) {
            text = text.replaceAll("\\b" + Pattern.quote(entry.getKey()) + "\\b", entry.getValue());
        }
        for (Map.Entry<String, String> entry : commonReplacements.entrySet()) {
            text = text.replaceAll("\\b" + Pattern.quote(entry.getKey()) + "\\b", entry.getValue());
        }
        return text;
    }

    // Every test has to call this loop. It returns false when the test should terminate.
    boolean prompt() { return prompt(""); }
    boolean prompt(String text) {
        text = format((text.isEmpty() ? "\n" : "\n" + text + ", ") + "B to exit");

        ui.out(text); // Add the prompt text
        ui.update(); // Send all output to the driver station
        if (!isActive() || gamepad1.bWasPressed()) {
            return false;
        }
        String gray = "#808080";
        ui.line(Html.big(2,Html.bold("\"%s\"")), testDescriptor.deviceName);
        ui.line(Html.color(gray, "Description: %s"), testDescriptor.hardwareDevice.getDeviceName());
        ui.line(Html.color(gray, "Connection: %s"), testDescriptor.hardwareDevice.getConnectionInfo());
        ui.line(Html.color(gray, "Loop I/O performance: %s"), loopTimer.get());
        return true;
    }

    // Exclude the following device names from the enumeration because they're for built-in
    // devices that are boring.
    ArrayList<String> EXCLUDE_DEVICE_NAMES = new ArrayList<>(Arrays.asList(
            "Control Hub Portal",
            "Control Hub",
            "Expansion Hub 2"
    ));

    // Helper class for doing formatted output to the Driver Station.
    static class Ui {
        static final String HIGHLIGHT_COLOR = "#9090c0";

        Telemetry telemetry;
        StringBuilder buffer;

        Ui(Telemetry telemetry) {
            this.telemetry = telemetry;
            this.buffer = new StringBuilder();
            // Enable our extensive use of HTML:
            Html.initialize(telemetry);
            // Change the update interval from 250ms to 50ms for a more responsive UI:
            telemetry.setMsTransmissionInterval(50);
        }

        // Output without a newline:
        void out(String format, Object... args) {
            buffer.append(String.format(format, args));
        }
        // Output with a newline:
        void line(String format, Object... args) {
            out(format + "\n", args);
        }
        // Send the output to the Driver Station:
        void update() {
            telemetry.addLine(buffer.toString());
            telemetry.update();
            buffer = new StringBuilder();
        }
    }

    Ui ui; // Class for outputting UI to the Driver Station
    TestDescriptor testDescriptor; // Descriptor of currently executing test
    LoopTimer loopTimer; // Loop timer for the current test

    // Structure for registering tests.
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

    // Class to time loops.
    static class LoopTimer {
        final double DURATION = 1.0; // Update every second
        int loopCount; // Count of loops so far
        double timePerLoop; // Result from the last interval
        double startTime; // Start time of this interval
        LoopTimer() {
            startTime = time();
        }
        public String get() {
            loopCount++;
            if (time() - startTime > DURATION) {
                timePerLoop = (time() - startTime) / loopCount;
                startTime = time();
                loopCount = 0;
            }
            // Return the time in milliseconds:
            return (timePerLoop == 0) ? "-" : String.format("%.1fms", timePerLoop * 1000.0f);
        }
    }

    @Override
    public void runOpMode() {
        ui = new Ui(telemetry);

        // Show a splash screen while we initialize:
        double splashTime = time();
        ui.line(Html.big(5, Html.color(Ui.HIGHLIGHT_COLOR, Html.bold("Configuration Tester!"))));
        ui.line(Html.big(2, "By Swerve Robotics, Woodinville\n"));
        ui.line("Initializing...");
        ui.update();

        // If running Wily Works, register all of the potential classes:
        if (WilyWorks.isSimulating) {
            for (int i = 0; i < TESTS.length; i++) {
                hardwareMap.get(TESTS[i].klass, String.valueOf(i));
            }
        }

        // Query the hardwareMap for all of the registered device, instantiate them, and create
        // corresponding test entries:
        List<TestDescriptor> testList = new LinkedList<>();
        for (String name: hardwareMap.getAllNames(HardwareDevice.class)) {
            // Exclude some (boring built-in) devices based on their names:
            if (!EXCLUDE_DEVICE_NAMES.contains(name)) {
                HardwareDevice device = hardwareMap.get(name);
                // System.out.println(String.format("\"%s\": %s", name, device.getClass().getName()));

                // Find a test for this device type:
                int i;
                for (i = 0; i < TESTS.length; i++) {
                    Test test = TESTS[i];
                    if (test.klass.isAssignableFrom(device.getClass())) {
                        testList.add(new TestDescriptor(name, test.klass.getSimpleName(), device, test.test));
                        break;
                    }
                }
                // If we couldn't find a test that's appropriate for this type, use a generic one:
                if (i == TESTS.length) {
                    testList.add(new TestDescriptor(name, device.getClass().getSimpleName(), device, this::testGeneric));
                }

                // CRServos annoyingly default to a power of -1. Set it to zero here.
                if (device instanceof CRServo) {
                    ((CRServo) device).setPower(0);
                }
            }
        }

        // Major sort on class name, then minor sort on device name:
        testList.sort(Comparator.comparing(TestDescriptor::getClassName).thenComparing(TestDescriptor::getDeviceName));

        // Convert into a menu:
        ArrayList<String> options = new ArrayList<>();
        for (TestDescriptor testDescriptor : testList) {
            options.add(String.format("%s: \"<b>%s</b>\"", testDescriptor.className, testDescriptor.deviceName));
        }

        // Make sure the splash screen has been visible for a couple of seconds:
        while (isActive())
            if (time() - splashTime > 2.0)
                break;

        int selection = 0;
        while (isActive()) {
            String header = format("Here's your entire configuration. dpad to navigate, A to select. Tap %s to quit.",
                    Html.color("#05BD05", "\u25B6"));

            selection = menu(header + "\n", 6, options, selection, true);
            testDescriptor = testList.get(selection);
            loopTimer = new LoopTimer();
            // Invoke the test method:
            testDescriptor.testMethod.accept(testDescriptor.hardwareDevice);
        }
        ui.line("Configuration Tester is done!");
        ui.update();
    }

    // This method is for devices that don't have a specific test.
    void testGeneric(HardwareDevice device) {
        do {
            ui.line("Sorry, no test exists for %s. Please add one!", testDescriptor.className);
        } while (prompt());
    }

    // Test the built-in IMU.
    void testIMU(HardwareDevice device) {
        IMU imu = (IMU) device;
        do {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            ui.line("Yaw: " + Html.big(2, "%.2f\u00b0") +
                    ", Pitch: " + Html.big(2, "%.2f\u00b0") +
                    ", Roll: " + Html.big(2, "%.2f\u00b0"),
                    angles.getYaw(AngleUnit.DEGREES),
                    angles.getPitch(AngleUnit.DEGREES),
                    angles.getRoll(AngleUnit.DEGREES));
        } while (prompt());
    }

    // Test the voltage module.
    void testVoltage(HardwareDevice device) {
        VoltageSensor voltage = (VoltageSensor) device;
        do {
            ui.line(Html.big(3, "Voltage: %.2f"), voltage.getVoltage());
        } while (prompt());
    }

    // Test a motor.
    void testMotor(HardwareDevice device) {
        DcMotorEx motor = (DcMotorEx) device;
        double power = motor.getPower();
        String encoderStatus = "";
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        do {
            ui.line(Html.big(3, "Power: %.2f"), power);
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
                    encoderStatus = error("ERROR: Encoder turns opposite of motor; is motor wiring wrong?");
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

    // Test a Continuous Rotation servo.
    void testCRServo(HardwareDevice device) {
        CRServo crServo = (CRServo) device;
        double power = crServo.getPower();
        do {
            ui.line(Html.big(3, "Power: %.2f"), power);
            power = stickValue(gamepad1.right_stick_y, power, -1, 1);
            if (gamepad1.xWasPressed())
                power = 0;
            crServo.setPower(power);
        } while (prompt("RS for power, X to stop"));
    }

    // Test a servo.
    void testServo(HardwareDevice device) {
        Servo servo = (Servo) device;
        double position = servo.getPosition();
        boolean enabled = false;
        do {
            ui.line(Html.big(3, "Position: %.2f"), position);
            position = stickValue(gamepad1.right_stick_y, position, 0, 1);
            if (enabled)
                servo.setPosition(position);
            else
                ui.line(format("\nA to activate servo. Be prepared for the servo to jump to its position!"));
            if (gamepad1.aWasPressed())
                enabled = true;
        } while (prompt("RS for position"));
    }

    // Test a distance sensor.
    void testDistance(HardwareDevice device) {
        DistanceSensor distance = (DistanceSensor) device;
        do {
            ui.line(Html.big(2, "Distance: %.2fcm"), distance.getDistance(DistanceUnit.CM));
        } while (prompt());
    }

    // Digital channels are configurable for input and output.
    void testDigitalChannel(HardwareDevice device) {
        DigitalChannel channel = (DigitalChannel) device;
        DigitalChannel.Mode mode = channel.getMode();
        boolean outputValue = true;

        String promptText;
        do {
            if (mode == DigitalChannel.Mode.INPUT) {
                ui.line(Html.big(2, "Input: %s"), channel.getState());
                promptText = "Y to switch output mode";
            } else {
                ui.line(Html.big(2, "Output: %s"), outputValue);
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

    // Test webcams.
    void testCamera(HardwareDevice device) {
        WebcamName camera = (WebcamName) device;
        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, aprilTag);
        do {
            ui.out("Press ··· on the Driver Station and then select 'Camera Stream'. ");
            ui.line("That will show a snapshot from the camera. Tap the DS screen to update.\n");
            ui.line("<b>IMPORTANT</b>: Select 'Camera Stream' again when you want to return to this app!");
        } while (prompt());
        visionPortal.close();
    }

    // Test the Control Hub's analog input.
    void testAnalogInput(HardwareDevice device) {
        AnalogInput input = (AnalogInput) device;
        do {
            ui.line("Max voltage: %.2f", input.getMaxVoltage());
            ui.line(Html.big(2, "Voltage: %.2f"), input.getVoltage());
        } while (prompt());
    }

    // Test the SparkFun Optical Tracking Odometry Sensor.
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
            ui.line(Html.big(2, "Status: %s"), otos.selfTest() ? "Good" : "Bad");
        } while (prompt());
    }

    // Test the GoBilda Pinpoint odometry computer.
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
                ui.line(error("Turn both pod wheels manually to verify wiring. " +
                        "The encoder values shouldn't stay at zero."));
            }

            int loopTime = pinpoint.getLoopTime();
            double frequency = pinpoint.getFrequency();
            ui.line("Loop time: %d, frequency: %.1f", loopTime, frequency);

            // The GoBilda driver code says to contact tech support if the following
            // conditions are consistently seen:
            if ((loopTime < 500) || (loopTime > 1100)) {
                ui.line(error("Bad loop time, contact tech@gobilda.com"));
            }
            if ((frequency < 900) || (frequency > 2000)) {
                ui.line(error("Bad frequency, contact tech@gobilda.com"));
            }

            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY)
                ui.line(Html.big(0, "Reported status: Good"));
            else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ)
                ui.line(Html.big(0, "Reported status: Ok") + "(bad read)");
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
                ui.line(error("Status error: " + error));
            }
        } while (prompt());
    }

    // Test a color sensor.
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
            String color = String.format("#%06x", rgba.toColor() & 0xffffff); // Color in hex
            ui.line("Color: %s", Html.big(3, Html.color(color, "\u25a0"))); // Box
            ui.line("Normalized ARGB: (%.2f, %.2f, %.2f)", rgba.red, rgba.green, rgba.blue);
            ui.line("HSV: (%.2f, %.2f, %.2f)", hsv[0], hsv[1], hsv[2]);

            if (sensor instanceof DistanceSensor) {
                ui.line("Distance: %.2f\"", ((DistanceSensor) sensor).getDistance(DistanceUnit.INCH));
            }
        } while (prompt("RS to adjust gain"));
    }

    // Test an LED.
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

    // Test a Limelight 3A.
    void testLimelight(HardwareDevice device) {
        Limelight3A limelight = (Limelight3A) device;
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        do {
            limelight.getLatestResult();

            LLStatus status = limelight.getStatus();
            ui.line("Connected: %s", limelight.isConnected());
            ui.line("Name: %s", status.getName());
            ui.line("Temperature: %.1f°", status.getTemp());
            sleep(10);
        } while (prompt());
    }

    // Register your test here. Note that the order can be important if a device supports
    // multiple device objects (e.g., many color sensors support both NormalizedColorSensor
    // and DistanceSensor).
    final Test[] TESTS = {
            new Test(IMU.class, this::testIMU),
            new Test(VoltageSensor.class, this::testVoltage),
            new Test(CRServo.class, this::testCRServo),
            new Test(Servo.class, this::testServo),
            new Test(DcMotor.class, this::testMotor),
            new Test(NormalizedColorSensor.class, this::testNormalizedColorSensor),
            new Test(DistanceSensor.class, this::testDistance),
            new Test(DigitalChannel.class, this::testDigitalChannel),
            new Test(WebcamName.class, this::testCamera),
            new Test(AnalogInput.class, this::testAnalogInput),
            new Test(SparkFunOTOS.class, this::testOtos),
            new Test(GoBildaPinpointDriver.class, this::testPinpoint),
            new Test(LED.class, this::testLED),
            new Test(Limelight3A.class, this::testLimelight),
    };
}
