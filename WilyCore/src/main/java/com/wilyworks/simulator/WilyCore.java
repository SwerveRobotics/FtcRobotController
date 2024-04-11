package com.wilyworks.simulator;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.wilyworks.common.Wily;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.framework.Simulation;
import com.wilyworks.simulator.framework.WilyTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.reflections.Reflections;

import java.awt.BorderLayout;
import java.awt.Choice;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Transparency;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.prefs.Preferences;

import javax.imageio.ImageIO;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

/**
 * Structure for representing the choices of opMode:
 */
class OpModeChoice {
    Class<?> klass; // Class reference
    String givenName; // Name give by user to @teleOp or @autonomous, e.g., "match auton"
    String fullName; // Fully with group and given name, e.g., "tests: match auton"
    String className; // Root of the class name, e.g., "Auton"

    public OpModeChoice(Class<?> klass, String fullName, String givenName, String className) {
        this.klass = klass; this.fullName = fullName; this.givenName = givenName; this.className = className;
    }
}

/**
 * Class for returning all relative found annotated classes:
 */
class Annotations {
    Class<?> configKlass;
    List<OpModeChoice> opModeChoices;
    public Annotations(Class<?> configKlass, List<OpModeChoice> opModeChoices) {
        this.configKlass = configKlass; this.opModeChoices = opModeChoices;
    }
}

/**
 * Class responsible for creation of the main window.
 */
class DashboardWindow extends JFrame {
    final int WINDOW_WIDTH = 500;
    final int WINDOW_HEIGHT = 500;
    DashboardCanvas dashboardCanvas = new DashboardCanvas(WINDOW_WIDTH, WINDOW_HEIGHT);
    String opModeName = "";

    DashboardWindow(List<OpModeChoice> opModeChoices, String[] args) {
        Preferences preferences = Preferences.userRoot().node("com/wilyworks/simulator");
        setTitle("Dashboard");
        setDefaultCloseOperation(DO_NOTHING_ON_CLOSE);
        addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                super.windowClosing(e);
                dispose();
                System.exit(0);
            }
        });

        setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
        setLocation(400, 0);
        setResizable(false);

        BoxLayout layout = new BoxLayout(getContentPane(), BoxLayout.X_AXIS);

        Choice dropDown = new Choice();
        for (OpModeChoice opMode: opModeChoices) {
            dropDown.add(opMode.fullName);
        }
        dropDown.setMaximumSize(new Dimension(400, 100));

        // If an opMode was specified on the command line, look for it in the list of
        // potential choices:
        OpModeChoice autoStart = null;
        if (args.length > 0) {
            String requestedOpMode = args[0].toLowerCase();
            for (OpModeChoice choice: opModeChoices) {
                if ((choice.fullName.toLowerCase().equals(requestedOpMode)) ||
                        (choice.givenName.toLowerCase().equals(requestedOpMode)) ||
                        (choice.className.toLowerCase().equals(requestedOpMode))) {
                    autoStart = choice;
                }
            }
            if (autoStart == null) {
                String message = String.format(
                    "Couldn't find an opMode called '%s'. Are you sure you set the\n" +
                    "configuration's program argument correctly?", args[0]);
                JOptionPane.showMessageDialog(null, message, "Exception",
                        JOptionPane.INFORMATION_MESSAGE);
            }
        }

        // Pre-select the preferred opMode, either from the registry or from the auto-start:
        if (opModeChoices.size() > 0) {
            dropDown.select((autoStart != null)
                    ? autoStart.fullName
                    : preferences.get("opmode", opModeChoices.get(0).fullName));
        }

        JButton button = new JButton("Init");
        button.setMaximumSize(new Dimension(100, 50));
        JLabel label = new JLabel("");

        button.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                switch (WilyCore.status.state) {
                    case STOPPED:
                        // Inform the main thread of the choice and save the preference:
                        OpModeChoice opModeChoice = opModeChoices.get(dropDown.getSelectedIndex());
                        WilyCore.status = new WilyCore.Status(WilyCore.State.INITIALIZED, opModeChoice.klass, button);
                        dropDown.setMaximumSize(new Dimension(0, 0));
                        dropDown.setVisible(false); // Needed for long opMode names, for whatever reason
                        button.setText("Start");

                        opModeName = opModeChoice.fullName;
                        preferences.put("opmode", opModeName);
                        label.setText(opModeName);
                        break;

                    case INITIALIZED:
                        WilyCore.status = new WilyCore.Status(WilyCore.State.STARTED, WilyCore.status.klass, button);
                        button.setText("Stop");
                        break;

                    case STARTED:
                        WilyCore.opModeThread.interrupt();
                        WilyCore.status = new WilyCore.Status(WilyCore.State.STOPPED, null, null);
                        button.setText("Init");
                        dropDown.setMaximumSize(new Dimension(400, 100));
                        dropDown.setVisible(true);
                        label.setText("");
                        break;
                }
            }
        });

        // When auto-starting, press the button twice to jump straight from 'STOPPED' to 'STARTED':
        if (autoStart != null) {
            button.doClick(0);
            button.doClick(0);
        }

        JPanel masterPanel = new JPanel(new BorderLayout());

        JPanel menuPanel = new JPanel();
        menuPanel.setLayout(new BoxLayout(menuPanel, BoxLayout.X_AXIS));
        menuPanel.add(button);
        menuPanel.add(dropDown);
        menuPanel.add(label);
        masterPanel.add(menuPanel, BorderLayout.NORTH);

        JPanel canvasPanel = new JPanel();
        canvasPanel.add(dashboardCanvas);
        masterPanel.add(canvasPanel, BorderLayout.CENTER);

        getContentPane().add(masterPanel);
        pack();

        dashboardCanvas.start();
    }
}

/**
 * Wrapper for the dashboard drawing canvas.
 */
class DashboardCanvas extends java.awt.Canvas {
    BufferStrategy bufferStrat;
    int width;
    int height;

    DashboardCanvas(int width, int height) {
        this.width = width;
        this.height = height;

        setBounds(0, 0, width, height);
        setPreferredSize(new Dimension(width, height));
        setIgnoreRepaint(true);
    }

    void start() {
        createBufferStrategy(2);
        bufferStrat = getBufferStrategy();
        requestFocus();
    }

    @Override
    public Dimension getPreferredSize() {
        return new Dimension(width, height);
    }
}

/**
 * Field view manager.
 */
class Field {
    // Make the field view 720x720 pixels but inset the field surface so that there's padding
    // all around it:
    final int FIELD_VIEW_DIMENSION = 500;
    final int FIELD_SURFACE_DIMENSION = 480;

    // These are derived from the above to describe the field rendering:
    final int FIELD_INSET = (FIELD_VIEW_DIMENSION - FIELD_SURFACE_DIMENSION) / 2;
    final Rectangle FIELD_VIEW = new Rectangle(0, 0, FIELD_VIEW_DIMENSION, FIELD_VIEW_DIMENSION);

    // Robot dimensions:
    final int ROBOT_IMAGE_WIDTH = 128;
    final int ROBOT_IMAGE_HEIGHT = 128;

    Simulation simulation;
    Image backgroundImage;
    BufferedImage robotImage;

    Field(Simulation simulation) {
        this.simulation = simulation;
        ClassLoader classLoader = currentThread().getContextClassLoader();

        InputStream stream = classLoader.getResourceAsStream("background/season-2023-centerstage/field-2023-juice-dark.png");
        if (stream != null) {
            try {
                backgroundImage = ImageIO
                        .read(stream)
                        .getScaledInstance(FIELD_SURFACE_DIMENSION, FIELD_SURFACE_DIMENSION, Image.SCALE_SMOOTH);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        initializeRobotImage();
    }

    // Round to an integer:
    static int round(double value) {
        return (int) Math.round(value);
    }

    // Initialize the robot image bitmap:
    private void initializeRobotImage() {
        final int OPACITY = round(255 * 0.8);
        final double WHEEL_PADDING_X = 0.05;
        final double WHEEL_PADDING_Y = 0.05;
        final double WHEEL_WIDTH = 0.2;
        final double WHEEL_HEIGHT = 0.3;
        final double DIRECTION_LINE_WIDTH = 0.05;
        final double DIRECTION_LINE_HEIGHT = 0.4;

        GraphicsConfiguration config =
                GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration();
        robotImage = config.createCompatibleImage(ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT, Transparency.TRANSLUCENT);

        Graphics2D g = robotImage.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        // Draw the body:
        g.setColor(new Color(0xe5, 0x3e, 0x3d, OPACITY));
        g.fillRect(0, 0, ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT);

        // Draw the wheels:
        g.setColor(new Color(0x74, 0x2a, 0x2a, OPACITY));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH), round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT), round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH),
                round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));

        // Draw the direction indicator:
        g.setColor(new Color(0x74, 0x2a, 0x2a));
        g.fillRect(round(ROBOT_IMAGE_WIDTH / 2.0 - DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH / 2.0), 0,
                round(DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT * DIRECTION_LINE_HEIGHT));
    }

    // Render just the robot:
    void renderRobot(Graphics2D g) {
        AffineTransform imageTransform = new AffineTransform();
        imageTransform.translate(simulation.pose.position.x, simulation.pose.position.y);
        imageTransform.scale(1.0 / ROBOT_IMAGE_WIDTH,1.0 / ROBOT_IMAGE_HEIGHT);
        imageTransform.rotate(simulation.pose.heading.log() + Math.toRadians(90));
        imageTransform.scale(simulation.robotSize.width, simulation.robotSize.height);
        imageTransform.translate(-ROBOT_IMAGE_HEIGHT / 2.0, -ROBOT_IMAGE_HEIGHT / 2.0);
        g.drawImage(robotImage, imageTransform, null);
    }

    // Render the field, the robot, and the field overlay:
    void render(Graphics2D g) {
        // Lay down the background image without needing a transform:
        g.drawImage(backgroundImage, FIELD_VIEW.x + FIELD_INSET, FIELD_VIEW.y + FIELD_INSET, null);

        // Prime the viewport/transform and the clipping for field and overlay rendering:
        AffineTransform oldTransform = g.getTransform();
        g.setClip(FIELD_VIEW.x, FIELD_VIEW.y, FIELD_VIEW.width, FIELD_VIEW.height);
        g.transform(new AffineTransform(
                FIELD_SURFACE_DIMENSION / 144.0, 0,
                0, -FIELD_SURFACE_DIMENSION / 144.0,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET));

        renderRobot(g);
        if (FtcDashboard.fieldOverlay != null)
            FtcDashboard.fieldOverlay.renderAndClear(g);

        // Restore:
        g.setTransform(oldTransform);
    }
}

/**
 * This thread is tasked with regularly updating the Gamepad steate.
 */
class GamepadThread extends Thread {
    Gamepad gamepad1;
    Gamepad gamepad2;

    GamepadThread(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        // TODO: @@@ Need to add gamepad2 support
        setName("Wily gamepad thread");
    }

    @Override
    public void run() {
        while (true) {
            // Update the gamepad state every 10 milliseconds:
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            gamepad1.update();
        }
    }
}

/**
 * Core class for Wily Works. This provides the entry point to the simulator and is the
 * interface with the guest application.
 */
public class WilyCore {
    private static final double DELTA_T = 0.100; // 100ms

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static Simulation simulation;
    public static Field field;
    public static DashboardCanvas dashboardCanvas;
    public static GamepadThread gamepadThread;
    public static OpModeThread opModeThread;
    public static Status status = new Status(State.STOPPED, null, null);

    private static boolean simulationUpdated; // True if WilyCore.update() has been called since
    private static double lastUpdateTime = time(); // Time of last update() call, in seconds

    public static double time() {
        return System.currentTimeMillis() / 1000.0;
    }

    /**
     * Structure to communicate between the UI and the thread running the opMode.
     */
    public enum State { STOPPED, INITIALIZED, STARTED };
    public static class Status {
        public Class<?> klass;
        public State state;
        public JButton stopButton;
        public Status(State state, Class<?> klass, JButton button) {
            this.state = state; this.klass = klass; this.stopButton = button;
        }
    }

    // Render the entire window:
    static public void render() {
        // All Graphics objects can be cast to Graphics2D:
        Graphics2D g = (Graphics2D) dashboardCanvas.getBufferStrategy().getDrawGraphics();

        g.clearRect(0, 0, dashboardCanvas.getWidth(), dashboardCanvas.getHeight());
        field.render(g);
        g.dispose();
        dashboardCanvas.getBufferStrategy().show();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Callbacks provided to the guest. These are all called via reflection from the WilyWorks
    // class.

    // The guest can specify the delta-t (which is handy when single stepping):
    static public void update(double deltaT) {
        // If delta-t is zero then use the real-time clock:
        double time = time();
        if (deltaT <= 0) {
            deltaT = time - lastUpdateTime;
        }
        lastUpdateTime = time;

        // Advance the simulation:
        simulation.advance(deltaT);

        // Render everything:
        render();

        simulationUpdated = true;
    }

    // Guest call to set the pose and velocity:
    static public void setPose(double x, double y, double heading,
                               double xVelocity, double yVelocity, double headingVelocity) {
        // If the user didn't explicitly call the simulation update() API, do it now:
        if (!simulationUpdated)
            update(0);
        simulation.setPose(
                new Pose2d(x, y, heading),
                new PoseVelocity2d(new Vector2d(xVelocity, yVelocity), headingVelocity));
        simulationUpdated = false;
    }

    // Get the simulation's true pose and velocity:
    static public Pose2d getPose() {
        return simulation.pose;
    }
    static public PoseVelocity2d getPoseVelocity() {
        return simulation.poseVelocity;
    }

    // Guest call to set the drive powers:
    static public void setDrivePowers(
            PoseVelocity2d stickVelocity,
            PoseVelocity2d assistVelocity) {

        // If the user didn't explicitly call the simulation update() API, do it now:
        if (!simulationUpdated)
            update(0);

        simulation.setDrivePowers(stickVelocity, assistVelocity);
        simulationUpdated = false;
    }

    // Guest call to get the localized position:
    static public double[] getLocalization() {
        return simulation.localizerUpdate();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Enumerate all useful annotated classes:
    static Annotations getAnnotations() {
        // Use the Reflections library to enumerate all classes in this package that have the
        // @Autonomous and @TeleOp annotations:
        Reflections reflections = new Reflections("org.firstinspires.ftc");
        Set<Class<?>> allOps = new HashSet<>();
        allOps.addAll(reflections.getTypesAnnotatedWith(Autonomous.class));
        allOps.addAll(reflections.getTypesAnnotatedWith(TeleOp.class));
        allOps.addAll(reflections.getTypesAnnotatedWith(Wily.class));
        ArrayList<OpModeChoice> choices = new ArrayList<>();
        Class<?> config = null;
        boolean multipleConfigs = false;

        // Build a list of the eligible opModes along with their friendly names:
        for (Class klass: allOps) {
            if ((WilyWorks.Config.class.isAssignableFrom(klass))) {
                multipleConfigs = (config != null);
                config = klass;
            }

            if ((OpMode.class.isAssignableFrom(klass)) &&
                    (!klass.isAnnotationPresent(Disabled.class))) {

                // getName() returns a fully qualified name ("org.firstinspires.ftc.teamcode.MyOp").
                // Use only the last portion ("MyOp" in this example):
                String className = klass.getName();
                className = className.substring(className.lastIndexOf(".") + 1); // Skip the dot itself
                String givenName = className;
                String fullName = className;

                // Override the name if an annotation exists:
                TeleOp teleOpAnnotation = (TeleOp) klass.getAnnotation(TeleOp.class);
                if (teleOpAnnotation != null) {
                    if (!teleOpAnnotation.name().equals("")) {
                        givenName = teleOpAnnotation.name();
                    }
                    if (!teleOpAnnotation.group().equals("")) {
                        fullName = teleOpAnnotation.group() + ": " + givenName;
                    }
                }
                Autonomous autonomousAnnotation = (Autonomous) klass.getAnnotation(Autonomous.class);
                if (autonomousAnnotation != null) {
                    if (!autonomousAnnotation.name().equals("")) {
                        givenName = autonomousAnnotation.name();
                    }
                    if (!autonomousAnnotation.group().equals("")) {
                        fullName = autonomousAnnotation.group() + ": " + givenName;
                    }
                }
                choices.add(new OpModeChoice(klass, fullName, givenName, className));
            }
        }
        if (multipleConfigs) {
            JOptionPane.showMessageDialog(
                    null,
                    "Only one class should be derived from WilyWorks.Config and annotated with '@Wily'.",
                    "Too many Configs",
                    JOptionPane.INFORMATION_MESSAGE);
        }
        choices.sort(Comparator.comparing(x -> x.fullName));
        return new Annotations(config, choices);
    }

    // Allocate a configuration object. Use the specified class if provided, otherwise use a default.
    static WilyWorks.Config getConfig(Class<?> configKlass) {
        if (configKlass == null) {
            try {
                return (WilyWorks.Config) configKlass.newInstance();
            } catch (InstantiationException|IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        return new WilyWorks.Config();
    }

    // Call from the window manager to invoke the user's chosen "runOpMode" method:
    static void runOpMode(Class<?> klass) {
        OpMode opMode;
        try {
            opMode = (OpMode) klass.newInstance();
        } catch (InstantiationException|IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        opMode.hardwareMap = new HardwareMap();
        opMode.gamepad1 = gamepad1;
        opMode.gamepad2 = gamepad2;
        opMode.telemetry = telemetry;

        if (LinearOpMode.class.isAssignableFrom(klass)) {
            LinearOpMode linearOpMode = (LinearOpMode) opMode;
            try {
                linearOpMode.runOpMode();
            } catch (Exception exception) {
                // There was an exception. Print the stack trace to stdout:
                exception.printStackTrace();

                // Now put the stack trace into a popup:
                StringWriter writer = new StringWriter();
                PrintWriter printWriter = new PrintWriter(writer);
                exception.printStackTrace(printWriter);
                printWriter.flush();
                String stackTrace = writer.toString().replace("\n", "\n    ");
                String message = "Your program hit an unhandled exception:\n\n    " + stackTrace + "\n"
                        + "You can also find this stacktrace in clickable form in the 'Debug/Console' or \n"
                        + "'Run' tabs. Once there click on 'Create breakpoint' and re-run using the debug\n"
                        + "'bug' icon to stop the debugger at exactly the right spot.";

                JOptionPane.showMessageDialog(null, message, "Exception",
                        JOptionPane.INFORMATION_MESSAGE);
            }
        } else {
            // TODO: Implement non-LinearOpMode support
        }
    }

    // Thread dedicated to running the user's opMode:
    static class OpModeThread extends Thread {
        Class<?> opModeClass;
        OpModeThread(Class<?> opModeClass) {
            this.opModeClass = opModeClass;
            setName("Wily OpMode thread");
        }
        @Override
        public void run() {
            WilyCore.runOpMode(status.klass);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // This is the application entry point that starts up all of Wily Works!
    public static void main(String[] args)
    {
        Thread.currentThread().setName("Wily core thread");

        // Enumerate all opModes and find a configuration class:
        Annotations annotations = getAnnotations();
        if (annotations.opModeChoices.size() == 0) {
            String message = "Couldn't find @TeleOp or @Autonomous classes anywhere. Is the SRC_ROOT\n"
                + "environment variable set correctly in the WilyWorks configuration you created?";
            JOptionPane.showMessageDialog(null, message, "Exception",
                    JOptionPane.INFORMATION_MESSAGE);
            System.exit(0); // ====>
        }

        // Start the UI:
        DashboardWindow dashboardWindow = new DashboardWindow(annotations.opModeChoices, args);
        dashboardWindow.setVisible(true);

        dashboardCanvas = dashboardWindow.dashboardCanvas;
        simulation = new Simulation(getConfig(annotations.configKlass));
        field = new Field(simulation);
        gamepad1 = new Gamepad();
        gamepad2 = new Gamepad(); // @@@ Need to hook into
        telemetry = new WilyTelemetry();

        gamepadThread = new GamepadThread(gamepad1, gamepad2);
        gamepadThread.start();

        // Endlessly call opModes:
        while (true) {
            // Wait for the DashboardWindow UI to tell us what opMode to run:
            // TODO: Make this wait on an event.
            while (status.state == State.STOPPED) {
                try {
                    sleep(30);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            // Run the opMode on a dedicated thread so that it can be interrupted as necessary:
            opModeThread = new OpModeThread(status.klass);
            opModeThread.start();
            try {
                // Wait for the opMode thread to complete:
                opModeThread.join();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            // If the thread finished without the user stopping it, switch the mode back to STOPPED:
            while (status.state != State.STOPPED) {
                status.stopButton.doClick(0);
            }
        }
    }
}
