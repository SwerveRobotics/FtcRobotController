package com.wilyworks.simulator.framework.mechsim;

import static com.wilyworks.simulator.WilyCore.time;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.framework.WilyLED;
import com.wilyworks.simulator.helpers.Point;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Arc2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

// This is the base class for game simulations for particular years and robots.
public class MechSim {
    // Create the game simulation appropriate to this year.
    static public MechSim create() {
        return new DecodeSlowBotMechSim();
    }

    // Hook the creation of a particular device.
    public HardwareDevice hookDevice(String deviceName, HardwareDevice original) {
        return original;
    }

    // Run the simulation and render the mechanisms.
    public void update(Graphics2D g, Pose2d pose) {
        renderRobotBox(g, pose);
        renderLeds(g, pose);
    }

    // Advance the simulation time.
    public void advance(double deltaT) {}

    // Render the robot box.
    protected void renderRobotBox(Graphics2D g, Pose2d pose) {
        AffineTransform originalTransform = g.getTransform();
        g.translate(pose.position.x, pose.position.y);
        g.rotate(pose.heading.log());
        g.setColor(Color.RED);
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, (float) 0.5));
        // Draw a rectangle for the true position:
        g.draw(new Rectangle2D.Double(
                -WilyCore.config.robotWidth / 2.0, -WilyCore.config.robotLength / 2.0,
                WilyCore.config.robotWidth, WilyCore.config.robotLength));
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, (float) 1.0));
        g.setTransform(originalTransform);
    }

    // Render the LED for REV Digital LEDs.
    protected void renderLeds(Graphics2D g, Pose2d pose) {
        HardwareMap hardwareMap = WilyCore.hardwareMap;
        if (hardwareMap == null)
            return; // Might not have been created yet

        final int[] colors = { 0, 0xff0000, 0x00ff00, 0xffbf00 }; // black, red, green, amber
        final double radius = 2.0; // Circle radius, in inches

        ArrayList<WilyLED> ledArray = new ArrayList<>();
        for (LED led: hardwareMap.led) {
            ledArray.add((WilyLED) led);
        }
        for (int i = 0; i < ledArray.size(); i++) {
            WilyLED led = ledArray.get(i);
            int colorIndex = 0;
            colorIndex |= (led.isRed && led.enable) ? 1 : 0;
            colorIndex |= (!led.isRed && led.enable) ? 2 : 0;

            // The LED actually needs two digital channels to describe all 4 possible colors.
            // Assume that consecutively registered channels make a pair:
            if (i + 1 < ledArray.size()) {
                WilyLED nextLed = ledArray.get(i + 1);
                if ((nextLed.x == led.x) &&
                        (nextLed.y == led.y) &&
                        (nextLed.isRed == !led.isRed)) {

                    colorIndex |= (nextLed.isRed && nextLed.enable) ? 1 : 0;
                    colorIndex |= (!nextLed.isRed && nextLed.enable) ? 2 : 0;
                    i++;
                }
            }

            // Draw the circle at the location of the sensor on the robot, accounting for its
            // current heading:
            Point point = new Point(led.x, led.y)
                    .rotate(pose.heading.log())
                    .add(new Point(pose.position));
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            g.setColor(new Color(0xffffff));
            g.fill(new Ellipse2D.Double(point.x - radius - 0.5, point.y - radius - 0.5,2 * radius + 1, 2 * radius + 1));
            g.setColor(new Color(colors[colorIndex]));
            g.fill(new Ellipse2D.Double(point.x - radius, point.y - radius,2 * radius, 2 * radius));
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_OFF);
        }
    }

}

// Simulation for the SlowBot in the 2025/2026 Decode game.
class DecodeSlowBotMechSim extends MechSim {
    enum BallColor {PURPLE, GREEN, GOLD}

    final double WIDTH = 18; // Robot width
    final double HEIGHT = 18; // Robot height
    final double BALL_SIZE = 5; // 5 inches in diameter
    final double DRUM_SERVO_SPEED = 1.0 / 0.9; // Speed of the drum servo, position/s
    final Point[] INTAKE_OFFSETS = { new Point(8, -1), new Point(8, +1) };
    final double INTAKE_EPSILON = 2.5; // Epsilon for intake distance
    final double INTAKE_POWER = 0.1; // Minimum power for intake
    final double[] INTAKE_POSITIONS = { 0.0/6, 2.0/6, 4.0/6 }; // AKA 'launch' positions
    final double[] TRANSFER_POSITIONS = { 3.0/6, 5.0/6, 1.0/6 }; // Servo positions for intaking
    final double SLOT_EPSILON = 0.02; // Epsilon for determining a slot relative to a [0, 1] range
    final double MIN_TRANSFER_TIME = 0.020; // Second it takes for a transfer
    final double MIN_TRANSFER_POSITION = 0.6; // Minimum position to start a transfer
    final double TRANSFER_SERVO_SPEED = (60.0 / 360) / 0.25; // Speed of a goBilda torque servo, position/s
    final double LAUNCH_SPEED = 144; // Ball launch speed, inches per second
    final Point LAUNCH_OFFSET = new Point(-4, 0);
    final double GOAL_EPSILON = 12; // Distance from goal center to consider a goal
    final Point[] GOAL_CENTERS = { new Point(-72, 72), new Point(-72, -72) };
    final Point[] CLASSIFIER_STARTS = { new Point(-BALL_SIZE/2, 72-BALL_SIZE/2),
        new Point(-BALL_SIZE/2, -72-BALL_SIZE/2) }; // Start locations, corresponding to goals
    final double LAUNCH_ACCELERATION = 1000; // Increase flywheel speed by this many ticks per second
    final double LAUNCH_DROP = 500; // Drop flywheel speed by this many ticks on launch
    final double LAUNCH_EPSILON = 50; // Target and actual flywheel velocities must be within this amount
    final double INTAKE_ERROR_PROBABILITY = 0.2; // Probability that a ball is missed on intake

    // Struct for tracking ball locations:
    static class Ball {
        BallColor color; // GREEN or PURPLE
        Point point; // Current location; not relevant if in robot mechanisms
        Point velocity; // Velocity vector; null for balls lying on the field
        public Ball(BallColor color, double x, double y) {
            this.color = color;
            this.point = new Point(x, y);
        }
    }

    // These preset balls are mirrored in y the constructor:
    Ball[] ballPresets = {
            new Ball(BallColor.GREEN, -12, 53),
            new Ball(BallColor.PURPLE, -12, 48),
            new Ball(BallColor.PURPLE, -12, 43),
            new Ball(BallColor.PURPLE, 12, 53),
            new Ball(BallColor.GREEN, 12, 48),
            new Ball(BallColor.PURPLE, 12, 43),
            new Ball(BallColor.PURPLE, 36, 53),
            new Ball(BallColor.PURPLE, 36, 48),
            new Ball(BallColor.GREEN, 36, 43),
            new Ball(BallColor.PURPLE, 69.5, 69.5),
            new Ball(BallColor.GREEN, 64.5, 69.5),
            new Ball(BallColor.PURPLE, 59.5, 69.5)
    };
    Ball[] ballPreloads = {
            new Ball(BallColor.GOLD, 0, 0),
            new Ball(BallColor.GOLD, 0, 0),
            new Ball(BallColor.GOLD, 0, 0)
    };

    // Hooked devices:
    LaunchMotor upperLaunchMotor;
    LaunchMotor lowerLaunchMotor;
    AnalogInput analogDrum;
    NormalizedColorSensor sensorColor1;
    NormalizedColorSensor sensorColor2;
    DigitalChannel digitalChannel1;
    DigitalChannel digitalChannel2;
    DcMotorEx intakeMotor;
    Servo drumServo;
    Servo transferServo;

    // State:
    double accumulatedDeltaT;
    Ball intakeBall; // Ball in the intake, may be null
    List<Ball> slotBalls = new ArrayList<>(Collections.nCopies(3, null));
    List<Ball> airBalls = new LinkedList<>(); // Balls flying through the air
    List<Ball> fieldBalls = new LinkedList<>(); // Pickable balls on the field
    List<List<Ball>> classifierBalls = IntStream.range(0, 2)
            .mapToObj(i -> new LinkedList<Ball>()).collect(Collectors.toList());
    double actualDrumPosition; // Current location of the drum, [0, 1]
    double actualTransferPosition; // Current transfer servo position, [0, 1]
    double transferStartTime; // Time that a transfer started, zero when not transferring
    int colorSensorMask = -1; // Random 2-bit mask indicating which sensors return true data; -1 if reset
    boolean hasIntaken = false; // True once any ball has been taken into the drum
    boolean hasLaunched = false; // True once any ball has been launched

    // Initialize the beast.
    DecodeSlowBotMechSim() {
        // Add the presets to the field, along with the mirrored-in y versions:
        for (Ball ball: ballPresets) {
            fieldBalls.add(ball);
            fieldBalls.add(new Ball(ball.color, ball.point.x, -ball.point.y));
        }
    }

    // Compute the draw color from the ball object.
    private Color ballColor(Ball ball) {
        if (ball == null)
            return Color.BLACK;
        else if (ball.color == BallColor.PURPLE)
            return new Color(128, 0, 128);
        else if (ball.color == BallColor.GREEN)
            return Color.GREEN;
        else
            return new Color(255, 215, 0); // Gold
    }

    // WilyHardwareMap calls this method when it creates a device, allowing us to substitute
    // with a different device object.
    @Override
    public HardwareDevice hookDevice(String name, HardwareDevice device) {
        // These are input-only devices:
        if (name.equals("motIntake")) {
            intakeMotor = (DcMotorEx) device;
        }
        if (name.equals("servoDrum")) {
            drumServo = (Servo) device;
        }
        if (name.equals("servoTransfer")) {
            transferServo = (Servo) device;
        }

        // There have outputs:
        if (name.equals("motULauncher")) {
            device = upperLaunchMotor = new LaunchMotor();
        }
        if (name.equals("motLLauncher")) {
            device = lowerLaunchMotor = new LaunchMotor();
        }
        if (name.equals("analogDrum")) {
            device = analogDrum = new DrumAnalogInput(this);
        }
        if (name.equals("sensorColor1")) {
            device = sensorColor1 = new DrumColorSensor(this, 0);
        }
        if (name.equals("sensorColor2")) {
            device = sensorColor2 = new DrumColorSensor(this, 1);
        }
        if (name.equals("breakBeam1")) {
            device = digitalChannel1 = new DrumDigitalChannel(this, 1);
        }
        if (name.equals("breakBeam2")) {
            device = digitalChannel2 = new DrumDigitalChannel(this, 2);
        }
        return device;
    }

    // Check to see if the caller created all of the expected state.
    void verifyState() {
        if (upperLaunchMotor == null)
            throw new RuntimeException("Missing upper launch motor");
        if (lowerLaunchMotor == null)
            throw new RuntimeException("Missing lower launch motor");
        if (intakeMotor == null)
            throw new RuntimeException("Missing intake motor");
        if (drumServo == null)
            throw new RuntimeException("Missing drum servo");
        if (transferServo == null)
            throw new RuntimeException("Missing transfer servo");
        if (analogDrum == null)
            throw new RuntimeException("Missing analog drum");
        if (sensorColor1 == null)
            throw new RuntimeException("Missing color sensor 1");
        if (sensorColor2 == null)
            throw new RuntimeException("Missing color sensor 2");
    }

    void render(Graphics2D g, Pose2d pose) {
        // Draw the balls on the field:
        for (Ball ball : fieldBalls) {
            g.setColor(ballColor(ball));
            g.fill(ballEllipse(ball.point.x, ball.point.y));
        }

        // Draw the classifier balls:
        for (int i = 0; i < 2; i++) {
            double x = CLASSIFIER_STARTS[i].x;
            double y = CLASSIFIER_STARTS[i].y;
            int count = 0;
            for (Ball ball: classifierBalls.get(i)) {
                g.setColor(ballColor(ball));
                g.fill(ballEllipse(x, y));
                x -= BALL_SIZE;
                count++;
                if (count >= 9)
                    break; // Classifier maxes out at 9 balls, more causes an overflow
            }
        }

        AffineTransform fieldTransform = g.getTransform();

        // Set the transform to draw the robots in the canonical space, from (-1, -1) to (1, 1):
        g.translate(pose.position.x, pose.position.y);
        g.rotate(pose.heading.log());
        g.scale(WIDTH / 2, HEIGHT / 2);

        // Draw the robot outline:
        g.setStroke(new BasicStroke(0.1f));
        g.setColor(Color.DARK_GRAY);
        g.fill(new Rectangle2D.Double(-1, -1, 2, 2));

        // Draw the intake wheels:
        Color intakeColor = Color.GRAY;
        if (intakeMotor.getPower() > 0) {
            intakeColor = Color.GREEN; // Green  when intaking
        } else if (intakeMotor.getPower() < 0) {
            intakeColor = Color.RED; // Red when spitting out
        }
        g.setColor(intakeColor);
        final int INTAKE_WHEEL_COUNT = 4;
        final double INTAKE_HEIGHT = 0.2;
        final double INTAKE_SPACE = (1.6 - INTAKE_WHEEL_COUNT * INTAKE_HEIGHT) / (INTAKE_WHEEL_COUNT - 1);
        for (int i = 1; i < 3; i++) { // Just draw the middle two wheels
            double y = -0.8 + i * (INTAKE_HEIGHT + INTAKE_SPACE);
            g.fill(new Rectangle2D.Double(0.6, y, 0.4, INTAKE_HEIGHT));
        }

        // Draw the drum:
        g.setColor(Color.GRAY);
        g.translate(-0.33, 0);
        g.rotate(-2 * Math.PI * actualDrumPosition); // Note the negative
        g.fill(new Ellipse2D.Double(-0.8, -0.8, 1.6, 1.6));

        // Draw the dead wedge area:
        g.setColor(Color.DARK_GRAY);
        g.fill(new Arc2D.Double(-0.8, -0.8, 1.6, 1.6, 0.0f, -60, Arc2D.PIE));

        // Draw each of the drum slot circles:
        for (Ball ball: slotBalls) {
            g.setColor(ballColor(ball));
            g.fill(new Ellipse2D.Double(0.1, -0.25, 0.6, 0.6));
            g.rotate(Math.toRadians(120));
        }

        // Draw the air balls:
        g.setTransform(fieldTransform);
        for (Ball ball : airBalls) {
            g.setColor(ballColor(ball));
            g.fill(ballEllipse(ball.point.x, ball.point.y));
        }

        // Draw the intake ball, if there is one:
        if (intakeBall != null) {
            g.translate(pose.position.x, pose.position.y);
            g.rotate(pose.heading.log());
            g.setColor(ballColor(intakeBall));
            g.fill(ballEllipse(7, 0));
        }
    }

    // Shortcut to create an ellipse representing a ball, centered around the specified point.
    Ellipse2D ballEllipse(double x, double y) {
        return new Ellipse2D.Double(x - BALL_SIZE/2, y - BALL_SIZE/2, BALL_SIZE, BALL_SIZE);
    }


    // Find the slot that corresponds to the current drum angle. Returns -1 if not found.
    int findDrumSlot(double[] slotPositions) {
        for (int i = 0; i < 3; i++) {
            if (Math.abs(actualDrumPosition - slotPositions[i]) < SLOT_EPSILON) {
                return i;
            }
        }
        return -1;
    }

    // Advance the simulation:
    void simulate(Pose2d pose, double deltaT) {
        verifyState();

        double heading = pose.heading.log();
        // Advance the balls flying through the air:
        Iterator<Ball> ballIterator = airBalls.iterator();
        while (ballIterator.hasNext()) {
            Ball ball = ballIterator.next();
            // Move the ball:
            ball.point = ball.point.add(ball.velocity.multiply(deltaT));

            // See if it's scored a goal:
            for (int i = 0; i < 2; i++) { // Goal index
                if (Math.hypot(GOAL_CENTERS[i].x - ball.point.x,
                        GOAL_CENTERS[i].y - ball.point.y) < GOAL_EPSILON) {
                    ballIterator.remove();
                    classifierBalls.get(i).add(ball);
                }
            }
        }

        // Ramp up the launcher motors velocities:
        double upperDiff = upperLaunchMotor.targetVelocity - upperLaunchMotor.actualVelocity;
        double lowerDiff = lowerLaunchMotor.targetVelocity - lowerLaunchMotor.actualVelocity;
        upperLaunchMotor.actualVelocity += Math.signum(upperDiff) * Math.min(Math.abs(upperDiff), deltaT * LAUNCH_ACCELERATION);
        lowerLaunchMotor.actualVelocity += Math.signum(lowerDiff) * Math.min(Math.abs(lowerDiff), deltaT * LAUNCH_ACCELERATION);

        // Move the transfer servo towards the target angle:
        double targetTransferPosition = transferServo.getPosition();
        double transferDiff = targetTransferPosition - actualTransferPosition;
        actualTransferPosition += Math.signum(transferDiff) * Math.min(Math.abs(transferDiff), deltaT * TRANSFER_SERVO_SPEED);

        // Move the drum towards the target angle:
        double targetDrumPosition = drumServo.getPosition();
        double drumDiff = targetDrumPosition - actualDrumPosition;
        actualDrumPosition += Math.signum(drumDiff) * Math.min(Math.abs(drumDiff), deltaT * DRUM_SERVO_SPEED);

        // Reset the color sensor mask whenever the drum moves:
        if (targetDrumPosition != actualDrumPosition) {
            colorSensorMask = -1;
        }

        // Find the slot if in position to transfer:
        int transferSlot = findDrumSlot(TRANSFER_POSITIONS);

        // Handle load requests for the launch calibration app, signaled by running the launchers
        // backwards:
        if (upperLaunchMotor.getVelocity() < 0) {
            // If the slot is empty, fill it up with a ball!
            if ((transferSlot != -1) && (slotBalls.get(transferSlot) == null)) {
                slotBalls.set(transferSlot, new Ball(BallColor.PURPLE, 0, 0));
            }
        }

        // Handle transfer requests:
        if (actualTransferPosition <= MIN_TRANSFER_POSITION) {
            if (transferStartTime != 0) {
                throw new RuntimeException("Didn't transfer for sufficient time.");
            }
        } else {
            if (transferSlot == -1) {
                throw new RuntimeException("A transfer is requested when drum isn't in the right spot. That will break things!");
            }
            if (targetDrumPosition != actualDrumPosition) {
                throw new RuntimeException("The drum is moving during a transfer. That will break things!");
            }
            if (upperLaunchMotor.getVelocity() <= 0) {
                throw new RuntimeException("A transfer is requested when upper launcher motor isn't running forward. That won't work!");
            }
            if (lowerLaunchMotor.getVelocity() <= 0) {
                throw new RuntimeException("A transfer is requested when lower launcher motor isn't running forward. That won't work!");
            }
            if (slotBalls.get(transferSlot) == null) {
                if (!hasIntaken && !hasLaunched) {
                    // A transfer has been requested there was no intake done. Assume that this
                    // is Auto with preloads and populate the drum with three golden balls. We
                    // make them gold balls so that we don't have to worry about getting the
                    // ordering correct.
                    slotBalls = new ArrayList<>(Arrays.asList(ballPreloads));
                }
            }
            if (slotBalls.get(transferSlot) != null) {
                if (transferStartTime == 0) {
                    transferStartTime = time();

                    // Check these only at the start of the transfer because we're going to
                    // immediately drop the speeds:
                    if (Math.abs(upperLaunchMotor.targetVelocity - upperLaunchMotor.actualVelocity) > LAUNCH_EPSILON) {
                        throw new RuntimeException("A transfer is requested when upper launcher motor isn't running. That won't work!");
                    }
                    if (Math.abs(lowerLaunchMotor.targetVelocity - lowerLaunchMotor.actualVelocity) > LAUNCH_EPSILON) {
                        throw new RuntimeException("A transfer is requested when lower launcher motor isn't running. That won't work!");
                    }

                    upperLaunchMotor.actualVelocity -= LAUNCH_DROP;
                    lowerLaunchMotor.actualVelocity -= LAUNCH_DROP;
                } else if (time() - transferStartTime > MIN_TRANSFER_TIME) {
                    // Transfer the ball from the drum to the air-balls list:
                    Ball ball = slotBalls.get(transferSlot);
                    ball.velocity = new Point(LAUNCH_SPEED, 0).rotate(heading);
                    ball.point = new Point(pose.position).add(LAUNCH_OFFSET.rotate(heading));
                    airBalls.add(ball);
                    slotBalls.set(transferSlot, null);
                    transferStartTime = 0;
                    hasLaunched = true;
                }
            }
        }

        // Finally, check for intake:
        if (intakeMotor.getPower() > INTAKE_POWER) {
            if (intakeBall == null) {
                // We're intaking from the field into the intake:
                for (Point intakeOffset : INTAKE_OFFSETS) {
                    Point intakePoint = new Point(pose.position).add(intakeOffset.rotate(pose.heading.log()));
                    for (Ball ball : fieldBalls) {
                        double distance = Math.hypot(ball.point.x - intakePoint.x, ball.point.y - intakePoint.y);
                        if (distance < INTAKE_EPSILON) {
                            // Remove the ball from the field. I think this is okay so long as we
                            // terminate the loop...
                            fieldBalls.remove(ball);

                            // Move the ball into an intake position, unless random error says
                            // to toss it:
                            if ((!WilyCore.enableSensorError) || (Math.random() > INTAKE_ERROR_PROBABILITY)) {
                                intakeBall = ball;
                            }
                            break;
                        }
                    }
                }
            } else {
                // We're intaking from the intake into a drum slot:
                int slot = findDrumSlot(INTAKE_POSITIONS);
                if (slot != -1) {
                    // Accumulate the empty slots:
                    LinkedList<Integer> emptySlots = new LinkedList<>();
                    for (int i = 0; i < 3; i++) {
                        if (slotBalls.get(i) == null) {
                            emptySlots.add(i);
                        }
                    }
                    // Assign the ball to an empty slot:
                    if (!emptySlots.isEmpty()) {
                        int emptySlot = emptySlots.get((int) (Math.random() * emptySlots.size()));
                        slotBalls.set(emptySlot, intakeBall);
                        intakeBall = null;
                        hasIntaken = true;
                    }
                }
            }
        }
    }

    // Advance the Mech simulation.
    @Override
    public void advance(double deltaT) {
        // Note that we don't update our simulation until update() time, when we also know the
        // new robot pose.
        this.accumulatedDeltaT += deltaT;
    }

    // Render the robot and the balls.
    @Override
    public void update(Graphics2D g, Pose2d pose) {
        // Don't call simulate() or render() for things like Configuration Tester.
        if (upperLaunchMotor != null) {
            simulate(pose, accumulatedDeltaT);
            render(g, pose);
        }
        accumulatedDeltaT = 0;
    }
}
