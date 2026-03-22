package org.firstinspires.ftc.team417;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.apriltags.LimelightAprilTagDetector;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

enum RequestedColor { //an enum for different color cases for launching
    PURPLE,
    GREEN,
    EITHER,
    NONE
}
enum PixelColor {
    PURPLE,
    GREEN,
    NONE
}
enum LaunchDistance {
    FAR_AUTO,
    NEAR_AUTO,
    FAR,
    NEAR,
    OFF //turns the flywheel off
}

class MechGlob { //a placeholder class encompassing all code that ISN'T for slowbot.
    Telemetry telemetry;

    MechGlob() {
    }

    //call DrumGlob.create to create a Glob object for slowbot or fastbot
    static MechGlob create (HardwareMap hardwareMap, Telemetry telemetry, PixelColor[] preloads){

        if (MecanumDrive.isSlowBot) { //if the robot is slowbot, use ComplexMechGlob.
            return new ComplexMechGlob(hardwareMap, telemetry, preloads); //Go to ComplexMechGlob class

        } else { //otherwise, use MechGlob
            return new MechGlob(); //Go to MechGlob class
        }
    }
    //a method that controls the intake based on gamepad2.leftstickx
    //if gamepad2.left_stick_x is > 0, intakeSpeed = 1. If negative, intakeSpeed = -1. If 0, 0.
    void intake (double intakeValue){}

    //a method that determines what color to launch. Options are purple, green, or either.
    boolean launch (RequestedColor requestedColor, LimelightAprilTagDetector detector) {
        detector.tryResetRobotPose(telemetry); // Resets the robot pose only if the robot is not moving
        return true;
    }

    void update () {}

    boolean isDoneLaunching () {
        return true;
    }

    boolean preLaunch (RequestedColor requestedColor) {
        return true;
    }
    void setLaunchVelocity (LaunchDistance launchDistance) {}

    public PixelColor getSlotColor (int slotIndex) {
        return PixelColor.NONE;
    }
    void controlDrumManually () {}
    void ohCrap (boolean engage) {}

    void stopLaunch() {}


}

@Config
public class ComplexMechGlob extends MechGlob { //a class encompassing all code that IS for slowbot
    public static double FEEDER_POWER = 1;
    public static double TRANSFER_TIME_UP = 0.4; //was 0.6
    public static double TRANSFER_TIME_DOWN = 1.2; //was 0.25
    public static double PADDLE_TRANSFER_DELAY = 0.3;
    public static double PADDLE_TRANSFER_TIME_UP = 0.5; // How long we wait before bringing the paddles down (we don't need time for down because they don't interfere with drum)

    //todo can decrease this
    public static double DRUM_TRANSFER_WAIT_TIME = 0.5;
    // how long we wait before continuing after the color detector
    // detects. this is 0 because it will likely become obsolete
    public static double INTAKE_TIMER = 0;
    public static double FAR_FLYWHEEL_VELOCITY = 1000; //was 1500
    public static double NEAR_FLYWHEEL_VELOCITY = 810; //was 1500
    public static double FLYWHEEL_BACK_SPIN = 300; //was 300
    public static double TRANSFER_INACTIVE_POSITION = 0.45;
    public static double TRANSFER_ACTIVE_POSITION = 0.8;
    public static double REVERSE_INTAKE_SPEED = -1;
    public static double INTAKE_SPEED = 1;
    public static double FLYWHEEL_VELOCITY_TOLERANCE = 22; //this is an epsiiiiiiiiilon  was 10
    public static double VOLTAGE_TOLERANCE = 0.04; //THIS IS AN EPSILON AS WELLLLLL
    public static double DRUM_GATE_OPEN_POSITION = 0.5;
    public static double DRUM_GATE_CLOSED_POSITION = 0.75;
    public static double MOTOR_D_VALUE = 1;
    public static double INTAKE_BACK_TIME = 0.25;
    public static double NEAR_AUTO_VELOCTIY = 780;
    public static double FAR_AUTO_VELOCITY = 1040;
    public static double LEFT_PADDLE_INACTIVE_POSITION = 0.2;
    public static double RIGHT_PADDLE_INACTIVE_POSITION = 0.2;
    public static double LEFT_PADDLE_ACTIVE_POSITION = 0.8;
    public static double RIGHT_PADDLE_ACTIVE_POSITION = 0.8;

    ElapsedTime transferTimer;
    ElapsedTime transferDownTimer;
    ElapsedTime intakeTimer;
    ElapsedTime intakeBackTimer;
    double userIntakeSpeed;
    ArrayList<DrumRequest> drumQueue = new ArrayList<> ();

    ArrayList<PixelColor> slotOccupiedBy = new ArrayList<> (Collections.nCopies(3, PixelColor.NONE));
    enum WaitState {
        DRUM_MOVE_WAIT, //waiting for the ball to fully enter the slot before moving the drum
        DRUM_MOVE, //waiting for the drum to reach desired position
        INTAKE, //waiting for the intake to finish
        SCAN, //check the contents of the drum
        TRANSFER, //waiting for the transfer to finish
        TRANSFER_DOWN, //when stopLaunch is called, waits for transfer to go down
        LAUNCH_AFTER_SCAN, //launch the color that was requested before scan
        SPIN_UP, //waiting for the flywheel(s) to spin up
        IDLE, //waiting for input when the drum is full

    }
    WaitState waitState = WaitState.IDLE;
    // arrays with placeholder values for servo positions and voltages relative to intake and launch
    double [] INTAKE_POSITIONS = {0.078, 0.451, 0.814};
    double [] INTAKE_VOLTS = {2.94, 1.83, 0.74};
    double [] LAUNCH_POSITIONS = {0.638, 1.11, 0.269};
    double [] LAUNCH_VOLTS = {1.27, 0.155, 2.37};
    double lastQueuedPosition; //where the servo was *queued* to go last. NOT THE SAME AS hwDrumPosition!
    double hwDrumPosition; //where the drum was *told* to go last. NOT THE SAME AS lastQueuedPosition!
    double upperLaunchVelocity;
    double lowerLaunchVelocity;
    double feederPower;
    boolean engageOhCrap;
    boolean scanRequired; //When the intake runs, assume we have a new ball and allow scanning again
    LaunchDistance launchDistance = LaunchDistance.OFF;


    HardwareMap hardwareMap;

    //hardware objects
    Servo servoDrum;
    Servo servoTransfer;
    AnalogInput analogDrum;
    DcMotorEx motLLauncher;
    DcMotorEx motULauncher;
    DcMotorEx motIntake;
    Servo servoDrumGate;
    NormalizedColorSensor sensorColor1;
    NormalizedColorSensor sensorColor2;
    Servo servoLeftPaddle;
    Servo servoRightPaddle;

    CoolColorDetector coolColorDetector;

    class DrumRequest {
        double newPosition;
        double oldPosition;
        WaitState nextState;
        RequestedColor requestedColor;

        public DrumRequest(double newPosition, double oldPosition, WaitState nextState, RequestedColor requestedColor) {
            this.nextState = nextState;
            this.newPosition = newPosition;
            this.oldPosition = oldPosition;
            this.requestedColor = requestedColor;
        }
    }
    ComplexMechGlob (HardwareMap hardwareMap, Telemetry telemetry, PixelColor[] preloads) {

        //this changes some lists if we are using WilyWorks
        if (WilyWorks.isSimulating) {
            INTAKE_POSITIONS = new double[]{0 / 6.0, 2 / 6.0, 4 / 6.0};
            LAUNCH_POSITIONS = new double[]{3 / 6.0, 5 / 6.0, 1.0 / 6};
            INTAKE_VOLTS = new double[]{3.5 * 0 / 6.0, 3.5 * 2 / 6.0, 3.5 * 4 / 6.0};
            LAUNCH_VOLTS = new double[]{3.5 * 3 / 6.0, 3.5 * 5 / 6.0, 3.5 * 1.0 / 6};
        }

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        servoDrum = hardwareMap.get(Servo.class, "servoDrum");
        servoTransfer = hardwareMap.get(Servo.class, "servoTransfer");
        analogDrum = hardwareMap.get(AnalogInput.class, "analogDrum");
        motLLauncher = hardwareMap.get(DcMotorEx.class, "motLLauncher");
        motULauncher = hardwareMap.get(DcMotorEx.class, "motULauncher");
        motIntake = hardwareMap.get(DcMotorEx.class, "motIntake");
        servoDrumGate = hardwareMap.get(Servo.class, "servoDrumGate");
        servoRightPaddle = hardwareMap.get(Servo.class, "servoRightPaddle");
        servoLeftPaddle = hardwareMap.get(Servo.class, "servoLeftPaddle");
        coolColorDetector = new CoolColorDetector(hardwareMap, telemetry);
        slotOccupiedBy = new ArrayList<>(Arrays.asList(preloads));


        /*
         * Here we set our flywheels to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        motLLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motULauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the motors to a braking behavior so it slows down faster when left trigger is pressed
        motLLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motULauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motLLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));
        motULauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, MOTOR_D_VALUE, 10));

        motULauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        motLLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        servoRightPaddle.setDirection(Servo.Direction.REVERSE);

        servoRightPaddle.setPosition(RIGHT_PADDLE_INACTIVE_POSITION);
        servoLeftPaddle.setPosition(LEFT_PADDLE_INACTIVE_POSITION);





    }

    //the position argument denotes whether we are using intake or launch positions
    //position takes INTAKE_POSITIONS or LAUNCH_POSITIONS.
    int findNearestSlot (double [] position, RequestedColor requestedColor) {

        double minDistance = Double.MAX_VALUE; //the minimum distance to a slot that has what we want
        int minSlot = -1; // this will only ever be 0, 1, or 2. -1 represents a invalid value

        // a for loop that will determine what slot has the requested color.
        for (int i = 0; i <= 2; i++){ //here, the integer i represents the slot we are currently checking
            double distance = Math.abs(position[i] - lastQueuedPosition);
            //each conditional checks if what we requested and what we have in a specific slot matches.
            if (distance < minDistance){
                if (requestedColor == RequestedColor.PURPLE && slotOccupiedBy.get (i) == PixelColor.PURPLE){
                    minDistance = distance;
                    minSlot = i;// if it does, mark the current slot as the nearest slot
                } else if (requestedColor == RequestedColor.GREEN && slotOccupiedBy.get (i) == PixelColor.GREEN){
                    minDistance = distance;
                    minSlot = i;
                } else if (requestedColor == RequestedColor.EITHER && slotOccupiedBy.get (i) != PixelColor.NONE){
                    minDistance = distance;
                    minSlot = i;
                } else if (requestedColor == RequestedColor.NONE && slotOccupiedBy.get (i) == PixelColor.NONE){
                    minDistance = distance;
                    minSlot = i;
                }
            }

        }
        return minSlot;
    }

    @Override
    void intake (double intakeSpeed) {
        userIntakeSpeed = intakeSpeed;

    }

    @Override
    // controls the launcher and transfer
    boolean launch (RequestedColor requestedColor, LimelightAprilTagDetector detector) {
        detector.tryResetRobotPose(telemetry); // Resets the robot pose only if the robot is not moving

        if (launchDistance == LaunchDistance.OFF) {
            launchDistance = LaunchDistance.NEAR;
        }

        if (requestedColor != RequestedColor.EITHER && scanRequired) {
            addToDrumQueue(INTAKE_POSITIONS[0], WaitState.SCAN);
            addToDrumQueue(INTAKE_POSITIONS[1], WaitState.SCAN);
            addToDrumQueue(INTAKE_POSITIONS[2], WaitState.SCAN);
            addToDrumQueue(INTAKE_POSITIONS[0], WaitState.LAUNCH_AFTER_SCAN, requestedColor);
            scanRequired = false;
        } else {
            int minSlot = findNearestSlot(LAUNCH_POSITIONS, requestedColor);
            if (minSlot != -1) {
                addToDrumQueue(LAUNCH_POSITIONS[minSlot], WaitState.SPIN_UP);
                slotOccupiedBy.set(minSlot, PixelColor.NONE); //marking this slot as empty so we don't accidentally try to use it again
                return true;
            }
        }

        return false;
    }

    // Adds new drum request to the queue when we don't need to specify requested color
    void addToDrumQueue(double position, WaitState nextState) {
        addToDrumQueue(position, nextState, RequestedColor.EITHER);
    }

    // This function adds a new drum request to the drum queue when we need to specify requested color.
    // nextState is the state do use after the drum is finished moving
    void addToDrumQueue(double position, WaitState nextState, RequestedColor requestedColor){
        drumQueue.add(new DrumRequest(position, lastQueuedPosition, nextState, requestedColor));
        lastQueuedPosition = position;
    }

    boolean drumAtPosition() {

        int intakeSlot = findSlotFromPosition(hwDrumPosition, INTAKE_POSITIONS);
        int launchSlot = findSlotFromPosition(hwDrumPosition, LAUNCH_POSITIONS);
        double expectedVolts;

        if (intakeSlot != -1) {
            expectedVolts = INTAKE_VOLTS[intakeSlot];
        } else {
            expectedVolts = LAUNCH_VOLTS[launchSlot];
        }
        telemetry.addLine(String.format("Position Delta: %.4f",analogDrum.getVoltage() - expectedVolts));
        return Math.abs(analogDrum.getVoltage() - expectedVolts) <= VOLTAGE_TOLERANCE;
    }
    @Override
    boolean isDoneLaunching () {
        return drumQueue.isEmpty() && (waitState == WaitState.IDLE || waitState == WaitState.INTAKE);
    }
    @Override
    //this function is just for auto. it rotates to the requested color but does not launch (to save time)
    boolean preLaunch (RequestedColor requestedColor) {
        int minSlot = findNearestSlot(LAUNCH_POSITIONS, requestedColor);
        if (minSlot == -1){
            return false;
        } else {
            addToDrumQueue(LAUNCH_POSITIONS[minSlot], WaitState.IDLE);
            return true;
        }
    }
    @Override
    void setLaunchVelocity (LaunchDistance launchDistance) {
        this.launchDistance = launchDistance;
    }
    void calculateLaunchVelocity () {
        if (launchDistance == LaunchDistance.NEAR) {
            upperLaunchVelocity = NEAR_FLYWHEEL_VELOCITY - (0.5 * FLYWHEEL_BACK_SPIN);
            lowerLaunchVelocity = NEAR_FLYWHEEL_VELOCITY + (0.5 * FLYWHEEL_BACK_SPIN);
            feederPower = FEEDER_POWER;
        } else if (launchDistance == LaunchDistance.FAR){
            upperLaunchVelocity = FAR_FLYWHEEL_VELOCITY - (0.5 * FLYWHEEL_BACK_SPIN);
            lowerLaunchVelocity = FAR_FLYWHEEL_VELOCITY + (0.5 * FLYWHEEL_BACK_SPIN);
            feederPower = FEEDER_POWER;
        } else if (launchDistance == LaunchDistance.NEAR_AUTO){
            upperLaunchVelocity = NEAR_AUTO_VELOCTIY - (0.5 * FLYWHEEL_BACK_SPIN);
            lowerLaunchVelocity = NEAR_AUTO_VELOCTIY + (0.5 * FLYWHEEL_BACK_SPIN);
            feederPower = FEEDER_POWER;
        } else if (launchDistance == LaunchDistance.FAR_AUTO){
            upperLaunchVelocity = FAR_AUTO_VELOCITY - (0.5 * FLYWHEEL_BACK_SPIN);
            lowerLaunchVelocity = FAR_AUTO_VELOCITY + (0.5 * FLYWHEEL_BACK_SPIN);
            feederPower = FEEDER_POWER;
        }
        else {
            upperLaunchVelocity = 0;
            lowerLaunchVelocity = 0;
            feederPower = 0;
        }
    }
    int findSlotFromPosition (double position, double [] positions) {
        for (int i = 0; i < positions.length; i++) {
            if (positions [i] == position){
               return i;
            }
        }
        return -1;
    }

    void controlDrumManually () {
        int currentSlot = findSlotFromPosition(hwDrumPosition, INTAKE_POSITIONS);
        if (currentSlot != -1) {
            slotOccupiedBy.set(currentSlot, PixelColor.PURPLE);
        }
        int minSlot = findNearestSlot(INTAKE_POSITIONS, RequestedColor.NONE);
        if (minSlot != -1) {
            addToDrumQueue(INTAKE_POSITIONS[minSlot], WaitState.INTAKE);
        }

    }
    //if the gate is stuck, move the drum back to get the stuck ball out, along with opening gate
    @Override
    void ohCrap (boolean engage) {
        engageOhCrap = engage;
        if (engage) {
            if (drumQueue.size() == 1) {
                DrumRequest request = drumQueue.get(0);
                hwDrumPosition = request.oldPosition;
                lastQueuedPosition = request.oldPosition;
            }
        }
    }

    @Override
    public PixelColor getSlotColor (int slotIndex) {
        PixelColor artifactColor = slotOccupiedBy.get(slotIndex);
        return artifactColor;
    }

    void scan () {
        PixelColor slotColor = coolColorDetector.detectArtifactColor();
        int slot = findSlotFromPosition(hwDrumPosition, INTAKE_POSITIONS);
        slotOccupiedBy.set(slot, slotColor);
    }

    @Override
    void stopLaunch() {
        if (waitState == WaitState.TRANSFER) {
            waitState = WaitState.TRANSFER_DOWN;
        }
    }

    @Override
    void update () {
        double intakePower = 0;
        double gatePosition;
        calculateLaunchVelocity();

        if (waitState == WaitState.DRUM_MOVE_WAIT) {
            // always run the intake, even while we're waiting for the ball to enter the drum
            intakePower = INTAKE_SPEED;
        } else if (userIntakeSpeed < 0 ) {
            // allow the intake to run if the driver wants it to
            intakePower = REVERSE_INTAKE_SPEED;
        } else if (userIntakeSpeed > 0) {
            intakePower = INTAKE_SPEED;
        }
            // if we are in the intake waitState, allow the intake to run
//            if (waitState == WaitState.INTAKE) {
//                intakePower = INTAKE_SPEED;
//            } else if (!drumQueue.isEmpty() && drumQueue.get(0).nextState == WaitState.INTAKE) {
//                intakePower = INTAKE_SPEED;
//            }
        // whenever we see a ball in the drum then we move the intake back so the ball immediately
        // so it doesn't get stuck in the drum
        if (intakeBackTimer != null) {
            if(intakeBackTimer.seconds() < INTAKE_BACK_TIME) {
                intakePower = -1;
            } else {
                intakeBackTimer = null;
            }
        }

        if (intakePower > 0 && waitState != WaitState.DRUM_MOVE) {
            gatePosition = DRUM_GATE_OPEN_POSITION; // Open gate to allow ball in
            scanRequired = true; // New ball means we will need to detect color if driver presses color launch
        } else {
            gatePosition = DRUM_GATE_CLOSED_POSITION; // By default close gate to prevent balls from falling out
        }
        if (engageOhCrap) {
            gatePosition = DRUM_GATE_OPEN_POSITION;
        }
        telemetry.addLine(String.format("intake: %.1f, waitState: %s, gatePosition: %.1f",
                intakePower, waitState, gatePosition));

        // TODO might be able to remove this state - and stay in current state (for intake/scan) instead of going here
        if (waitState == WaitState.IDLE) {
            // When driver starts the intake, need to rotate to an intake slot
            if (userIntakeSpeed > 0) {
                // At the beginning or after launching, all slots should be NONE. Find the nearest one.
                int minSlot = findNearestSlot(INTAKE_POSITIONS, RequestedColor.NONE);

                // TODO might not need this. Test without it and delete if not needed.
                if (minSlot == -1) {
                    minSlot = findNearestSlot(INTAKE_POSITIONS, RequestedColor.PURPLE);
                }

                // Check if our current slot is an intake slot. In that case, we don't need to do anything
                int currSlot = findSlotFromPosition(hwDrumPosition, INTAKE_POSITIONS);

                // If it isn't, go to the intake slot we found earlier
                if (currSlot == -1 && minSlot != -1) {
                    addToDrumQueue(INTAKE_POSITIONS[minSlot], WaitState.INTAKE);
                }

                waitState = WaitState.INTAKE;
            }

            // this makes it so that after we are done launching the drum goes to intake position
//            if (drumQueue.isEmpty() && slotOccupiedBy.stream().allMatch(e -> e == PixelColor.NONE)) {
//                addToDrumQueue(INTAKE_POSITIONS[0], WaitState.INTAKE);
//            }
        }
        if (waitState == WaitState.SCAN) {
            // Rotate to all 3 slots and then detect and store colors
            scan();

            // Then go to IDLE to wait for launch request
            waitState = WaitState.IDLE;
        }

        // let a firing request interrupt an intake
        if (waitState == WaitState.IDLE || waitState == WaitState.INTAKE) {
            // If there is a request in drumQueue find the requested position and go to DRUM_MOVE to move there
            if (!drumQueue.isEmpty()) {
                hwDrumPosition = drumQueue.get(0).newPosition;
                waitState = WaitState.DRUM_MOVE;
            }
        }

        if (waitState == WaitState.DRUM_MOVE) {
            if (drumAtPosition()) {
                DrumRequest request = drumQueue.get(0);
                waitState = request.nextState;
                RequestedColor requestedColor = request.requestedColor;
                drumQueue.remove(0);
                if (waitState == WaitState.LAUNCH_AFTER_SCAN) {
                    int minSlot = findNearestSlot(LAUNCH_POSITIONS, requestedColor);
                    if (minSlot != -1) {
                        addToDrumQueue(LAUNCH_POSITIONS[minSlot], WaitState.SPIN_UP);
                        slotOccupiedBy.set(minSlot, PixelColor.NONE); //marking this slot as empty so we don't accidentally try to use it again
                    }
                    waitState = WaitState.IDLE;

                }
            }
        }
        if (waitState == WaitState.SPIN_UP) {
            if (Math.abs(motLLauncher.getVelocity() - lowerLaunchVelocity) <= FLYWHEEL_VELOCITY_TOLERANCE &&
                    Math.abs(motULauncher.getVelocity() - upperLaunchVelocity) <= FLYWHEEL_VELOCITY_TOLERANCE) {
                waitState = WaitState.TRANSFER;
            }
        }
        double transferPosition = TRANSFER_INACTIVE_POSITION;
        double leftPaddlePosition = LEFT_PADDLE_INACTIVE_POSITION;
        double rightPaddlePosition = RIGHT_PADDLE_INACTIVE_POSITION;
        if (waitState == WaitState.TRANSFER) {
            if (transferTimer == null) {
                transferTimer = new ElapsedTime();
            }

            // Move the 4bar transfer up
            if ((transferTimer.seconds() <= TRANSFER_TIME_UP+DRUM_TRANSFER_WAIT_TIME) && (transferTimer.seconds() >= DRUM_TRANSFER_WAIT_TIME)) {
                transferPosition = TRANSFER_ACTIVE_POSITION;
            }
            // Wait to ensure the 4bar is fully up, then bring the paddles up
            if ((transferTimer.seconds() >= PADDLE_TRANSFER_DELAY+DRUM_TRANSFER_WAIT_TIME) && (transferTimer.seconds() <= PADDLE_TRANSFER_DELAY + PADDLE_TRANSFER_TIME_UP + DRUM_TRANSFER_WAIT_TIME)) {
                leftPaddlePosition = LEFT_PADDLE_ACTIVE_POSITION;
                rightPaddlePosition = RIGHT_PADDLE_ACTIVE_POSITION;
            }
            // Everything goes down and we go to IDLE
            if (transferTimer.seconds() >= TRANSFER_TIME_UP + TRANSFER_TIME_DOWN+DRUM_TRANSFER_WAIT_TIME) {
                waitState = WaitState.IDLE;
                transferTimer = null;
            }
        }

        if (waitState == WaitState.INTAKE) {
            launchDistance = LaunchDistance.OFF;
            // By default we assume all 3 slots are purple
            // We will check the color later if the driver needs to launch a color
            slotOccupiedBy.set(0, PixelColor.PURPLE);
            slotOccupiedBy.set(1, PixelColor.PURPLE);
            slotOccupiedBy.set(2, PixelColor.PURPLE);
        }

        // Manually overrides an ongoing launch
        if (waitState == WaitState.TRANSFER_DOWN) {
            if (transferDownTimer == null) {
                transferDownTimer = new ElapsedTime();

                // Mark slot as empty because we stopped the transfer
                int currSlot = findSlotFromPosition(hwDrumPosition, LAUNCH_POSITIONS);
                slotOccupiedBy.set(currSlot, PixelColor.NONE);
            }
            // If transfer is fully down, go to IDLE to wait for next action
            if (transferDownTimer.seconds() > TRANSFER_TIME_DOWN) {
                waitState = WaitState.IDLE;
                transferTimer = null;
                transferDownTimer = null;
            }
        }

        //TODO remove?
        if (waitState == WaitState.DRUM_MOVE_WAIT) {
            if (intakeTimer.seconds() >= INTAKE_TIMER) {
                waitState = WaitState.IDLE;
            }
        }

        servoDrum.setPosition(hwDrumPosition);
        servoLeftPaddle.setPosition(leftPaddlePosition);
        servoRightPaddle.setPosition(rightPaddlePosition);
        servoTransfer.setPosition(transferPosition);
        servoDrumGate.setPosition(gatePosition);
        motLLauncher.setVelocity(lowerLaunchVelocity);
        motULauncher.setVelocity(upperLaunchVelocity);
        motIntake.setPower(intakePower);
        telemetry.addData("hwDrumPos", hwDrumPosition);
        telemetry.addData("transferpos", transferPosition);
        telemetry.addData("currVoltage ", "%.2f", analogDrum.getVoltage());
    }
}

