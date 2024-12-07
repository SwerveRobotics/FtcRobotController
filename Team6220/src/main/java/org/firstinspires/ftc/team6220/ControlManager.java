package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.function.Predicate;

public class ControlManager {

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    private int armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_OUT;
    private int slidesMotorPosition;
    private double intakeServoPower;
    private double dumperServoPosition = DRIFTConstants.DUMPER_SERVO_POSITION_INIT;
    private double armElbowServoPosition = 0;
    private boolean shouldResetSlideEncoder;

    private double currentRunTime;

    boolean oldLeftBumperToggleState = false;

    private ProtectedButton slideEncoderResetToggle;

    private ArrayList<DelayedAction> delayedActions = new ArrayList<>();

    public ControlManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.slideEncoderResetToggle = new ProtectedButton();
    }

    public void update(double currentRunTime) {
        // update current run time
        this.currentRunTime = currentRunTime;

        // Raise arm elbow to go over submersible bar
        if(gamepad2.a && !gamepad2.b) {
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_OVER_BAR;
            clearList(armElbowServoPosition);
        }

        // Lower arm elbow to pick up samples
        if(gamepad2.b && !gamepad2.a) {
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_GROUND;
            clearList(armElbowServoPosition);
        }

        // Transfer sequence for intake to dumper
       if(gamepad2.x && !gamepad2.y) {
           armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_TRANSFER;
           armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_TRANSFER;
           dumperServoPosition = DRIFTConstants.DUMPER_SERVO_POSITION_TRANSFER;
           clearList(armElbowServoPosition);
           clearList(armBaseMotorPosition);
           clearList(dumperServoPosition);
        }

        // Raises the arm up to its initial position (against the hubs)
        if (gamepad2.y && !gamepad2.x) {
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_OVER_BAR;
            clearList(armElbowServoPosition);
        }

        // Powers the intake
        intakeServoPower = -gamepad2.left_stick_y;
        // slows it down if its in transfer position
        if (dumperServoPosition == DRIFTConstants.DUMPER_SERVO_POSITION_TRANSFER) {
            intakeServoPower *= 0.4;
        }

        // Lowers the arm to go over the submersible
        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            armBaseMotorPosition = DRIFTConstants.ARM_BASE_MOTOR_POSITION_OVER_BAR;
            clearList(armBaseMotorPosition);
        }

        // Lowers the slides to the ground position
        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            slidesMotorPosition = DRIFTConstants.SLIDES_MOTOR_GROUND_POSITION;
            // move dumper out of the way of the buckets
            dumperServoPosition = DRIFTConstants.DUMPER_SERVO_POSITION_TRANSFER;
            // move intake out of the way of the dumper
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_OVER_BAR;
            clearList(slidesMotorPosition);
            clearList(dumperServoPosition);
            clearList(armElbowServoPosition);
        }

        // Raises the slides to reach the high basket
        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            slidesMotorPosition = DRIFTConstants.SLIDES_MOTOR_HIGH_BASKET_POSITION;
            dumperServoPosition = DRIFTConstants.DUMPER_SERVO_POSITION_HORIZONTAL;
            clearList(slidesMotorPosition);
            clearList(dumperServoPosition);
        }
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            slidesMotorPosition = DRIFTConstants.SLIDES_MOTOR_LOW_BASKET_POSITION;
            dumperServoPosition = DRIFTConstants.DUMPER_SERVO_POSITION_HORIZONTAL;
            clearList(slidesMotorPosition);
            clearList(dumperServoPosition);
        }

        // Resets slide encoder if left stick button is pressed
        //shouldResetSlideEncoder = slideEncoderResetToggle.getToggleState(gamepad2.left_stick_button);


        // hacky crap code
        if (oldLeftBumperToggleState != gamepad2.left_bumper) {
            double interimDumperServoPosition = dumperServoPosition;
            dumperServoPosition = gamepad2.left_bumper ? DRIFTConstants.DUMPER_SERVO_POSITION_DUMP : DRIFTConstants.DUMPER_SERVO_POSITION_HORIZONTAL;
            if (interimDumperServoPosition != dumperServoPosition) {
                clearList(dumperServoPosition);
            }
        }
        oldLeftBumperToggleState = gamepad2.left_bumper;


        slidesMotorPosition += (int) (10 * -gamepad2.right_stick_y);

        if (gamepad2.right_stick_y != 0) {
            clearList(slidesMotorPosition);
        }

/*
        armElbowServoPosition = Math.abs(gamepad1.left_stick_y);
        intakeServoPower = -gamepad1.right_stick_y;
        slidesMotorPosition += (int) (10 * -gamepad2.left_stick_y);
        if (gamepad2.right_trigger >= 0) {
            armElbowServoPosition = DRIFTConstants.ARM_ELBOW_SERVO_POSITION_TRANSFER;
        }
 */
        // iterate through all the delayed actions, and run them if possible
        for (DelayedAction delayedAction : delayedActions) {
            delayedAction.runActionIfPossible();
        }
    }

    public int getArmBaseMotorPosition() {
        return armBaseMotorPosition;
    }

    public int getSlidesMotorPosition() {
        return slidesMotorPosition;
    }

    public double getIntakeServoPower() {
        return intakeServoPower;
    }

    public double getDumperServoPosition() {
        return dumperServoPosition;
    }

    public double getArmElbowServoPosition() {
        return armElbowServoPosition;
    }

    public boolean shouldResetSlideEncoder() {
        return shouldResetSlideEncoder;
    }


    private class ProtectedButton {
        private boolean wasPressed;

        public ProtectedButton() {
            this.wasPressed = false;
        }

        public boolean getToggleState(boolean protectedCondition) {
            // check that it isnt being held down
            boolean protectedOutput = protectedCondition && !wasPressed;
            // update button to being pressed
            this.wasPressed = protectedCondition;
            return protectedOutput;
        }
    }

    private class DelayedAction {
       private final Predicate<Boolean> completionCondition;
       private final Runnable runnableAction;
       private final double targetRunTime;
       private final Object trackedObject;

        public DelayedAction(Object trackedObject, Runnable runnableAction, double delaySeconds, Predicate<Boolean> completionCondition) {
            clearList(trackedObject);
            this.completionCondition = completionCondition;
            this.runnableAction = runnableAction;
            this.targetRunTime = currentRunTime + delaySeconds;
            this.trackedObject = trackedObject;
        }

        private Object getTrackedObject() {
            return trackedObject;
        }

        private void runActionIfPossible() {
            if (completionCondition.test(true) && currentRunTime >= targetRunTime) {
                runnableAction.run();
                delayedActions.remove(this);
            }
        }
    }

    private void addDelayedAction(Object trackedObject, Runnable runnableAction, double delaySeconds, Predicate<Boolean> completionCondition) {
        delayedActions.add(new DelayedAction(trackedObject, runnableAction, delaySeconds, completionCondition));
    }

    private void clearList(Object trackedObject) {
        delayedActions.removeIf(action -> action.getTrackedObject().equals(trackedObject));
    }
}
