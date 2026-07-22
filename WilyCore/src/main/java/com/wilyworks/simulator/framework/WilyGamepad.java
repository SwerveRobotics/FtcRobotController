package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.util.ArrayList;

/**
 * Monitors changes to buttons on a Gamepad.
 * <p>
 * wasPressed() will return true if the button was pressed since the last call
 * of that method.
 * <p>
 * wasReleased() will return ture if the button was released since the last call
 * of that method.
 * <p>
 * @see Gamepad
 */
class GamepadStateChanges {
    protected ButtonStateMonitor dpadDown = new ButtonStateMonitor();
    protected ButtonStateMonitor dpadLeft = new ButtonStateMonitor();
    protected ButtonStateMonitor dpadRight = new ButtonStateMonitor();
    protected ButtonStateMonitor a = new ButtonStateMonitor();
    protected ButtonStateMonitor dpadUp = new ButtonStateMonitor();
    protected ButtonStateMonitor b = new ButtonStateMonitor();
    protected ButtonStateMonitor x = new ButtonStateMonitor();
    protected ButtonStateMonitor y = new ButtonStateMonitor();
    protected ButtonStateMonitor guide = new ButtonStateMonitor();
    protected ButtonStateMonitor start = new ButtonStateMonitor();
    protected ButtonStateMonitor back = new ButtonStateMonitor();
    protected ButtonStateMonitor leftBumper = new ButtonStateMonitor();
    protected ButtonStateMonitor rightBumper = new ButtonStateMonitor();
    protected ButtonStateMonitor leftStickButton = new ButtonStateMonitor();
    protected ButtonStateMonitor rightStickButton = new ButtonStateMonitor();
    protected ButtonStateMonitor circle = new ButtonStateMonitor();
    protected ButtonStateMonitor cross = new ButtonStateMonitor();
    protected ButtonStateMonitor triangle = new ButtonStateMonitor();
    protected ButtonStateMonitor square = new ButtonStateMonitor();
    protected ButtonStateMonitor share = new ButtonStateMonitor();
    protected ButtonStateMonitor options = new ButtonStateMonitor();
    protected ButtonStateMonitor touchpad = new ButtonStateMonitor();
    protected ButtonStateMonitor ps = new ButtonStateMonitor();

    protected class ButtonStateMonitor {
        private boolean lastPressed = false;
        private boolean pressNotify = false;
        private boolean releaseNotify = false;

        private void update(boolean nowPressed) {
            if (!lastPressed && nowPressed) {
                pressNotify = true;
            }
            if (lastPressed && !nowPressed) {
                releaseNotify = true;
            }
            lastPressed = nowPressed;
        }

        protected boolean wasPressed() {
            boolean pressed = pressNotify;
            pressNotify = false;
            return pressed;
        }

        protected boolean wasReleased() {
            boolean released = releaseNotify;
            releaseNotify = false;
            return released;
        }
    }

    protected void updateAllButtons(WilyGamepad gamepad) {
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        guide.update(gamepad.guide);
        start.update(gamepad.start);
        back.update(gamepad.back);
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);
        circle.update(gamepad.circle);
        cross.update(gamepad.cross);
        triangle.update(gamepad.triangle);
        square.update(gamepad.square);
        share.update(gamepad.share);
        options.update(gamepad.options);
        touchpad.update(gamepad.touchpad);
        ps.update(gamepad.ps);
    }
}

/**
 * Wily Works Gamepad implementation that takes input either from a connected gamepad or
 * from the keyboard.
 */
public class WilyGamepad {


    public enum Type {
        // Do NOT change the order/names of existing entries,
        // you will break backwards compatibility!!
        UNKNOWN(LegacyType.UNKNOWN),
        LOGITECH_F310(LegacyType.LOGITECH_F310),
        XBOX_360(LegacyType.XBOX_360),
        SONY_PS4(LegacyType.SONY_PS4), // This indicates a PS4-compatible controller that is being used through our compatibility mode
        SONY_PS4_SUPPORTED_BY_KERNEL(LegacyType.SONY_PS4); // This indicates a PS4-compatible controller that is being used through the DualShock 4 Linux kernel driver.

        private final LegacyType correspondingLegacyType;
        Type(LegacyType correspondingLegacyType) {
            this.correspondingLegacyType = correspondingLegacyType;
        }
    }

    // LegacyType is necessary because robocol gamepad version 3 was written in a way that was not
    // forwards-compatible, so we have to keep sending V3-compatible values.
    public enum LegacyType {
        // Do NOT change the order or names of existing entries, or add new entries.
        // You will break backwards compatibility!!
        UNKNOWN,
        LOGITECH_F310,
        XBOX_360,
        SONY_PS4;
    }

    /**
     * Get the type of gamepad as a {@link Type}. This method defaults to "UNKNOWN".
     * @return gamepad type
     */
    public Type type() {
        return type;
    }

    @SuppressWarnings("UnusedAssignment")
    public volatile Type type = Type.XBOX_360; // IntelliJ thinks this is redundant, but it is NOT. Must be a bug in the analyzer?

    public volatile float left_stick_x = 0f;
    public volatile float left_stick_y = 0f;
    public volatile float right_stick_x = 0f;
    public volatile float right_stick_y = 0f;
    public volatile boolean dpad_up = false;
    public volatile boolean dpad_down = false;
    public volatile boolean dpad_left = false;
    public volatile boolean dpad_right = false;
    public volatile boolean a = false;
    public volatile boolean b = false;
    public volatile boolean x = false;
    public volatile boolean y = false;
    public volatile boolean guide = false;
    public volatile boolean start = false;
    public volatile boolean back = false;
    public volatile boolean left_bumper = false;
    public volatile boolean right_bumper = false;
    public volatile boolean left_stick_button = false;
    public volatile boolean right_stick_button = false;
    public volatile float left_trigger = 0f;
    public volatile float right_trigger = 0f;
    public volatile boolean circle = false;
    public volatile boolean cross = false;
    public volatile boolean triangle = false;
    public volatile boolean square = false;
    public volatile boolean share = false;
    public volatile boolean options = false;
    public volatile boolean ps = false;

    public volatile boolean touchpad = false;
    public volatile boolean touchpad_finger_1;
    public volatile boolean touchpad_finger_2;
    public volatile float touchpad_finger_1_x;
    public volatile float touchpad_finger_1_y;
    public volatile float touchpad_finger_2_x;
    public volatile float touchpad_finger_2_y;

    public WilyGamepad() {
    }

    /**
     * Edge detection for gamepads
     */
    private volatile GamepadStateChanges changes = new GamepadStateChanges();

    public void updateButtonAliases(){
        // There is no assignment for touchpad because there is no equivalent on XBOX controllers.
        circle = b;
        cross = a;
        triangle = y;
        square = x;
        share = back;
        options = start;
        ps = guide;
    }

    public void updateEdgeDetection(){
        changes.updateAllButtons(this);
    }

    public void runRumbleEffect(RumbleEffect effect) { }
    public void rumble(int durationMs) { }
    public void rumble(double rumble1, double rumble2, int durationMs) { }
    public void stopRumble() { }
    public void rumbleBlips(int count) { }

    public static class RumbleEffect {
        public static class Step {
            public int large;
            public int small;
            public int duration;
        }

        public int user;
        public final ArrayList<Step> steps;
        private RumbleEffect(ArrayList<Step> steps) {
            this.steps = steps;
        }
        public String serialize() { return ""; }
        public static RumbleEffect deserialize(String serialized) {
            return new RumbleEffect(new ArrayList<>());
        }
        public static class Builder {
            public Builder addStep(double rumble1, double rumble2, int durationMs) {
                return this;
            }
            public RumbleEffect build() {
                return new RumbleEffect(new ArrayList<>());
            }
        }
    }

    public GamepadUser getUser() {
        return GamepadUser.ONE;
    }

    /**
     * Checks if dpad_up was pressed since the last call of this method
     * @return true if dpad_up was pressed since the last call of this method; otherwise false
     */
    public boolean dpadUpWasPressed() {
        return changes.dpadUp.wasPressed();
    }

    /**
     * Checks if dpad_up was released since the last call of this method
     * @return true if dpad_up was released since the last call of this method; otherwise false
     */
    public boolean dpadUpWasReleased() {
        return changes.dpadUp.wasReleased();
    }

    /**
     * Checks if dpad_down was pressed since the last call of this method
     * @return true if dpad_down was pressed since the last call of this method; otherwise false
     */
    public boolean dpadDownWasPressed() {
        return changes.dpadDown.wasPressed();
    }

    /**
     * Checks if dpad_down was released since the last call of this method
     * @return true if dpad_down was released since the last call of this method; otherwise false
     */
    public boolean dpadDownWasReleased() {
        return changes.dpadDown.wasReleased();
    }

    /**
     * Checks if dpad_left was pressed since the last call of this method
     * @return true if dpad_left was pressed since the last call of this method; otherwise false
     */
    public boolean dpadLeftWasPressed() {
        return changes.dpadLeft.wasPressed();
    }

    /**
     * Checks if dpad_left was released since the last call of this method
     * @return true if dpad_left was released since the last call of this method; otherwise false
     */
    public boolean dpadLeftWasReleased() {
        return changes.dpadLeft.wasReleased();
    }

    /**
     * Checks if dpad_right was pressed since the last call of this method
     * @return true if dpad_right was pressed since the last call of this method; otherwise false
     */
    public boolean dpadRightWasPressed() {
        return changes.dpadRight.wasPressed();
    }

    /**
     * Checks if dpad_right was released since the last call of this methmethodod
     * @return true if dpad_right was released since the last call of this ; otherwise false
     */
    public boolean dpadRightWasReleased() {
        return changes.dpadRight.wasReleased();
    }

    /**
     * Checks if a was pressed since the last call of this method
     * @return true if a was pressed since the last call of this method; otherwise false
     */
    public boolean aWasPressed() {
        return changes.a.wasPressed();
    }

    /**
     * Checks if a was released since the last call of this method
     * @return true if a was released since the last call of this method; otherwise false
     */
    public boolean aWasReleased() {
        return changes.a.wasReleased();
    }

    /**
     * Checks if b was pressed since the last call of this method
     * @return true if b was pressed since the last call of this method; otherwise false
     */
    public boolean bWasPressed() {
        return changes.b.wasPressed();
    }

    /**
     * Checks if b was released since the last call of this method
     * @return true if b was released since the last call of this method; otherwise false
     */
    public boolean bWasReleased() {
        return changes.b.wasReleased();
    }

    /**
     * Checks if x was pressed since the last call of this method
     * @return true if x was pressed since the last call of this method; otherwise false
     */
    public boolean xWasPressed() {
        return changes.x.wasPressed();
    }

    /**
     * Checks if x was released since the last call of this method
     * @return true if x was released since the last call of this method; otherwise false
     */
    public boolean xWasReleased() {
        return changes.x.wasReleased();
    }

    /**
     * Checks if y was pressed since the last call of this method
     * @return true if y was pressed since the last call of this method; otherwise false
     */
    public boolean yWasPressed() {
        return changes.y.wasPressed();
    }

    /**
     * Checks if y was released since the last call of this method
     * @return true if y was released since the last call of this method; otherwise false
     */
    public boolean yWasReleased() {
        return changes.y.wasReleased();
    }

    /**
     * Checks if guide was pressed since the last call of this method
     * @return true if guide was pressed since the last call of this method; otherwise false
     */
    public boolean guideWasPressed() {
        return changes.guide.wasPressed();
    }

    /**
     * Checks if guide was released since the last call of this method
     * @return true if guide was released since the last call of this method; otherwise false
     */
    public boolean guideWasReleased() {
        return changes.guide.wasReleased();
    }

    /**
     * Checks if start was pressed since the last call of this method
     * @return true if start was pressed since the last call of this method; otherwise false
     */
    public boolean startWasPressed() {
        return changes.start.wasPressed();
    }

    /**
     * Checks if start was released since the last call of this method
     * @return true if start was released since the last call of this method; otherwise false
     */
    public boolean startWasReleased() {
        return changes.start.wasReleased();
    }

    /**
     * Checks if back was pressed since the last call of this method
     * @return true if back was pressed since the last call of this method; otherwise false
     */
    public boolean backWasPressed() {
        return changes.back.wasPressed();
    }

    /**
     * Checks if back was released since the last call of this method
     * @return true if back was released since the last call of this method; otherwise false
     */
    public boolean backWasReleased() {
        return changes.back.wasReleased();
    }

    /**
     * Checks if left_bumper was pressed since the last call of this method
     * @return true if left_bumper was pressed since the last call of this method; otherwise false
     */
    public boolean leftBumperWasPressed() {
        return changes.leftBumper.wasPressed();
    }

    /**
     * Checks if left_bumper was released since the last call of this method
     * @return true if left_bumper was released since the last call of this method; otherwise false
     */
    public boolean leftBumperWasReleased() {
        return changes.leftBumper.wasReleased();
    }

    /**
     * Checks if right_bumper was pressed since the last call of this method
     * @return true if right_bumper was pressed since the last call of this method; otherwise false
     */
    public boolean rightBumperWasPressed() {
        return changes.rightBumper.wasPressed();
    }

    /**
     * Checks if right_bumper was released since the last call of this method
     * @return true if right_bumper was released since the last call of this method; otherwise false
     */
    public boolean rightBumperWasReleased() {
        return changes.rightBumper.wasReleased();
    }

    /**
     * Checks if left_stick_button was pressed since the last call of this method
     * @return true if left_stick_button was pressed since the last call of this method; otherwise false
     */
    public boolean leftStickButtonWasPressed() {
        return changes.leftStickButton.wasPressed();
    }

    /**
     * Checks if left_stick_button was released since the last call of this method
     * @return true if left_stick_button was released since the last call of this method; otherwise false
     */
    public boolean leftStickButtonWasReleased() {
        return changes.leftStickButton.wasReleased();
    }

    /**
     * Checks if right_stick_button was pressed since the last call of this method
     * @return true if right_stick_button was pressed since the last call of this method; otherwise false
     */
    public boolean rightStickButtonWasPressed() {
        return changes.rightStickButton.wasPressed();
    }

    /**
     * Checks if right_stick_button was released since the last call of this method
     * @return true if right_stick_button was released since the last call of this method; otherwise false
     */
    public boolean rightStickButtonWasReleased() {
        return changes.rightStickButton.wasReleased();
    }

    /**
     * Checks if circle was pressed since the last call of this method
     * @return true if circle was pressed since the last call of this method; otherwise false
     */
    public boolean circleWasPressed() {
        return changes.circle.wasPressed();
    }

    /**
     * Checks if circle was released since the last call of this method
     * @return true if circle was released since the last call of this method; otherwise false
     */
    public boolean circleWasReleased() {
        return changes.circle.wasReleased();
    }

    /**
     * Checks if cross was pressed since the last call of this method
     * @return true if cross was pressed since the last call of this method; otherwise false
     */
    public boolean crossWasPressed() {
        return changes.cross.wasPressed();
    }

    /**
     * Checks if cross was released since the last call of this method
     * @return true if cross was released since the last call of this method; otherwise false
     */
    public boolean crossWasReleased() {
        return changes.cross.wasReleased();
    }

    /**
     * Checks if triangle was pressed since the last call of this method
     * @return true if triangle was pressed since the last call of this method; otherwise false
     */
    public boolean triangleWasPressed() {
        return changes.triangle.wasPressed();
    }

    /**
     * Checks if triangle was released since the last call of this method
     * @return true if triangle was released since the last call of this method; otherwise false
     */
    public boolean triangleWasReleased() {
        return changes.triangle.wasReleased();
    }

    /**
     * Checks if square was pressed since the last call of this method
     * @return true if square was pressed since the last call of this method; otherwise false
     */
    public boolean squareWasPressed() {
        return changes.square.wasPressed();
    }

    /**
     * Checks if square was released since the last call of this method
     * @return true if square was released since the last call of this method; otherwise false
     */
    public boolean squareWasReleased() {
        return changes.square.wasReleased();
    }

    /**
     * Checks if share was pressed since the last call of this method
     * @return true if share was pressed since the last call of this method; otherwise false
     */
    public boolean shareWasPressed() {
        return changes.share.wasPressed();
    }

    /**
     * Checks if share was released since the last call of this method
     * @return true if share was released since the last call of this method; otherwise false
     */
    public boolean shareWasReleased() {
        return changes.share.wasReleased();
    }

    /**
     * Checks if options was pressed since the last call of this method
     * @return true if options was pressed since the last call of this method; otherwise false
     */
    public boolean optionsWasPressed() {
        return changes.options.wasPressed();
    }

    /**
     * Checks if options was released since the last call of this method
     * @return true if options was released since the last call of this method; otherwise false
     */
    public boolean optionsWasReleased() {
        return changes.options.wasReleased();
    }

    /**
     * Checks if touchpad was pressed since the last call of this method
     * @return true if touchpad was pressed since the last call of this method; otherwise false
     */
    public boolean touchpadWasPressed() {
        return changes.touchpad.wasPressed();
    }

    /**
     * Checks if touchpad was released since the last call of this method
     * @return true if touchpad was released since the last call of this method; otherwise false
     */
    public boolean touchpadWasReleased() {
        return changes.touchpad.wasReleased();
    }

    /**
     * Checks if ps was pressed since the last call of this method
     * @return true if ps was pressed since the last call of this method; otherwise false
     */
    public boolean psWasPressed() {
        return changes.ps.wasPressed();
    }

    /**
     * Checks if ps was released since the last call of this method
     * @return true if ps was released since the last call of this method; otherwise false
     */
    public boolean psWasReleased() {
        return changes.ps.wasReleased();
    }


}
