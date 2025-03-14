package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * Wily Works Gamepad implementation that takes input either from a connected gamepad or
 * from the keyboard.
 */
public class WilyGamepad {

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

}
