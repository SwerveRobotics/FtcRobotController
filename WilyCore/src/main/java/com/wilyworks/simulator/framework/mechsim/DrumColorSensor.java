package com.wilyworks.simulator.framework.mechsim;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.wilyworks.simulator.framework.WilyNormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.awt.Color;

// Hooked class for determining the color of the ball once it's in the drum:
public class DrumColorSensor extends WilyNormalizedColorSensor {
    DecodeSlowBotMechSim mechSim;
    int idMask; // Sensor 0 or 1

    DrumColorSensor(DecodeSlowBotMechSim mechSim, int index) {
        this.mechSim = mechSim;
        this.idMask = 1 << index;
    }

    // Returns true if this sensor can read a ball; false if a hole in the ball is positioned
    // over the sensor.
    boolean sensorCanReadBall() {
        // Every time we get a new ball, reset our variations:
        if (mechSim.colorSensorMask == -1) {
            mechSim.colorSensorMask = 1 + (int) (Math.random() * 3.0); // Mask = 1, 2 or 3
        }
        return ((mechSim.colorSensorMask & idMask) != 0);
    }

    @Override
    public NormalizedRGBA getNormalizedColors() {
        NormalizedRGBA normalizedColor = new NormalizedRGBA();

        // Simulate the ball holes for some reads:
        int rgbColor = 0;
        // Figure out what slot is being input into, if any:
        int slot = mechSim.findDrumSlot(mechSim.INTAKE_POSITIONS);
        if (slot != -1) {
            DecodeSlowBotMechSim.Ball ball = mechSim.slotBalls.get(slot);
            if (ball != null) {
                // There's ball over the sensors. See if they can be read:
                if (sensorCanReadBall()) {
                    if (ball.color == DecodeSlowBotMechSim.BallColor.GREEN) {
                        rgbColor = android.graphics.Color.HSVToColor(new float[]{175, 1, 1});
                    } else {
                        rgbColor = android.graphics.Color.HSVToColor(new float[]{210, 1, 1});
                    }
                }
            }
        }
        normalizedColor.red = new Color(rgbColor).getRed() / 255.0f;
        normalizedColor.green = new Color(rgbColor).getGreen() / 255.0f;
        normalizedColor.blue = new Color(rgbColor).getBlue() / 255.0f;
        return normalizedColor;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        int slot = mechSim.findDrumSlot(mechSim.INTAKE_POSITIONS);
        boolean ballPositionedForRead = (slot != -1) && mechSim.slotBalls.get(slot) != null;
        return ballPositionedForRead && sensorCanReadBall() ? unit.fromMm(18) : unit.fromMm(70);
    }
}
