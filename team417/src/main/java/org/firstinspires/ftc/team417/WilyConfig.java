package org.firstinspires.ftc.team417;

import com.wilyworks.common.Wily;
import com.wilyworks.common.WilyWorks;

/**
 * This customizes the configuration of the Wily Works simulator to match your robot.
 *
 * @noinspection unused
 */
@Wily
public class WilyConfig extends WilyWorks.Config {
    WilyConfig() {
        // Impersonate the DevBot when running the simulator:
        deviceName = "DevBot";

        // Use these dimensions for the robot:
        robotWidth = 18.0;
        robotLength = 18.0;

        distanceSensors = new DistanceSensor[]{
                new DistanceSensor("leftSonic", 7.75, 6.75, Math.toRadians(45)),
                new DistanceSensor("rightSonic", 7.75, -6.75, Math.toRadians(-45)) // Not the real DevBot coordinates
        };

        positionError = 1.0; // Percentage error as a function of distance traveled
        headingError = 2.0; // Degrees of drift per minute
        distanceSensorError = 0.5; // Range of error for distance sensors, in inches
    }
}
