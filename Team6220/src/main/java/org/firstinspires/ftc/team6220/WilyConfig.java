package org.firstinspires.ftc.team6220;

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
        deviceName = "6220-D-RC";

        // Use these dimensions for the robot:
        robotWidth = 17.0;
        robotLength = 17.0;
    }
}
