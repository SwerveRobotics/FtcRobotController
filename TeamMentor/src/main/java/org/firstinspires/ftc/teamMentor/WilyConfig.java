package org.firstinspires.ftc.teamMentor;

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
        // Impersonate this robot when running the simulator:
        deviceName = "99866-RC";

        // Use these dimensions for the robot:
        robotWidth = Specs.Robot.MENTORBOT_WIDTH; // Inches
        robotLength = Specs.Robot.MENTORBOT_LENGTH; // Inches

        // Register where the LED indicators are on the robot:
        ledIndicators = new LEDIndicator[]{
            new LEDIndicator("redLed", true, 0, 0),
            new LEDIndicator("greenLed", false, 0, 0),
        };

        // Register where the distance sensors are on the robot:
        distanceSensors = new DistanceSensor[]{
            new DistanceSensor(Poser.ultrasonics[0].name, Poser.ultrasonics[0].x, Poser.ultrasonics[0].y, Poser.ultrasonics[0].orientation),
            new DistanceSensor(Poser.ultrasonics[1].name, Poser.ultrasonics[1].x, Poser.ultrasonics[1].y, Poser.ultrasonics[1].orientation),
        };
    }
}
