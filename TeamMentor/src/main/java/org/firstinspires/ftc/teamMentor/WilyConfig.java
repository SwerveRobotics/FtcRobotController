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
            new DistanceSensor(Poser.mentorBotUltrasonics[0].name, Poser.mentorBotUltrasonics[0].x, Poser.mentorBotUltrasonics[0].y, Poser.mentorBotUltrasonics[0].orientation),
            new DistanceSensor(Poser.mentorBotUltrasonics[1].name, Poser.mentorBotUltrasonics[1].x, Poser.mentorBotUltrasonics[1].y, Poser.mentorBotUltrasonics[1].orientation),
        };
    }
}
