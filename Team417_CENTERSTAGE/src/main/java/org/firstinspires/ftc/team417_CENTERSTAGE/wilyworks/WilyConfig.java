package org.firstinspires.ftc.team417_CENTERSTAGE.wilyworks;

import com.wilyworks.common.Wily;
import com.wilyworks.common.WilyWorks;

/**
 * Set the configuration for Wily Works.
 */
@Wily
public class WilyConfig extends WilyWorks.Config {
    WilyConfig() {
        robotWidth = 16;
        robotLength = 18;
        cameras = new Camera[]{
                new Camera("webcamback", -5.75, -6, Math.PI, Math.toRadians(75), 0),
                new Camera("webcamfront", 7, -0.5, 0, Math.toRadians(70.4), 0.190)
        };
        distanceSensors = new DistanceSensor[]{
                new DistanceSensor("distance", -1, 3.5, Math.PI)
        };
    }
}
