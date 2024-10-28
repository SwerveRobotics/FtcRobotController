package org.firstinspires.ftc.team417.distance;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DistancePID", group = "Concept")
@Config
public class DistancePIDConcept extends DistanceSensorConcept {
    UltrasonicDistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");


    }
}
