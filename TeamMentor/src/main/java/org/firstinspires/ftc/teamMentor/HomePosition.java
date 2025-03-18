package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode to be run for the inspection fitting box.
 */
@TeleOp (name = "Home Position", group = "MentorBot")
public class HomePosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap, telemetry);
        arm.sloMo = true;

        telemetry.addLine("The Arm will move into its home position once 'Start' is pressed\n");
        telemetry.addLine(String.format("Current shoulder servo position: %.2f", arm.joints[Id.SHOULDER].servos[0].getPosition()));
        telemetry.update();
        waitForStart();

        // Run the inspection fitting box
        while (opModeIsActive()) {
            telemetry.addLine("Press stop once in position.");
            telemetry.update();

            // Move the arm to its start:
            arm.start();
            TelemetryPacket packet = new TelemetryPacket();
            arm.update(new Pose2d(0, 0, 0), packet.fieldOverlay()); // Update arm position
        }
    }
}
