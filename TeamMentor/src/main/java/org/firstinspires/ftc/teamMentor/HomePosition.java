package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * OpMode to be run for the inspection fitting box.
 */
@TeleOp (name = "Home Position", group = "MentorBot")
public class HomePosition extends LinearOpMode {
    @Override
    public void runOpMode() {

        telemetry.addLine("The Arm will move into its home position once 'Start' is pressed\n");
        telemetry.update();
        waitForStart();

        Arm arm = new Arm(hardwareMap, telemetry);
        arm.sloMo = true;

        // Run the inspection fitting box
        while (opModeIsActive()) {
            telemetry.addLine("Press stop once in position.");
            telemetry.update();
        }
    }
}
