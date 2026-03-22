package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Preload", group = "Auto")
public class Preload extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PixelColor[] preloads = { PixelColor.PURPLE, PixelColor.GREEN, PixelColor.PURPLE };
        ComplexMechGlob mechGlob = new ComplexMechGlob(hardwareMap, telemetry, preloads);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addLine("Ready to start preload.\n");
        telemetry.addLine("WARNING: The drum will move suddenly when Start is pressed!!");
        telemetry.update();
        waitForStart();

        final double DRUM_VELOCITY = 0.5; // Speed to move the servo drum, position units per second

        int currentSlot = 0; // Currently active slot
        int targetSlot = 0; // Target slot the drum is moving to
        double currentDrumPosition = mechGlob.LAUNCH_POSITIONS[0]; // Set the drum's current position to slot 0
        ElapsedTime loopTime = new ElapsedTime();
        ElapsedTime timeSinceTransfer = new ElapsedTime();

        while (!isStopRequested()) {
            if (currentSlot != targetSlot) {
                telemetry.addLine(String.format("<big><big>Moving to slot #%d</big></big>\n", targetSlot + 1));
                telemetry.addLine("Hold down the bumper button until movement is complete.");
            } else {
                String color = preloads[currentSlot] == PixelColor.PURPLE ? "#800080" : "#008000";
                String name = preloads[currentSlot] == PixelColor.PURPLE ? "Purple" : "Green";
                telemetry.addLine(String.format("<big><big>Slot #%d</big></big>", currentSlot + 1));
                telemetry.addLine(String.format("<span style='background: %s'>Make it %s</span>\n", color, name));
                telemetry.addLine("Hold A to intake a ball through the launcher");
                telemetry.addLine("Hold B to eject a ball");
                telemetry.addLine("Right shoulder button to advance the drum");
            }

            double transferPosition = ComplexMechGlob.TRANSFER_INACTIVE_POSITION;
            double feederPower = 0;

            if (gamepad1.rightBumperWasPressed()) {
                targetSlot = (currentSlot + 1) % 3;
            }

            // Advance the drum position:
            double targetDrumPosition = mechGlob.LAUNCH_POSITIONS[targetSlot];
            double deltaPosition = targetDrumPosition - currentDrumPosition;
            if (deltaPosition != 0) {
                // If a bumper is pressed, and it's been sufficient time since the transfer
                // mechanism has activated, then move the drum:
                if ((gamepad1.right_bumper) && (timeSinceTransfer.seconds() > ComplexMechGlob.TRANSFER_TIME_DOWN)) {
                    currentDrumPosition += Math.signum(deltaPosition)
                            * Math.min(DRUM_VELOCITY * loopTime.seconds(), Math.abs(deltaPosition));
                }
            } else {
                // If we're here, we made it to the target slot:
                currentSlot = targetSlot;

                // Only allow intaking/out-taking when not moving the drum:
                if (gamepad1.a) {
                    // Intake a ball:
                } else if (gamepad1.b) {
                    // Eject a ball (without the flywheel!):
                    transferPosition = ComplexMechGlob.TRANSFER_ACTIVE_POSITION;
                    timeSinceTransfer = new ElapsedTime(); // Start the transfer timer
                }
            }

            // Update the loop time:
            loopTime = new ElapsedTime();

            // Feed the servos:
            mechGlob.servoTransfer.setPosition(transferPosition);
            mechGlob.servoDrum.setPosition(currentDrumPosition);

            // Finish up:
            telemetry.update();
            WilyWorks.updateSimulation(0);
        }
    }
}
