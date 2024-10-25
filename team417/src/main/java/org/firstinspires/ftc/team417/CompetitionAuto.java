package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name = "Auto", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionAuto extends BaseOpMode {

    // RC 17.50
    // DEV 17.75
    final double ROBOT_LENGTH = 17.50;
    // RC 16.50
    // DEV 18.50
    final double ROBOT_WIDTH = 16.50;

    // This class contains the function to lift the arm
    class MoveArm extends RobotAction{

        double targetPosition;
        MoveArm (double targetPosition) {
            this.targetPosition = targetPosition;
        }
        //ARM_SCORE_SAMPLE_IN_LOW

        @Override
        public boolean run(double elapsedTime) {
            double error = Math.abs(armMotor.getTargetPosition() - targetPosition);
            final double EPSILON = 3.00;

            // Prevents hanging from running on wily works
            if(WilyWorks.isSimulating){
                return false;
            }
            if (error < EPSILON)
                // Arm is within range so arm stops
                return false;
            // Arm isn't withjn range so we keep calling
            armMotor.setTargetPosition((int) (targetPosition));
            wrist.setPosition(WRIST_FOLDED_OUT);
            return true;
        }
    }

    class ScoreSample extends RobotAction{
        @Override
        public boolean run(double elapsedTime) {
            // Keep the intake deposit on until the 2 seconds are over
            if(elapsedTime <= 2) {
                intake.setPower(INTAKE_DEPOSIT);
                return true;
            }
            // Turn off deposit after 2 seconds and then end action
            intake.setPower(INTAKE_OFF);
            return false;
        }
    }

    @Override
    public void runOpMode() {

        // BeginPose is the 2nd tile away from the basket, facing the basket, lined up with the tile boundary
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / 2) + 24, 72 - (ROBOT_WIDTH / 2), 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        Action trajectoryAction;

        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!

        // Position relative to basket, far is away from the basket vice versa
        boolean isFar;

        trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(50, 50, Math.toRadians(45)), Math.toRadians(-45))
                .stopAndAdd(new MoveArm(ARM_SCORE_SAMPLE_IN_LOW))
                .stopAndAdd(new ScoreSample())
                .stopAndAdd(new MoveArm(ARM_CLEAR_BARRIER))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48,12, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(28,12, Math.toRadians(180)), Math.toRadians(180))
                .build();


        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);


        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);


        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();


        boolean more = true;
        while (opModeIsActive() && more) {
            telemetry.addLine("Running Auto!");
            telemetry.addData("Kinematic Type", kinematicType);


            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();


            // Draw the preview and then run the next step of the trajectory on top:
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            more = trajectoryAction.run(packet);


            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}

