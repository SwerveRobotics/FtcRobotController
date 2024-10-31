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
@Autonomous(name = "AutoBasket", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionBasketAuto extends BaseOpMode {



    // This class contains the function to lift the arm




    @Override
    public void runOpMode() {

        // BeginPose is the 2nd tile away from the basket, facing the basket, lined up with the tile boundary
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / 2) + 24, 72 - (ROBOT_WIDTH / 2), 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();


        // Build the trajectory *before* the start button is pressed because Road Runner
        // can take multiple seconds for this operation. We wouldn't want to have to wait
        // as soon as the Start button is pressed!


        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(50, 50, Math.toRadians(45)), Math.toRadians(-45))
                .afterDisp(0, new MoveArm(ARM_COLLAPSED_INTO_ROBOT, WRIST_FOLDED_IN))
                .stopAndAdd(new MoveArm(ARM_SCORE_SAMPLE_IN_LOW, WRIST_FOLDED_OUT))
                .stopAndAdd(new ScoreSample())


                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48, 12, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(28, 12, Math.toRadians(180)), Math.toRadians(180))
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

