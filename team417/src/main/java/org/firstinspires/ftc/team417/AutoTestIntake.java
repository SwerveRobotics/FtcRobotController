package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous(name = "AutoIntake", group = "FastBot", preselectTeleOp = "CompetitionTeleOp")
public class AutoTestIntake extends BaseOpModeFastBot {
    @Override

    public void runOpMode() {
        // Signal initializeHardware() to remake the armMotor object:

        armPosition = 0;

        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2), 72 - (ROBOT_WIDTH / 2), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initFastBot();
        RobotAction foldOutArm = new MoveArm(ARM_SCORE_SPECIMEN, WRIST_SCORE_SPECIMEN);
        Action trajectoryAction = drive.actionBuilder(beginPose)

                .stopAndAdd(new MoveArm(ARM_COLLECT, WRIST_FOLDED_OUT))
                .setTangent(Math.toRadians(180))
                .afterDisp(0, new RunIntake(INTAKE_COLLECT))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2) - 26, 72 - (ROBOT_WIDTH / 2), Math.toRadians(180)), Math.toRadians(180), new TranslationalVelConstraint(10))
                .setTangent(Math.toRadians(0))
                .afterDisp(0, foldOutArm)
                

                .splineToLinearHeading(new Pose2d(0, Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new WaitAction(foldOutArm))
                .stopAndAdd(new MoveArm(ARM_SCORE_SPECIMEN + 20 * ARM_TICKS_PER_DEGREE, WRIST_SCORE_SPECIMEN))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, Y_SCORE_POSE + 8 , Math.toRadians(-90)), Math.toRadians(90), new TranslationalVelConstraint(20))
                .stopAndAdd(new ScoreSample())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(beginPose, Math.toRadians(90))
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

            // Draw the uncorrected pose in green if it's correcting and red if it's not:
            if (drive.distanceLocalizer != null) {
                Pose2d oldPose = new Pose2d(
                        drive.pose.position.x - drive.distanceLocalizer.correction.x,
                        drive.pose.position.y - drive.distanceLocalizer.correction.y,
                        drive.pose.heading.log()
                );
                if (drive.distanceLocalizer.correcting) {
                    packet.fieldOverlay().setStroke("#00FF00");
                } else {
                    packet.fieldOverlay().setStroke("#FF0000");
                }
                packet.fieldOverlay().strokeLine(
                        oldPose.position.x,
                        oldPose.position.y,
                        drive.pose.position.x,
                        drive.pose.position.y
                );
                Drawing.drawRobot(packet.fieldOverlay(), oldPose);
            }

            // Only send the packet if there's more to do in order to keep the very last
            // drawing up on the field once the robot is done:
            if (more)
                MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }


}

