package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous(name = "AutoSpecimen", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionSpecimenAuto extends BaseOpMode {
    @Override

    public void runOpMode() {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        RobotAction foldOutArm = new MoveArm(ARM_SCORE_SPECIMEN, WRIST_FOLDED_IN);
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .afterDisp(0, foldOutArm)
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2), Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new WaitAction(foldOutArm))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60,60,Math.toRadians(-90)),Math.toRadians(90))
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
