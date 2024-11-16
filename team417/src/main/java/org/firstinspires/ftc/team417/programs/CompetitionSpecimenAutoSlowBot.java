package org.firstinspires.ftc.team417.programs;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

// @Autonomous (name = "AutoSpecimenXDrive", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionSpecimenAutoSlowBot extends BaseOpModeSlowBot {
    public CompetitionSpecimenAutoSlowBot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void runOpMode () {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), Math.toRadians(-90));  // sets the beginning pose relative to the robot and  cxc
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                // after disp arm up
                .splineToLinearHeading(new Pose2d(0, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                // stop and wait arm down SCORES
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))
                //four bar out
                //intake sample
                //four bar in
                .splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
                //output sample
                .splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49,72-(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
                //.setTangent(Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))
                //four bar out
                //intake sample
                //four bar in
                //.splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
                //after disp arm out
                //output sample after disp 0.5 seconds
                .setTangent(Math.toRadians(-90))
                // after disp arm up
                .splineToLinearHeading(new Pose2d(-3, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))
                //four bar out
                //intake sample
                //four bar in
                .splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
                //after disp arm out
                //output sample after disp 0.5 seconds
                .splineToLinearHeading(new Pose2d(-49,72 -(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
                .setTangent(Math.toRadians(-90))
                // after disp arm up
                .splineToLinearHeading(new Pose2d(-6, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-49,72 -(ROBOT_WIDTH/2),Math.toRadians(-90 )), Math.toRadians(90))
                .build();
        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);


        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);
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
