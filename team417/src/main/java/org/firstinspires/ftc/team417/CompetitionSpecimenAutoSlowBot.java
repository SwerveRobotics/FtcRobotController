package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

@Autonomous (name = "AutoSpecimen", group = "SlowBot", preselectTeleOp = "TeleOp")
public class CompetitionSpecimenAutoSlowBot extends BaseOpModeSlowBot {
    @Override
    public void runOpMode () {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), Math.toRadians(-90));  // sets the beginning pose relative to the robot and  cxc
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);
        initializeHardware();
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                // after disp arm up
                .splineToLinearHeading(new Pose2d(0, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN, LIFT_SCORE_HIGH_SPECIMEN))
                .splineToLinearHeading(new Pose2d(0,XDRIVE_Y_SCORE_POSE - 3, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36,24,Math.toRadians(-90)),Math.toRadians(-90))   // bring sample to obs zone
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-48,12,Math.toRadians(-90)),Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48,57, Math.toRadians(-90)), Math.toRadians(90))
                .setTangent(Math.toRadians(-90))

                .splineToLinearHeading(new Pose2d(-60,12, Math.toRadians(-90)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60,60,Math.toRadians(-90)),Math.toRadians(90))  // finish bringing to obs zone
                .setTangent(Math.toRadians(-90))

                .splineToLinearHeading(new Pose2d(-49,55,Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(10))
                .stopAndAdd(new SleepAction(1))
                .splineToLinearHeading(new Pose2d(-49,65,Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(25))
                .afterDisp(0,new LiftSpecimenAction()) //after intaking lift 4 bar up

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-49,63,Math.toRadians(90)),Math.toRadians(90),new TranslationalVelConstraint(5))
                .setTangent(Math.toRadians(-90))
                // after disp arm up
                .splineToLinearHeading(new Pose2d(-3, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN, LIFT_SCORE_HIGH_SPECIMEN))
                .splineToLinearHeading(new Pose2d(-3,XDRIVE_Y_SCORE_POSE - 3, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
                //.setTangent(Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))



//                //four bar out
//                //intake sample
//                //four bar in
//                .splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
//                //output sample
//                .splineToLinearHeading(new Pose2d(-49,68-(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-49,72-(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
//                //.setTangent(Math.toRadians(180))
//                //.splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))
//                //four bar out
//                //intake sample
//                //four bar in
//                //.splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
//                //after disp arm out
//                //output sample after disp 0.5 seconds
//                .setTangent(Math.toRadians(-90))
//                // after disp arm up
//                .splineToLinearHeading(new Pose2d(-3, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(-90)), Math.toRadians(180))
//                //four bar out
//                //intake sample
//                //four bar in
//                .splineToLinearHeading(new Pose2d(-59,68-(ROBOT_WIDTH/2),Math.toRadians(180 )), Math.toRadians(180))
//                //after disp arm out
//                //output sample after disp 0.5 seconds
//                .splineToLinearHeading(new Pose2d(-49,72 -(ROBOT_WIDTH/2),Math.toRadians(90 )), Math.toRadians(180))
//                .setTangent(Math.toRadians(-90))
//                // after disp arm up
//                .splineToLinearHeading(new Pose2d(-6, XDRIVE_Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-49,72 -(ROBOT_WIDTH/2),Math.toRadians(-90 )), Math.toRadians(90))
                .build();

        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);


        // Show the preview on FTC Dashboard now.

        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);

        while (!isStarted()) {
            if (getLiftPosition() < LIFT_TICKS_EPSILON) {
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
                telemetry.addLine("<big><big><font color='red'>WARNING! WARNING! LIFT POSITION NOT CALIBRATED ");
                telemetry.update();

            }
            else {
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
                telemetry.addLine("good to go");
                telemetry.update();
            }

        }
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
        stopCrash();
    }
}
