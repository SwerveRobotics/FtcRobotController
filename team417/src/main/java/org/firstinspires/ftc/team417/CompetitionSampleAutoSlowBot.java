package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

@Autonomous (name = "AutoSample", group = "SlowBot", preselectTeleOp = "TeleOp")
public class CompetitionSampleAutoSlowBot extends BaseOpModeSlowBot {
    @Override
    public void runOpMode () {
        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) + 48 , 72 - (ROBOT_WIDTH / 2), Math.toRadians(0));  // sets the beginning pose relative to the robot and  cxc
        initializeHardware(beginPose);

        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-45))

                // Splines to face the basket
                .splineToLinearHeading(new Pose2d(basketPositionX, basketPositionY, basketHeading), basketHeading, new TranslationalVelConstraint(sampleAutoSpeedConst))
                // Lift to basket
                .stopAndAdd(new ControlAction(SLIDE_SCORE_IN_BASKET,WRIST_IN, LIFT_SCORE_HIGH_BASKET))
                .stopAndAdd(new ControlAction(SLIDE_SCORE_IN_BASKET,WRIST_SCORE, LIFT_SCORE_HIGH_BASKET))
                // Deposit then off
                .stopAndAdd(new IntakeAction(INTAKE_DEPOSIT))
                .stopAndAdd(new SleepAction(2))
                .stopAndAdd(new IntakeAction(INTAKE_OFF))
                // Lower lift
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
                .setTangent(Math.toRadians(180))

                // Splines to face first floor sample
                .splineToLinearHeading(new Pose2d(48,58, Math.toRadians(-90)), Math.toRadians(180), new TranslationalVelConstraint(25))
                // Slides extend out and wrist folds out
                .stopAndAdd(new ControlAction(SLIDE_COLLECT, WRIST_IN, LIFT_HOME_POSITION))
                .stopAndAdd(new ControlAction(SLIDE_COLLECT, WRIST_OUT, LIFT_HOME_POSITION))
                // Turn on intake for 2 seconds then turn off
                .stopAndAdd(new IntakeAction(INTAKE_COLLECT))
                .stopAndAdd(new SleepAction(2))
                .stopAndAdd(new IntakeAction(INTAKE_OFF))
                // Pull in slides and fold in the wrist
                .stopAndAdd(new ControlAction(SLIDE_COLLECT, WRIST_IN, LIFT_HOME_POSITION))
                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION, WRIST_IN, LIFT_HOME_POSITION))
//
//                // Splines to face the basket
//                .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45), new TranslationalVelConstraint(25))
//                // Lift to basket
//                .stopAndAdd(new ControlAction(SLIDE_SCORE_IN_BASKET,WRIST_OUT, LIFT_SCORE_HIGH_BASKET))
//                // Deposit then off
//                .stopAndAdd(new IntakeAction(INTAKE_DEPOSIT))
//                .stopAndAdd(new SleepAction(2))
//                .stopAndAdd(new IntakeAction(INTAKE_OFF))
//                // Lower lift
//                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
//                .setTangent(Math.toRadians(0))
//
//                // Spline to second floor sample
//                .splineToLinearHeading(new Pose2d(59,50,Math.toRadians(-90)), Math.toRadians(-90), new TranslationalVelConstraint(25))
//                // Slides extend out and wrist folds out
//                .stopAndAdd(new ControlAction(SLIDE_COLLECT, WRIST_OUT, LIFT_HOME_POSITION))
//                // Turn on intake for 2 seconds then turn off
//                .stopAndAdd(new IntakeAction(INTAKE_COLLECT))
//                .stopAndAdd(new SleepAction(2))
//                .stopAndAdd(new IntakeAction(INTAKE_OFF))
//                // Pull in slides and fold in the wrist
//                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION, WRIST_IN, LIFT_HOME_POSITION))
//
//                // Splines to face the basket
//                .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45), new TranslationalVelConstraint(25))
//                // Lift to basket
//                .stopAndAdd(new ControlAction(SLIDE_SCORE_IN_BASKET,WRIST_OUT, LIFT_SCORE_HIGH_BASKET))
//                // Deposit then off
//                .stopAndAdd(new IntakeAction(INTAKE_DEPOSIT))
//                .stopAndAdd(new SleepAction(2))
//                .stopAndAdd(new IntakeAction(INTAKE_OFF))
//                // Lower lift
//                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
//                .setTangent(Math.toRadians(0))
//
//                // Spline to third floor sample
//                .splineToLinearHeading(new Pose2d(46,26,Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(25))
//                // Slides extend out and wrist folds out
//                .stopAndAdd(new ControlAction(SLIDE_COLLECT, WRIST_OUT, LIFT_HOME_POSITION))
//                // Turn on intake for 2 seconds then turn off
//                .stopAndAdd(new IntakeAction(INTAKE_COLLECT))
//                .stopAndAdd(new SleepAction(2))
//                .stopAndAdd(new IntakeAction(INTAKE_OFF))
//                // Pull in slides and fold in the wrist
//                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION, WRIST_IN, LIFT_HOME_POSITION))
//
//                // Splines to face the basket
//                .splineToLinearHeading(new Pose2d(50,50, Math.toRadians(45)), Math.toRadians(45), new TranslationalVelConstraint(25))
//                // Lift to basket
//                .stopAndAdd(new ControlAction(SLIDE_SCORE_IN_BASKET,WRIST_OUT, LIFT_SCORE_HIGH_BASKET))
//                // Deposit then off
//                .stopAndAdd(new IntakeAction(INTAKE_DEPOSIT))
//                .stopAndAdd(new SleepAction(2))
//                .stopAndAdd(new IntakeAction(INTAKE_OFF))
//                // Lower lift
//                .stopAndAdd(new ControlAction(SLIDE_HOME_POSITION,WRIST_IN,LIFT_HOME_POSITION))
//                .setTangent(Math.toRadians(0))

                .build();

        Canvas previewCanvas = new Canvas();
        trajectoryAction.preview(previewCanvas);

        // Show the preview on FTC Dashboard now.
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
        MecanumDrive.sendTelemetryPacket(packet);
        drive.distanceLocalizer.enabled = false;
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
        stopCrash();
    }
}
