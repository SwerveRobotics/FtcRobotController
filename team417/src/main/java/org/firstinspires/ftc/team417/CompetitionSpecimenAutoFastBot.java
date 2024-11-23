package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@Autonomous(name = "AutoSpecimen", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class CompetitionSpecimenAutoFastBot extends BaseOpMode {
    @Override

    public void runOpMode() {
        // Signal initializeHardware() to remake the armMotor object:

        armPosition = 0;

        Pose2d beginPose = new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        initFastBot();
        RobotAction foldOutArm = new MoveArm(ARM_SCORE_SPECIMEN, WRIST_SCORE_SPECIMEN);
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(-90))
                .afterDisp(0, foldOutArm)
                .splineToLinearHeading(new Pose2d(7, Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))  // goes up to the specimen high bar
                .stopAndAdd(new WaitAction(foldOutArm))
                .stopAndAdd(new MoveArm(ARM_SCORE_SPECIMEN+20*ARM_TICKS_PER_DEGREE, WRIST_SCORE_SPECIMEN))       //scores the specimen with slight downward force
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(7, Y_SCORE_POSE+6, Math.toRadians(-90)), Math.toRadians(90),new TranslationalVelConstraint(20))
                .stopAndAdd(new ScoreSample())
                .setTangent(Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(-39.5,24,Math.toRadians(-90)),Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-48,12,Math.toRadians(-90)),Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-62,62,Math.toRadians(-90)),Math.toRadians(90))

                // go collect sample to obs zone

                .setTangent(Math.toRadians(-90))
                .afterDisp(15 ,new MoveArm((255.5*ARM_TICKS_PER_DEGREE),WRIST_FOLDED_OUT))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), Math.toRadians(180)),Math.toRadians(90))
                .setTangent(Math.toRadians(180))


                .afterDisp(0, new RunIntake(INTAKE_COLLECT))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2) -30, 72 -(ROBOT_WIDTH / 2),Math.toRadians(180)), Math.toRadians(180),new TranslationalVelConstraint(20))
                .setTangent(Math.toRadians(0))
                .afterDisp(0,foldOutArm)
                .afterDisp(4,new RunIntake(INTAKE_OFF))

                .splineToLinearHeading(new Pose2d(0, Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new WaitAction(foldOutArm))
                .stopAndAdd(new MoveArm(ARM_SCORE_SPECIMEN+20*ARM_TICKS_PER_DEGREE, WRIST_SCORE_SPECIMEN))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, Y_SCORE_POSE+6, Math.toRadians(-90)), Math.toRadians(90),new TranslationalVelConstraint(20))
                .stopAndAdd(new ScoreSample())
                .setTangent(Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .afterDisp(15 ,new MoveArm((255.5*ARM_TICKS_PER_DEGREE),WRIST_FOLDED_OUT))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2) , 72 - (ROBOT_WIDTH / 2), Math.toRadians(180)),Math.toRadians(90))
                .setTangent(Math.toRadians(180))


                .afterDisp(0, new RunIntake(INTAKE_COLLECT))
                .splineToLinearHeading(new Pose2d((ROBOT_LENGTH / -2) -30, 72 -(ROBOT_WIDTH / 2),Math.toRadians(180)), Math.toRadians(180),new TranslationalVelConstraint(20))
                .setTangent(Math.toRadians(0))
                .afterDisp(0,foldOutArm)
                .afterDisp(4,new RunIntake(INTAKE_OFF))

                .splineToLinearHeading(new Pose2d(-7, Y_SCORE_POSE, Math.toRadians(-90)), Math.toRadians(-90))
                .stopAndAdd(new WaitAction(foldOutArm))
                .stopAndAdd(new MoveArm(ARM_SCORE_SPECIMEN+20*ARM_TICKS_PER_DEGREE, WRIST_SCORE_SPECIMEN))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-7, Y_SCORE_POSE+6, Math.toRadians(-90)), Math.toRadians(90),new TranslationalVelConstraint(20))
                .stopAndAdd(new ScoreSample())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60,60,Math.toRadians(-90)),Math.toRadians(90),new TranslationalVelConstraint(60))
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
