package org.firstinspires.ftc.team417_CENTERSTAGE.benchmarking;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagPoseEstimator;
import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;

@Autonomous(name="Localization Benchmark")
public class LocalizationBenchmark extends BaseOpMode {
    private final boolean USE_APRIL_TAGS = true;

    public AprilTagPoseEstimator myATPoseEstimator;

    public void initBenchmarking() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initializeHardware();

        if (USE_APRIL_TAGS) {
            myATPoseEstimator = new AprilTagPoseEstimator(hardwareMap, telemetry);

            // Pass an April Tag Helper object so drive can add twists to it (See MecanumDrive for
            //     updatePoseEstimate() an explanation on twists)
            drive.setATLHelper(myATPoseEstimator.myAprilTagLatencyHelper);
        }

        telemetry.addData("Init State", "Init Finished");

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    class ATContinuallyEstimatePoseAction implements Action {
        public boolean run(TelemetryPacket packet) {
            if (!opModeIsActive()) return false;
            myATPoseEstimator.updatePoseEstimate();
            return true;
        }
    }

    @Override
    public void runOpMode() {
        initBenchmarking();

        TrajectoryActionBuilder benchmarkTo = drive.actionBuilder(new Pose2d(-48.00, 48.00, Math.toRadians(135.00)))
                .splineTo(new Vector2d(-57.00, 24.00), Math.toRadians(270.00))
                .splineTo(new Vector2d(-57.00, -24.00), Math.toRadians(270.00))
                .splineTo(new Vector2d(-51.00, -51.00), Math.toRadians(315.00))
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.00, -60.00), Math.toRadians(180))
                .splineTo(new Vector2d(48.00, -48.00), Math.toRadians(180));

        TrajectoryActionBuilder benchmarkBack = drive.actionBuilder(new Pose2d(48.00, -48.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(24.00, -60.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-24.00, -60.00), Math.toRadians(180.00))
                .splineTo(new Vector2d(-51.00, -51.00), Math.toRadians(135.00))
                .splineTo(new Vector2d(-57.00, -24.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(-57.00, 24.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(-48.00, 48.00), Math.toRadians(135.00));

        Action toAction = benchmarkTo.build();
        Action backAction = benchmarkBack.build();

        waitForStart();

        drive.pose = new Pose2d(-48.00, 48.00, Math.toRadians(135.00));

        if (USE_APRIL_TAGS) {
            drive.runParallel(new ATContinuallyEstimatePoseAction());
        }

        boolean going = true;

        while (opModeIsActive()) {
            String message;
            if (going) {
                message = "Going!";
                drive.runParallel(toAction);
            } else {
                message = "Back";
                drive.runParallel(backAction);
            }

            going = !going;

            while (opModeIsActive()) {
                telemetry.addLine(message);
                telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)",
                        drive.pose.position.x, drive.pose.position.y,
                        drive.pose.heading.log()));
                telemetry.addLine("Press A to continue:");
                telemetry.addLine("Or press X to reset position and continue:");
                if (gamepad1.x) {
                    if (going) {
                        drive.pose = new Pose2d(-48.00, 48.00, Math.toRadians(135.00));
                    } else {
                        drive.pose = new Pose2d(48.00, -48.00, Math.toRadians(180.00));
                    }
                    break;
                } else if (gamepad1.a) {
                    break;
                }
                drive.doActionsWork();
                telemetry.update();
            }
        }

        telemetry.addLine("Running closing procedure: ");
        telemetry.update();

        if (myATPoseEstimator != null) {
            myATPoseEstimator.visionPortal.close();
        }
    }
}
