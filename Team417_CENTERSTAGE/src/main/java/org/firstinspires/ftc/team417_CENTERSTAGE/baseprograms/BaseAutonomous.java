package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.System.nanoTime;

import android.graphics.PointF;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.tfod.Results;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.Pose;

import java.util.ArrayList;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final boolean USE_OPEN_CV_PROP_DETECTION = false;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 1;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    MecanumDrive drive;

    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initializeAuto() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        myColorDetection.init();
        initializeHardware();

        telemetry.addData("Init State", "Init Finished");

        // Allow the OpenCV to process
        sleep(500);

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public void runAuto(boolean red, boolean close) {
        OpenCvColorDetection.SideDetected result;

        if (red) {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            telemetry.addLine("Looking for red");
        } else {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
            telemetry.addLine("Looking for blue");
        }

        initializeAuto();

        waitForStart();

        AutonDriveFactory.SpikeMarks sawarResult;

        if (USE_OPEN_CV_PROP_DETECTION) {
            result = myColorDetection.detectTeamProp();
            if (result == OpenCvColorDetection.SideDetected.LEFT) {
                sawarResult = AutonDriveFactory.SpikeMarks.LEFT;
            } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
                sawarResult = AutonDriveFactory.SpikeMarks.CENTER;
            } else {
                sawarResult = AutonDriveFactory.SpikeMarks.RIGHT;
            }
        } else {
            PropDistanceResults distanceResult = new PropDistanceResults();
            PropDistanceFactory propDistance = new PropDistanceFactory(drive);
            Action sweepAction = distanceResult.sweepAction(drive, distSensor, distanceResult);
            PropDistanceFactory.PoseAndAction poseAndAction = propDistance.getDistanceAction(red, !close, sweepAction);

            // Do the robot movement for prop detection:
            drive.pose = poseAndAction.startPose;
            Actions.runBlocking(poseAndAction.action);

            if (distanceResult.result == PropDistanceResults.SpikeMarks.LEFT)
                sawarResult = AutonDriveFactory.SpikeMarks.LEFT;
            else if (distanceResult.result == PropDistanceResults.SpikeMarks.CENTER)
                sawarResult = AutonDriveFactory.SpikeMarks.CENTER;
            else
                sawarResult = AutonDriveFactory.SpikeMarks.RIGHT;

            telemetry.addData("sawarResult", sawarResult);
            telemetry.update();
            sleep(1000000000); //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        }

        // Close cameras to avoid errors
        myColorDetection.robotCamera.closeCameraDevice();

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red, !close, sawarResult, dropPixel());

        drive.pose = poseAndAction.startPose;
        // @@@ Actions.runBlocking(poseAndAction.action);
    }

    public Action dropPixel() {
        return new Action() {
            double startTime = 0;  // startTime value to compare to
            @Override
            public boolean run(TelemetryPacket packet) {
                if (startTime == 0) { // does this on first loop
                    intakeMotor.setPower(INTAKE_SPEED2);
                    startTime = nanoTime() * NANO_TO_SECONDS_MULTIPLIER;
                }
                // current time - start time has to be greater than the intake time for the motor to stop
                if(nanoTime() * NANO_TO_SECONDS_MULTIPLIER - startTime > INTAKE_TIME) {
                    intakeMotor.setPower(0);
                    startTime = 0; // reset for next run
                    return false;
                } else {
                    return true;
                }
            }
        };
    }
}

class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;
    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    /*
     * Call this routine from your robot's competition code to get the sequence to drive. You
     * can invoke it there by calling "Actions.runBlocking(driveAction);".
     */
    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    }

    class PoseAndAction {
        Action action;
        Pose2d startPose;

        PoseAndAction(Action action, Pose2d startPose) {
            this.action = action;
            this.startPose = startPose;
        }
    }
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake) {

        if (isFar) {
            xOffset = 0;
        } else {
            xOffset = 48;
        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }

        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if(intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -36)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-38, -34)), xForm(Math.toRadians(180) + (1e-6)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -33)), xForm(Math.toRadians(90)))
                // arm
                .splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(-55, -10)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));


        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        spikeRight = spikeRight.splineToSplineHeading(xForm(new Pose2d(-35, -32, Math.toRadians(0))), xForm(Math.toRadians(90)))
                // arm action
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(58, -10)), xForm(Math.toRadians(0)));

        if(location == SpikeMarks.LEFT) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else if(location == SpikeMarks.CENTER) {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -60, Math.toRadians(90))));
        }

    }


    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }


    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, true, SpikeMarks.LEFT, null).action;
    }
}

@Config
class PropDistanceResults {

    public static double propSpot1Angle = 116, propSpot2Angle = 145,
            propSpot3Angle = 190, noPropAngle = 220, doneAngle = 220;
    public static double maxDist = 28;
    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    }
    public SpikeMarks result = SpikeMarks.LEFT;

    public Action sweepAction(MecanumDrive drive, DistanceSensor distSensor, PropDistanceResults results) {
        return new Action() {
            private final ArrayList<PointF> propPos1 = new ArrayList<>();
            private final ArrayList<PointF> propPos2 = new ArrayList<>();
            private final ArrayList<PointF> propPos3 = new ArrayList<>();
            private double initAngle;
            private boolean inited = false;
            @Override
            public boolean run(TelemetryPacket packet) {
                Canvas canvas = packet.fieldOverlay();

                double distanceSensorReturn = distSensor.getDistance(DistanceUnit.INCH);
                double currentAngleRadians;

                if (!inited) {
                    initAngle = drive.pose.heading.log() + Math.PI;
                    inited = true;
                }

                currentAngleRadians = (drive.pose.heading.log() + Math.PI) - initAngle;
                if (currentAngleRadians < Math.toRadians(-3)) // Allow 3 degrees of backtracking
                    currentAngleRadians += 2 * Math.PI;

                if (distanceSensorReturn <= maxDist){
                    if (currentAngleRadians > Math.toRadians(noPropAngle))
                        ;
                    else if (currentAngleRadians > Math.toRadians(propSpot3Angle)) {
                        propPos3.add(new PointF((float) (Math.cos(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.x),
                                (float) (Math.sin(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.y)));
                    } else if (currentAngleRadians > Math.toRadians(propSpot2Angle)) {
                        propPos2.add(new PointF((float) (Math.cos(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.x),
                                (float) (Math.sin(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.y)));
                    } else if (currentAngleRadians > Math.toRadians(propSpot1Angle)) {
                        propPos1.add(new PointF((float) (Math.cos(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.x),
                                (float) (Math.sin(drive.pose.heading.log() + Math.PI) * distanceSensorReturn + drive.pose.position.y)));
                    }
                }

                packet.put("propPos1", propPos1.size());
                packet.put("propPos2", propPos2.size());
                packet.put("propPos3", propPos3.size());
                packet.put("noPropAngle", noPropAngle);
                packet.put("currentAngle", Math.toDegrees(currentAngleRadians));

                canvas.setStrokeWidth(1);
                canvas.setFill("#ff0000");
                plotPointsInRoadRunner(propPos1, canvas);

                canvas.setFill("#00ff00");
                plotPointsInRoadRunner(propPos2, canvas);

                canvas.setFill("#0000ff");
                plotPointsInRoadRunner(propPos3, canvas);

                canvas.setStroke("#808080");
                canvas.strokeCircle(drive.pose.position.x, drive.pose.position.y, maxDist);

                plotPropSpots(Math.toRadians(propSpot1Angle), canvas);
                plotPropSpots(Math.toRadians(propSpot2Angle), canvas);
                plotPropSpots(Math.toRadians(propSpot3Angle), canvas);

                if (currentAngleRadians < Math.toRadians(doneAngle))
                    return true;

                //movement has finished. find result.
                if (propPos1.size() > propPos2.size() && propPos1.size() > propPos3.size()) {
                    results.result = SpikeMarks.RIGHT;

                } else if (propPos2.size() > propPos3.size()) {
                    results.result = SpikeMarks.CENTER;

                } else {
                    results.result = SpikeMarks.LEFT;
                }
                packet.put("Final result", results.result);

                return false;
            }

            private void plotPropSpots(double lineAngle, Canvas canvas) {
                canvas.strokeLine(drive.pose.position.x, drive.pose.position.y,
                        Math.cos(lineAngle + initAngle) * maxDist + drive.pose.position.x,
                        Math.sin(lineAngle + initAngle) * maxDist + drive.pose.position.y);
            }

            private void plotPointsInRoadRunner(ArrayList<PointF> arrayOfPoints, Canvas canvas) {
                for (PointF point: arrayOfPoints)
                    canvas.fillRect(point.x - 1, point.y - 1, 3, 3);
            }
        };
    }

}

class PropDistanceFactory {
    class PoseAndAction {
        Action action;
        Pose2d startPose;

        PoseAndAction(Action action, Pose2d startPose) {
            this.action = action;
            this.startPose = startPose;
        }
    }

    private final double xOffset = -24;
    private final double yOffset = 0;
    MecanumDrive drive;
    PropDistanceFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    Pose2d xForm(double x, double y, double theta, boolean isRed, boolean isFar) {
        if (!isFar)
            x = x * -1 + xOffset;

        if (!isRed)
            y = y * -1 + yOffset;

        return new Pose2d(x, y, theta);
    }
    PoseAndAction getDistanceAction(boolean isRed, boolean isFar, Action sweepAction) {
        double tangent = Math.PI / 2;

        if (sweepAction == null) {
            sweepAction = new SleepAction(0.5);
        }

        if (!isRed) {
            tangent = tangent * -1;
        }

        Pose2d startPose = xForm(-34, -60, Math.PI /2, isRed, isFar);

        TrajectoryActionBuilder builder
                = this.drive.actionBuilder(startPose)
                .setTangent(tangent)
                .splineToLinearHeading(xForm(-42, -45, Math.PI / 2, isRed, isFar), tangent)
                .afterTime(0, sweepAction)
                .turn(2 * Math.PI)
                .setTangent(-tangent)
//                 .stopAndAdd(new SleepAction(100000000))
                .splineToLinearHeading(startPose, -tangent);

        return new PoseAndAction(builder.build(), startPose);
    }
    Action getMeepMeepAction() {
        return getDistanceAction(false, false, null).action;
    }
}
