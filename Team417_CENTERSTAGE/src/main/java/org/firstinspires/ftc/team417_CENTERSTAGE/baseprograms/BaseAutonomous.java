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

import java.util.concurrent.CompletableFuture;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double APRIL_TAG_SLEEP_TIME = 500;
    public static double NO_APRIL_TAG_SLEEP_TIME = 2500;

    private final boolean USE_OPEN_CV_PROP_DETECTION = true;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 0.2;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    MecanumDrive drive;

    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);;

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        myColorDetection.init();
        initializeHardware();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Init State", "Init Finished");

        // Allow the OpenCV to process
        if (drive.USE_APRIL_TAGS) {
            sleep((long) APRIL_TAG_SLEEP_TIME);
        } else {
            sleep((long) NO_APRIL_TAG_SLEEP_TIME);
        }

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public void runAuto(boolean red, boolean close, boolean test) {
        if (red) {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            telemetry.addLine("Looking for red");
        } else {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
            telemetry.addLine("Looking for blue");
        }

        initializeAuto();

        waitForStart();

        OpenCvColorDetection.SideDetected result = myColorDetection.detectTeamProp();
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
        }

        // Close cameras to avoid errors
        myColorDetection.robotCamera.closeCameraDevice();

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red, !close, sawarResult, dropPixel());

        drive.pose = poseAndAction.startPose;

        if (!test) {
            CompletableFuture<Void> future = CompletableFuture.runAsync(() -> Actions.runBlocking(poseAndAction.action));

            while (opModeIsActive());

            future.cancel(true);
        } else {
            while (opModeIsActive());
        }


        //if (drive.myAprilTagPoseEstimator != null) {
        //    drive.myAprilTagPoseEstimator.visionPortal.close();
        //}
    }

    public void runAuto(boolean red, boolean close) {
        runAuto(red, close, false);
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

    double parkingOffset;

    double parkingOffsetCenterFar;

    double centerMultiplier;

    double centerOffset;



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
            parkingOffset = 48;
            centerMultiplier = 1;
            centerOffset = 0;

            /*if (location == xForm(SpikeMarks.CENTER)) {
                parkingOffset = 100;
            }*/


        } else {
            xOffset = 48;
            parkingOffset = 2;
            centerMultiplier = -1;
            centerOffset = 96;

        }

        if (isRed) {
            yMultiplier = 1;
        } else {
            yMultiplier = -1;
        }

        // in MeepMeep, intake needs to be null however .stopAndAdd() can't be null because it will crash so we set to a random sleep
        if (intake == null) {
            intake = new SleepAction(3);
        }

        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xMirroringXForm(new Pose2d(-36, -60, Math.toRadians(90)), isFar, 0));
        spikeRight = spikeRight.splineTo(xMirroringXForm(new Vector2d(-36, -38), isFar), xForm(Math.toRadians(90))) //Drive forward into the center of the spike marks
                .setTangent(xForm(0))
                .splineToConstantHeading(xForm(new Vector2d(-24, -38)), xForm(Math.toRadians(0))) //strafe onto the left spike mark
                .stopAndAdd(intake)
                .setTangent(xForm(Math.toRadians(180)))
                .splineToConstantHeading(xMirroringXForm(new Vector2d(-39, -38), isFar), xForm(Math.toRadians(180))) //strafe back between the spike marks
                .setTangent(xForm(Math.toRadians(90)))
                .splineTo(xMirroringXForm(new Vector2d(-39, -30), isFar), xForm(Math.toRadians(90))) //drive forward before starting the turn to go the backdrop
                .splineTo(xForm(new Vector2d(-18, -12)), xForm(Math.toRadians(0))) //curve to face the backdrop
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -29.5)), xForm(Math.toRadians(-90))) //strafe to the corresponding april tag on the backdrop
                .setTangent(xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(90)));

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-36, -60, (Math.toRadians(90)))));
        spikeCenter = spikeCenter.splineToSplineHeading(xMirroringXForm(new Pose2d(-42, -24, Math.toRadians(0)), isFar, 1), xForm(Math.toRadians(90)))
                .stopAndAdd(intake)
                //.splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                //.splineToConstantHeading(xFormCenter(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                //.splineToConstantHeading(xFormCenter(new Vector2d(-55, -30)), xForm(Math.toRadians(90)))
                .setTangent(xForm(Math.toRadians(90)))
                .splineToConstantHeading(xMirroringXForm(new Vector2d(-42, -12), isFar), xForm(Math.toRadians(90)))
                .setTangent(xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(xMirroringAngle(Math.toRadians(180), isFar)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -36)), xForm(Math.toRadians(-90))) //strafe to the corresponding april tag on the backdrop
                .setTangent(xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(90)));


        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xMirroringXForm(new Pose2d(-36, -60, Math.toRadians(90)), isFar, 0));
        spikeLeft = spikeLeft.splineTo(xMirroringXForm(new Vector2d(-36, -38), isFar), xForm(Math.toRadians(90))) //Drive forward into the center of the spike marks
                .setTangent(xForm(Math.toRadians(180)))
                .splineToConstantHeading(xForm(new Vector2d(-46, -38)), xForm(Math.toRadians(180))) //strafe onto the left spike mark
                .stopAndAdd(intake)
                .setTangent(xForm(Math.toRadians(0)))
                .splineToConstantHeading(xMirroringXForm(new Vector2d(-39, -38), isFar), xForm(Math.toRadians(0))) //strafe back between the spike marks
                .setTangent(xForm(Math.toRadians(90)))
                .splineTo(xMirroringXForm(new Vector2d(-39, -30), isFar), xForm(Math.toRadians(90))) //drive forward before starting the turn to go the backdrop
                .splineTo(xForm(new Vector2d(-18, -12)), xForm(Math.toRadians(0))) //curve to face the backdrop
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(0))) //drive to the backdrop
                .turn(Math.toRadians(180)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(-90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -44)), xForm(Math.toRadians(-90))) //strafe to the corresponding april tag on the backdrop
                .setTangent(xForm(Math.toRadians(90)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -12)), xForm(Math.toRadians(90)));

        if (location == xForm(SpikeMarks.LEFT)) {
            return new PoseAndAction(spikeLeft.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        } else if (location == xForm(SpikeMarks.RIGHT)) {
            return new PoseAndAction(spikeRight.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        } else {
            return new PoseAndAction(spikeCenter.build(), xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        }

    }


    Pose2d xForm(Pose2d pose) {
        return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    private final double xMirroringOffset = -24;

    Pose2d xMirroringXForm(Pose2d pose, boolean isFar, double angleFormAmount) {
        if (!isFar)
            return new Pose2d(pose.position.x * -1 + xMirroringOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier + Math.toRadians(180) * angleFormAmount);
        else
            return new Pose2d(pose.position.x + xOffset, pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Pose2d xFormCenter(Pose2d pose) {
        return new Pose2d((pose.position.x + centerOffset), pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    Vector2d xMirroringXForm(Vector2d vector, boolean isFar) {
        if (!isFar)
            return new Vector2d(vector.x * -1 + xMirroringOffset, vector.y * yMultiplier);
        else
            return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xMirroringAngle(double angle, boolean isFar) {
        if (!isFar)
            return angle - Math.PI + 0.00000000000001;
        else
            return angle;
    }

    Vector2d xFormCenter(Vector2d vector) {
        return new Vector2d((vector.x + centerOffset), vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }


    SpikeMarks xForm(SpikeMarks spike) {
        if (yMultiplier == -1) {
            switch (spike) {
                case LEFT:
                    return SpikeMarks.RIGHT;
                case RIGHT:
                    return SpikeMarks.LEFT;
            }
        }
        return spike;
    }





    /*
     * MeepMeep calls this routine to get a trajectory sequence action to draw. Modify the
     * arguments here to test your different code paths.
     */
    Action getMeepMeepAction() {
        return getDriveAction(true, false, SpikeMarks.LEFT , null).action;
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
                .splineToLinearHeading(startPose, -tangent);

        return new PoseAndAction(builder.build(), startPose);
    }
    Action getMeepMeepAction() {
        return getDistanceAction(false, false, null).action;
    }
}
