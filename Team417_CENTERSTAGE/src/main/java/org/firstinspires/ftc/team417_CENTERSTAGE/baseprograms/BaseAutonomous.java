package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.ArrayList;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double APRIL_TAG_SLEEP_TIME = 500;
    public static double NO_APRIL_TAG_SLEEP_TIME = 5000;

    private final boolean USE_OPEN_CV_PROP_DETECTION = false;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 2; // in seconds

    public static double INTAKE_SPEED2 = 1;

    public static double INTAKE_TIME2 = 10; // in seconds

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    MecanumDrive drive;

    public OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        if (USE_OPEN_CV_PROP_DETECTION) {
            myColorDetection = new OpenCvColorDetection(this);
            myColorDetection.init();
        }
        initializeHardware();

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
        if (USE_OPEN_CV_PROP_DETECTION) {
            if (red) {
                myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
                telemetry.addLine("Looking for red");
            } else {
                myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
                telemetry.addLine("Looking for blue");
            }
        }

        initializeAuto();

        waitForStart();

        /*The variable 'translateEnum' converts the OpenCV Enum by Hank to the Enum used by
        AutonDriveFactory created by Sawar.
        */
        AutonDriveFactory.SpikeMarks translateEnum;

        if (USE_OPEN_CV_PROP_DETECTION) {
            OpenCvColorDetection.SideDetected result = myColorDetection.detectTeamProp();

            if (result == OpenCvColorDetection.SideDetected.LEFT) {
                translateEnum = AutonDriveFactory.SpikeMarks.LEFT;
            } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
                translateEnum = AutonDriveFactory.SpikeMarks.CENTER;
            } else {
                translateEnum = AutonDriveFactory.SpikeMarks.RIGHT;
            }

            // Close camera to avoid errors
            myColorDetection.robotCamera.stopStreaming();
            myColorDetection.robotCamera.closeCameraDevice();
        } else {
            PropDistanceResults distanceResult = new PropDistanceResults();
            PropDistanceFactory propDistance = new PropDistanceFactory(drive);
            Action sweepAction = distanceResult.sweepAction(drive, distSensor, distanceResult);
            PropDistanceFactory.PoseAndAction poseAndAction = propDistance.getDistanceAction(red, !close, sweepAction);

            // Do the robot movement for prop detection:
            drive.pose = poseAndAction.startPose;
            Actions.runBlocking(poseAndAction.action);

            if (distanceResult.result == PropDistanceResults.SpikeMarks.LEFT)
                translateEnum = AutonDriveFactory.SpikeMarks.LEFT;
            else if (distanceResult.result == PropDistanceResults.SpikeMarks.CENTER)
                translateEnum = AutonDriveFactory.SpikeMarks.CENTER;
            else
                translateEnum = AutonDriveFactory.SpikeMarks.RIGHT;

            telemetry.addData("translateEnum", translateEnum);
            telemetry.update();
        }

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction poseAndAction = auton.getDriveAction(red, !close, translateEnum, dropPixel(1, 2));

        drive.pose = poseAndAction.startPose;

        if (!test) {
            Actions.runBlocking(poseAndAction.action);
        }

        telemetry.addLine("Running closing procedure: ");
        telemetry.update();

        if (drive.myAprilTagPoseEstimator != null) {
            drive.myAprilTagPoseEstimator.visionPortal.stopStreaming();
            drive.myAprilTagPoseEstimator.visionPortal.close();
        }
    }

    public void runAuto(boolean red, boolean close) {

        runAuto(red, close, false);
    }

    //Action: Spits out pixel in trajectory; see usage in AutonDriveFactory below.

    public Action dropPixel(double intakeTime, double intakeSpeed) {
        return new Action() {

            // Variable to store the start time for comparison
            double startTime = 0;

            @Override
            public boolean run(TelemetryPacket packet) {

                // Executes on the first loop to start the intake motor
                if (startTime == 0) {
                    if (drive.isDevBot && intakeMotor != null) {
                        intakeMotor.setPower(intakeSpeed);
                    }
                    startTime = nanoTime() * NANO_TO_SECONDS_MULTIPLIER;
                }
                // Checks if the elapsed time is greater than the intake time to stop the motor
                if (nanoTime() * NANO_TO_SECONDS_MULTIPLIER - startTime > intakeTime) {
                    if (drive.isDevBot && intakeMotor != null) {
                        intakeMotor.setPower(0);
                    }
                    // Resets start time for the next run
                    startTime = 0;
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

    /* Booleans 'isRed' (red or blue side), 'isFar' (far or close to backdrop)
     'location' (center, middle, or right), and 'intake' (Action for use).
     */
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake) {

        if (isFar) {
            xOffset = 0;
            parkingOffset = 55;
            centerMultiplier = 1;
            centerOffset = 0;
            if (location == xForm(SpikeMarks.CENTER)) {
                parkingOffset = 100;
            }
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

        TrajectoryActionBuilder spikeLeft = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeLeft = spikeLeft.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-35, -34)), xForm((Math.toRadians(180) + (1e-6))))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -10)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, (Math.toRadians(90)))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xFormCenter(new Vector2d(-55, -39)), xForm(Math.toRadians(90)))
                .splineToConstantHeading(xFormCenter(new Vector2d(-55, -30)), xForm(Math.toRadians(90)))
                .splineTo(xFormCenter(new Vector2d(parkingOffset - 43, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xFormCenter(new Vector2d(parkingOffset - 43, -10)), xForm(Math.toRadians(0)));

        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeRight = spikeRight.splineTo(xForm(new Vector2d(-35, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-33, -37)), xForm(Math.toRadians(0)))
                .stopAndAdd(intake)
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(parkingOffset, -10)), xForm(Math.toRadians(0)));

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

    Pose2d xFormCenter(Pose2d pose) {
        return new Pose2d((pose.position.x + centerOffset), pose.position.y * yMultiplier, pose.heading.log() * yMultiplier);
    }

    Vector2d xForm(Vector2d vector) {
        return new Vector2d(vector.x + xOffset, vector.y * yMultiplier);
    }

    double xForm(double angle) {
        return (angle * yMultiplier);
    }

    Vector2d xFormCenter(Vector2d vector) {
        return new Vector2d((vector.x + centerOffset), vector.y * yMultiplier);
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
        return getDriveAction(true, true, SpikeMarks.LEFT, null).action;
    }
}

//use the distance sensor to find the prop location.
@Config
class PropDistanceResults {

    //The angles in degrees that each spike mark is located at.
    public static double propSpot1Angle = 116, propSpot2Angle = 145,
            propSpot3Angle = 190, noSpikeMarkAngle = 220, doneAngle = 220;
    //The max distance a result from the distance sensor can be for it to be considered.
    public static double maxDist = 28;

    enum SpikeMarks {
        LEFT,
        CENTER,
        RIGHT
    } //The different locations the function can return.

    public SpikeMarks result = SpikeMarks.LEFT; //The value the prop distance factory is returning.

    //Checks for the prop with the distance sensor while the robot is spinning.
    public Action sweepAction(MecanumDrive drive, DistanceSensor distSensor, PropDistanceResults results) {
        return new Action() {
            //Arrays that store the points that the distance sensor detected were on a spike mark.
            private final ArrayList<PointF> SpikeMark1Pos = new ArrayList<>();
            private final ArrayList<PointF> SpikeMark2Pos = new ArrayList<>();
            private final ArrayList<PointF> SpikeMark3Pos = new ArrayList<>();
            private double initAngle; //The angle of the robot when the function is first called.
            private boolean inited = false; //If the function has inited.

            @Override
            public boolean run(TelemetryPacket packet) {
                //Setup canvas to draw to later.
                Canvas canvas = packet.fieldOverlay();

                //What the distance sensor is detecting.
                double distanceSensorReturn = distSensor.getDistance(DistanceUnit.INCH);
                //The current angle of the robot in radians.
                double currentAngleRadians;
                //The x and y location of the point the distance sensor is looking at.
                PointF selectedPoint;

                //Only runs once when the function is first called. Sets the value of the starting angle.
                if (!inited) {
                    initAngle = drive.pose.heading.log() + Math.PI;
                    inited = true;
                }

                //sets the current angle the robot is facing.
                currentAngleRadians = (drive.pose.heading.log() + Math.PI) - initAngle;
                // Allow 3 degrees of backtracking
                if (currentAngleRadians < Math.toRadians(-3))
                    currentAngleRadians += 2 * Math.PI;

                //sets selected point to the current point the distance sensor is detecting.
                selectedPoint = new PointF((float) (Math.cos(drive.pose.heading.log() + Math.PI)
                        * distanceSensorReturn + drive.pose.position.x),
                        (float) (Math.sin(drive.pose.heading.log() + Math.PI)
                                * distanceSensorReturn + drive.pose.position.y));

                //if the distance sensor reading is within max dist,
                // update the corresponding array with the point that the distance sensor is detecting.
                if (distanceSensorReturn <= maxDist) {
                    if (currentAngleRadians > Math.toRadians(noSpikeMarkAngle))
                        ;
                    else if (currentAngleRadians > Math.toRadians(propSpot3Angle)) {
                        SpikeMark3Pos.add(selectedPoint);
                    } else if (currentAngleRadians > Math.toRadians(propSpot2Angle)) {
                        SpikeMark2Pos.add(selectedPoint);
                    } else if (currentAngleRadians > Math.toRadians(propSpot1Angle)) {
                        SpikeMark1Pos.add(selectedPoint);
                    }
                }

                //send the number of points in each array of detected points to FTC dashboard.
                packet.put("propPos1", SpikeMark1Pos.size());
                packet.put("propPos2", SpikeMark2Pos.size());
                packet.put("propPos3", SpikeMark3Pos.size());
                packet.put("noPropAngle", noSpikeMarkAngle);
                packet.put("currentAngle", Math.toDegrees(currentAngleRadians));

                //Plot the points from the first array of detected points on FTC dashboard in red.
                canvas.setStrokeWidth(1);
                canvas.setFill("#ff0000");
                plotPointsInRoadRunner(SpikeMark1Pos, canvas);

                //Plot the points from the second array of detected points on FTC dashboard in green.
                canvas.setFill("#00ff00");
                plotPointsInRoadRunner(SpikeMark2Pos, canvas);

                //Plot the points from the third array of detected points on FTC dashboard in blue.
                canvas.setFill("#0000ff");
                plotPointsInRoadRunner(SpikeMark3Pos, canvas);

                //Draw a circle centered on the robot with the diameter of maxDist.
                canvas.setStroke("#808080");
                canvas.strokeCircle(drive.pose.position.x, drive.pose.position.y, maxDist);

                //Draw lines to show the detection areas for each spike mark
                plotPropSpots(Math.toRadians(propSpot1Angle), canvas);
                plotPropSpots(Math.toRadians(propSpot2Angle), canvas);
                plotPropSpots(Math.toRadians(propSpot3Angle), canvas);

                //If the current angle of the is not past the end of the last spike mark,
                // return true and restart the function.
                if (currentAngleRadians < Math.toRadians(doneAngle))
                    return true;

                //Once the robot is past the end of the last spike mark,
                // find the array of detected points that has the most points in it and update result.
                if (SpikeMark1Pos.size() > SpikeMark2Pos.size() && SpikeMark1Pos.size() > SpikeMark3Pos.size()) {
                    results.result = SpikeMarks.RIGHT;

                } else if (SpikeMark2Pos.size() > SpikeMark3Pos.size()) {
                    results.result = SpikeMarks.CENTER;

                } else {
                    results.result = SpikeMarks.LEFT;
                }
                packet.put("Final result", results.result); //send the final result to FTC dashboard.

                return false; //Once the result has been calculated, return false to end the action.
            }

            private void plotPropSpots(double lineAngle, Canvas canvas) { //Plot the areas of the each spike mark.
                canvas.strokeLine(drive.pose.position.x, drive.pose.position.y,
                        Math.cos(lineAngle + initAngle) * maxDist + drive.pose.position.x,
                        Math.sin(lineAngle + initAngle) * maxDist + drive.pose.position.y);
            }

            private void plotPointsInRoadRunner(ArrayList<PointF> arrayOfPoints, Canvas canvas) { //Plot each detected point.
                for (PointF point : arrayOfPoints)
                    canvas.fillRect(point.x - 1, point.y - 1, 3, 3);
            }
        };
    }

}

//Drive forward, spin, and drive back so the prop location can be found.
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

    //transforms the x and y coordinates for each starting location.
    Pose2d xForm(double x, double y, double theta, boolean isRed, boolean isFar) {
        if (!isFar)
            x = x * -1 + xOffset;

        if (!isRed)
            y = y * -1 + yOffset;

        return new Pose2d(x, y, theta);
    }

    //drives and spins to find the prop location.
    PoseAndAction getDistanceAction(boolean isRed, boolean isFar, Action sweepAction) {
        double tangent = Math.PI / 2;

        if (sweepAction == null) { //creates sweep action
            sweepAction = new SleepAction(0.5);
        }

        if (!isRed) {
            tangent = tangent * -1;
        }

        Pose2d startPose = xForm(-34, -60, Math.PI / 2, isRed, isFar);

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
