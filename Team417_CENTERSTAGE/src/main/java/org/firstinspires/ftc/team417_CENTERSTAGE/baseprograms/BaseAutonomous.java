package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import static java.lang.System.nanoTime;

import android.graphics.PointF;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.team417_CENTERSTAGE.apriltags.AprilTagPoseEstimator;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.stream.Stream;

@Config
abstract public class BaseAutonomous extends BaseOpMode {
    // Set to false for competitions to remove lags
    public static final boolean TESTING = true;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean USE_APRIL_TAGS = false;
    private boolean USE_OPEN_CV_PROP_DETECTION = true;

    public static double NANO_TO_SECONDS_MULTIPLIER = 1e-9;

    public OpenCvColorDetection myColorDetection;
    public AprilTagPoseEstimator myATPoseEstimator;

    public void initializeAuto() {
        if (USE_OPEN_CV_PROP_DETECTION) {
            myColorDetection = new OpenCvColorDetection(this);
            myColorDetection.init();
        }
        initializeHardware();
    }

    class ATContinuallyEstimatePoseAction implements Action {
        public boolean run(TelemetryPacket packet) {
            if (!opModeIsActive()) return false;
            myATPoseEstimator.updatePoseEstimate();
            return true;
        }
    }

    class UpdateVariablesInConfig implements Action {
        public boolean run(TelemetryPacket packet) {
            if (!opModeIsActive()) return false;
            org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.pose = drive.pose;
            org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.armPosition = armMotor.getCurrentPosition();
            return true;
        }
    }

    class DoTelemetry implements Action {
        public boolean run(TelemetryPacket packet) {
            if (!opModeIsActive()) return false;
            telemetry.addLine(org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.summary);
            telemetry.addLine(String.format("Robot XYθ %6.1f %6.1f %6.1f  (inch) (degrees)",
                    drive.pose.position.x, drive.pose.position.y,
                    Math.toDegrees(drive.pose.heading.log())));
            telemetry.update();
            return true;
        }
    }

    class updateToDashboard implements Action {
        Action path;

        public updateToDashboard(Action path) {
            this.path = path;
        }

        public boolean run(TelemetryPacket packet) {
            if (!opModeIsActive()) return false;
            // Code added to draw the pose, use only when testing
            if (TESTING) {
                TelemetryPacket p = new TelemetryPacket();
                Canvas c = p.fieldOverlay();

                path.preview(c);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.sendTelemetryPacket(p);
            }
            return true;
        }
    }

    public void runAuto(boolean red, boolean close, boolean openCV, boolean backdropPixel, boolean aprilTags) {
        telemetry.addLine(org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.summary);
        telemetry.addData("Init State", "Init Started");
        telemetry.update();

        USE_OPEN_CV_PROP_DETECTION = openCV;
        USE_APRIL_TAGS = false; // Just not using April Tags for auto, adds unpredicability

        initializeAuto();

        if (myColorDetection != null) {
            if (red) {
                myColorDetection.setDetectColor(OpenCvColorDetection.DetectColorType.RED);
            } else {
                myColorDetection.setDetectColor(OpenCvColorDetection.DetectColorType.BLUE);
            }
        }

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        AutonDriveFactory.PoseAndAction leftPoseAndAction = auton.getDriveAction(red, !close,
                AutonDriveFactory.SpikeMarks.LEFT, dropPixel(0.5, 0.3),
                driveToDistanceAndMoveArm(8, 2700), moveDumperAction(0), startMoveBackward(), endMoveBackward(), moveGateAction(0));
        AutonDriveFactory.PoseAndAction centerPoseAndAction = auton.getDriveAction(red, !close,
                AutonDriveFactory.SpikeMarks.CENTER, dropPixel(0.5, 0.8),
                driveToDistanceAndMoveArm(9, 2700), moveDumperAction(0), startMoveBackward(), endMoveBackward(), moveGateAction(0));
        AutonDriveFactory.PoseAndAction rightPoseAndAction = auton.getDriveAction(red, !close,
                AutonDriveFactory.SpikeMarks.RIGHT, dropPixel(0.5, 0.3),
                driveToDistanceAndMoveArm(9, 2700), moveDumperAction(0), startMoveBackward(), endMoveBackward(), moveGateAction(0));

        telemetry.addLine(org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.summary);
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addLine(org.firstinspires.ftc.team417_CENTERSTAGE.competitionprograms.Config.summary);
            telemetry.addData("Side detected", myColorDetection.detectTeamProp());
            telemetry.update();
        }

        /*The variable 'translateEnum' converts the OpenCV Enum by Hank to the Enum used by
        AutonDriveFactory created by Sawar.
        */
        AutonDriveFactory.SpikeMarks translateEnum;

        if (myColorDetection != null) {
            while (myColorDetection.detectTeamProp() == OpenCvColorDetection.SideDetected.INITIALIZED)
                ;

            // Just in case (flash after camera is completely initialized, could lead to instability)
            sleep(100);

            OpenCvColorDetection.SideDetected result = myColorDetection.detectTeamProp();

            if (result == OpenCvColorDetection.SideDetected.LEFT) {
                translateEnum = AutonDriveFactory.SpikeMarks.LEFT;
            } else if (result == OpenCvColorDetection.SideDetected.CENTER) {
                translateEnum = AutonDriveFactory.SpikeMarks.CENTER;
            } else {
                translateEnum = AutonDriveFactory.SpikeMarks.RIGHT;
            }

            // Since April Tag Detector objects cannot exist at the same time as any other robot
            //     camera objects, close the robot camera first... (see below)
            myColorDetection.robotCamera.closeCameraDevice();
        } else {
            PropDistanceResults distanceResult = new PropDistanceResults();
            PropDistanceFactory propDistance = new PropDistanceFactory(drive);
            Action sweepAction = distanceResult.sweepAction(drive, distSensor, distanceResult, !close);
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

        if (USE_APRIL_TAGS) {
            // (see above) ...and only then initialize the April Tag Pose Estimator
            myATPoseEstimator = new AprilTagPoseEstimator(hardwareMap, telemetry);

            // Pass an April Tag Helper object so drive can add twists to it (See MecanumDrive for
            //     updatePoseEstimate() an explanation on twists)
            drive.setATLHelper(myATPoseEstimator.myAprilTagLatencyHelper);
        }

        AutonDriveFactory.PoseAndAction poseAndAction;
        switch (translateEnum) {
            case LEFT:
                poseAndAction = leftPoseAndAction;
                break;
            case CENTER:
                poseAndAction = centerPoseAndAction;
                break;
            case RIGHT:
                poseAndAction = rightPoseAndAction;
                break;
            default:
                poseAndAction = centerPoseAndAction;
        }

        drive.pose = poseAndAction.startPose;

        drive.runParallel(poseAndAction.action);

        if (USE_APRIL_TAGS) {
            drive.runParallel(new ATContinuallyEstimatePoseAction());
        }

        drive.runParallel(new UpdateVariablesInConfig());

        drive.runParallel(new DoTelemetry());

        drive.runParallel(new updateToDashboard(poseAndAction.action));

        ElapsedTime timeStill = new ElapsedTime();
        timeStill.reset();
        while (opModeIsActive() && timeStill.seconds() < 5) {
            drive.doActionsWork();
            if (Stream.of(drive.rightFront, drive.leftBack, drive.rightBack, drive.leftFront).allMatch(motor -> isEpsilonEquals(motor.getPower(), 0))) {
                timeStill.reset();
            }
        }

        while (drive.pose.position.x < 48 && opModeIsActive()) {
            drive.rightBack.setPower(0.2);
            drive.rightFront.setPower(-0.2);
            drive.leftBack.setPower(-0.2);
            drive.rightFront.setPower(0.2);
        }

        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightFront.setPower(0);

        telemetry.addLine("Running closing procedure: ");
        telemetry.update();

        if (myATPoseEstimator != null) {
            myATPoseEstimator.visionPortal.close();
        }
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
                    if (intakeMotor != null) {
                        intakeMotor.setPower(intakeSpeed);
                    }
                    startTime = nanoTime() * NANO_TO_SECONDS_MULTIPLIER;
                }
                // Checks if the elapsed time is greater than the intake time to stop the motor
                if (nanoTime() * NANO_TO_SECONDS_MULTIPLIER - startTime > intakeTime) {
                    if (intakeMotor != null) {
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

    public void setMotorPower(double speed) {
        drive.leftFront.setPower(speed);
        drive.rightFront.setPower(speed);
        drive.leftBack.setPower(speed);
        drive.rightBack.setPower(speed);
    }

    public Action driveToDistanceAndMoveArm(double goalDistance, double armGoalPos) {
        return new Action() {
            final double epsilon = 1;
            double iterations;
            double goalIterations = 20;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (distSensor != null) {
                    if (Math.abs(goalDistance - distSensor.getDistance(DistanceUnit.INCH)) < epsilon) {
                        iterations++;

                        setMotorPower(0);

                        if (iterations < goalIterations)
                            return true;

                        if (drive.isDevBot || drive.is6220sDevBot)
                            return false;

                        dumperServo.setPosition(DUMPER_SERVO_DUMP_POSITION_AUTON);
                        armMotor.setPower(0.7);

                        if (armMotor.getCurrentPosition() < armGoalPos)
                            return true;

                        armMotor.setPower(0);
                        return false;
                    } else if (distSensor.getDistance(DistanceUnit.INCH) < goalDistance)
                        setMotorPower(0.1);
                    else
                        setMotorPower(-0.1);
                    return true;
                }

                return false;
            }
        };
    }

    public Action moveDumperAction(double armGoalPos) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                if (dumperServo != null) {
                    dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
                }
                return false;
            }
        };
    }

    public Action moveGateAction(double armGoalPos) {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                if (gateServo != null) {
                    gateServo.setPosition(GATE_SERVO_OPEN_POSITION);
                }
                return false;
            }
        };
    }

    public Action startMoveBackward() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                setMotorPower(0.1);
                return false;
            }
        };
    }

    public Action endMoveBackward() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                setMotorPower(0);
                return false;
            }
        };
    }
}

@Config
class AutonDriveFactory {
    MecanumDrive drive;
    double xOffset;
    double yMultiplier;

    double parkingOffset;

    double parkingOffsetCenterFar;

    double centerMultiplier;

    double centerOffset;

    public static double turnToBackdropAmount = 195;

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
    PoseAndAction getDriveAction(boolean isRed, boolean isFar, SpikeMarks location, Action intake, Action moveArm, Action moveDumper, Action startMoveBackward, Action endMoveBackward, Action moveGateAction) {

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
                .splineTo(xForm(new Vector2d(-33.5, -34)), xForm((Math.toRadians(180))))
                .stopAndAdd(intake);
                // TODO: Add if done with spike mark
                /*
                .splineToConstantHeading(xForm(new Vector2d(-30, -34)), xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-34, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(24, -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(turnToBackdropAmount)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                .afterTime(0, moveDumper)
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -38)), xForm(Math.toRadians(0)))
                .stopAndAdd(moveArm)
                .afterTime(0.1, moveGateAction)
                .afterTime(0.2, startMoveBackward)
                .afterTime(0.70, endMoveBackward);
                */

        TrajectoryActionBuilder spikeCenter = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, (Math.toRadians(90)))));
        spikeCenter = spikeCenter.splineTo(xForm(new Vector2d(-34, -37)), xForm(Math.toRadians(90)))
                .stopAndAdd(intake);
                //.splineTo(xForm(new Vector2d(-34, -60)), xForm(Math.toRadians(90)))
                //.splineTo(xForm(new Vector2d(-34, -60)), xForm(Math.toRadians(90)))
                //.splineTo(xForm(new Vector2d(0, -36)), xForm(Math.toRadians(90)));

                // TODO: Add if done with spike mark
                /*
                .setTangent(xForm(Math.toRadians(-90)))
                .splineTo(xForm(new Vector2d(-34, -39)), xForm(Math.toRadians(-90)))
                .setTangent(xForm(Math.toRadians(180)))
                .splineTo(xForm(new Vector2d(-55, -39)), xForm(Math.toRadians(180)))
                .setTangent(xForm(Math.toRadians(90)))
                //.splineTo(xForm(new Vector2d(-55, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(24, -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(turnToBackdropAmount)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                .stopAndAdd(moveDumper)
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -43)), xForm(Math.toRadians(0)))
                .stopAndAdd(moveArm)
                .afterTime(0.1, moveGateAction)
                .afterTime(0.2, startMoveBackward)
                .afterTime(0.70, endMoveBackward);
                */

        TrajectoryActionBuilder spikeRight = this.drive.actionBuilder(xForm(new Pose2d(-34, -64, Math.toRadians(90))));
        spikeRight = spikeRight.splineTo(xForm(new Vector2d(-35, -37)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-34.5, -37)), xForm(Math.toRadians(0)))
                .stopAndAdd(intake);
                // TODO: Add if done with spike mark
                /*
                .splineToConstantHeading(xForm(new Vector2d(-40, -34)), xForm(Math.toRadians(0)))
                .splineTo(xForm(new Vector2d(-36, -30)), xForm(Math.toRadians(90)))
                .splineTo(xForm(new Vector2d(-30, -10)), xForm(Math.toRadians(0)))
                .splineToConstantHeading(xForm(new Vector2d(24, -12)), xForm(Math.toRadians(0)))
                .turn(Math.toRadians(turnToBackdropAmount)) //Turn so the arm faces the backdrop
                .setTangent(xForm(Math.toRadians(0)))
                .afterTime(0, moveDumper)
                .splineToConstantHeading(xForm(new Vector2d(48 - xOffset, -50.5)), xForm(Math.toRadians(0)))
                .stopAndAdd(moveArm)
                .afterTime(0.1, moveGateAction)
                .afterTime(0.2, startMoveBackward)
                .afterTime(0.70, endMoveBackward);
                */

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
        return getDriveAction(true, true, SpikeMarks.LEFT, null, null, null, null, null, null).action;
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
    public Action sweepAction(MecanumDrive drive, DistanceSensor distSensor, PropDistanceResults results, boolean isFar) {
        return new Action() {
            //Arrays that store the points that the distance sensor detected were on a spike mark.
            private final ArrayList<PointF> LeftSpikeMarkPoints = new ArrayList<>();
            private final ArrayList<PointF> CenterSpikeMarkPoints = new ArrayList<>();
            private final ArrayList<PointF> RightSpikeMarkPoints = new ArrayList<>();
            private double initAngle; //The angle of the robot when the function is first called.
            private boolean inited = false; //If the function has inited.

            private double xFormAngleDegrees(double theta) {
                if (!isFar)
                    return 360 - theta;
                else
                    return theta;
            }

            private double angleToScope(double angle) {
                while (angle > Math.PI)
                    angle -= 2 * Math.PI;
                while (angle < -Math.PI)
                    angle += 2 * Math.PI;

                return angle;
            }

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
                double xOffset = 2.75, yOffset = 4;
                double rotationOffset;

                rotationOffset = Math.tan(yOffset / (xOffset + distanceSensorReturn));
                rotationOffset = angleToScope(rotationOffset);

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
                selectedPoint = new PointF((float) (Math.cos(drive.pose.heading.log() + Math.PI - rotationOffset)
                        * distanceSensorReturn + drive.pose.position.x),
                        (float) (Math.sin(drive.pose.heading.log() + Math.PI)
                                * distanceSensorReturn + drive.pose.position.y));

                //if the distance sensor reading is within max dist,
                // update the corresponding array with the point that the distance sensor is detecting.
                if (distanceSensorReturn <= maxDist && isFar) {
                    if (currentAngleRadians - rotationOffset > Math.toRadians(noSpikeMarkAngle))
                        ;
                    else if (currentAngleRadians - rotationOffset > Math.toRadians(propSpot3Angle)) {
                        RightSpikeMarkPoints.add(selectedPoint);
                    } else if (currentAngleRadians - rotationOffset > Math.toRadians(propSpot2Angle)) {
                        CenterSpikeMarkPoints.add(selectedPoint);
                    } else if (currentAngleRadians - rotationOffset > Math.toRadians(propSpot1Angle)) {
                        LeftSpikeMarkPoints.add(selectedPoint);
                    }
                } else if (distanceSensorReturn <= maxDist) {
                    if (currentAngleRadians - rotationOffset < Math.toRadians(360.0 - noSpikeMarkAngle))
                        ;
                    else if (currentAngleRadians - rotationOffset < Math.toRadians(360.0 - propSpot3Angle)) {
                        LeftSpikeMarkPoints.add(selectedPoint);
                    } else if (currentAngleRadians - rotationOffset < Math.toRadians(360.0 - propSpot2Angle)) {
                        CenterSpikeMarkPoints.add(selectedPoint);
                    } else if (currentAngleRadians - rotationOffset < Math.toRadians(360.0 - propSpot1Angle)) {
                        RightSpikeMarkPoints.add(selectedPoint);
                    }
                }

                //send the number of points in each array of detected points to FTC dashboard.
                packet.put("propPos1", LeftSpikeMarkPoints.size());
                packet.put("propPos2", CenterSpikeMarkPoints.size());
                packet.put("propPos3", RightSpikeMarkPoints.size());
                packet.put("noPropAngle", xFormAngleDegrees(noSpikeMarkAngle));
                packet.put("currentAngle", Math.toDegrees(currentAngleRadians));

                //Plot the points from the first array of detected points on FTC dashboard in red.
                canvas.setStrokeWidth(1);
                canvas.setFill("#ff0000");
                plotPointsInRoadRunner(LeftSpikeMarkPoints, canvas);

                //Plot the points from the second array of detected points on FTC dashboard in green.
                canvas.setFill("#00ff00");
                plotPointsInRoadRunner(CenterSpikeMarkPoints, canvas);

                //Plot the points from the third array of detected points on FTC dashboard in blue.
                canvas.setFill("#0000ff");
                plotPointsInRoadRunner(RightSpikeMarkPoints, canvas);

                //Draw a circle centered on the robot with the diameter of maxDist.
                canvas.setStroke("#808080");
                canvas.strokeCircle(drive.pose.position.x, drive.pose.position.y, maxDist);

                //Draw lines to show the detection areas for each spike mark
                plotPropSpots(Math.toRadians(xFormAngleDegrees(propSpot1Angle)), canvas);
                plotPropSpots(Math.toRadians(xFormAngleDegrees(propSpot2Angle)), canvas);
                plotPropSpots(Math.toRadians(xFormAngleDegrees(propSpot3Angle)), canvas);

                //If the current angle of the is not past the end of the last spike mark,
                // return true and restart the function.
                if (currentAngleRadians < Math.toRadians(xFormAngleDegrees(doneAngle)))
                    return true;

                //Once the robot is past the end of the last spike mark,
                // find the array of detected points that has the most points in it and update result.
                if (LeftSpikeMarkPoints.size() > CenterSpikeMarkPoints.size() && LeftSpikeMarkPoints.size() > RightSpikeMarkPoints.size()) {
                    results.result = SpikeMarks.RIGHT;

                } else if (CenterSpikeMarkPoints.size() > RightSpikeMarkPoints.size()) {
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

    double xFormAngle(double theta, boolean isFar) {
        if (!isFar)
            return theta * -1;
        else
            return theta;
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

        Pose2d startPose = xForm(-36, -60, Math.PI / 2, isRed, isFar);

        TrajectoryActionBuilder builder
                = this.drive.actionBuilder(startPose)
                .setTangent(tangent)
                .splineToLinearHeading(xForm(-42, -45, Math.PI / 2, isRed, isFar), tangent)
                .afterTime(0, sweepAction)
                .turn(xFormAngle(2 * Math.PI, isFar))
                .setTangent(-tangent)
                .splineToLinearHeading(startPose, -tangent);

        return new PoseAndAction(builder.build(), startPose);
    }

    Action getMeepMeepAction() {
        return getDistanceAction(false, false, null).action;
    }
}