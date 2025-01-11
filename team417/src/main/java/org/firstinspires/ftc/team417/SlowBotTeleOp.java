package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.color.Color;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.HolonomicKinematics;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.skidaddle.AutoDriveTo;
import org.firstinspires.ftc.team417.skidaddle.DPoint;

@TeleOp(name = "TeleOp", group = "SlowBot")
@Config
public class SlowBotTeleOp extends BaseOpModeSlowBot {
    private double speedMultiplier = 1 / Math.sqrt(2);
    boolean curve = true;
    boolean fieldCentered = false;

    public double startHeading;


    /* A number in degrees that the triggers can adjust the arm position by */
    // TODO: needs tuning
    public final double FUDGE_FACTOR_LIFT = 200; // Ticks

    /* Variables that are used to set the arm to a specific position */
    double liftPositionFudgeFactor;

    // This variable remembers the time in the previous loop to use for slide velocity
    public double previousTime = 0.0;

    boolean intakeEnabled = false;
    boolean buttonAPressed2 = false;
    boolean buttonDpadDown1 = false;
    boolean runSpecimenGrab = false;
    // These are variables that will be used for individual control actions
    double slidePosition = 0;
    double wristPosition = WRIST_IN;
    double liftPosition = LIFT_HOME_POSITION;
    double startLiftSpecimenY = 0;
    AutoDriveTo driveTo;

    boolean holdHeading = true;

    boolean x1Pressed = false;
    boolean y1Pressed = false;
    boolean back1Pressed = false;

    boolean pathing = false;
    public DPoint HUMAN_ZONE_DRIVE_TO = new DPoint(-49, 60);
    public double HUMAN_ZONE_DRIVE_TO_HEADING = Math.PI / 2.0;
    public DPoint SPECIMEN_DRIVE_TO = new DPoint(0, 37.21);
    public double SPECIMEN_DRIVE_TO_HEADING = -Math.PI / 2.0;

    Color color;
    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        if (!prepareRobot()) {
            telemetry.addLine("WARNING, WARNING: Need to run PrepareRobot first.");
            telemetry.update();
            waitForStart();
            return;
        }
        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        poseVelocity = drive.updatePoseEstimate();

        while (opModeIsActive()) {
            // deltaTime will be the actual current time minus the currentTime of the last loop
            double deltaTime = currentTime() - previousTime;
            previousTime = currentTime();

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            controlDrivebaseWithGamepads(curve, fieldCentered, deltaTime, packet);

            controlMechanismsWithGamepads(deltaTime);

            if (drive.colorProcessor != null) {
                color = drive.colorProcessor.update();
            }

            telemeterData();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);

            // Draw the uncorrected pose in green if it's correcting and red if it's not:
            if (drive.distanceLocalizer != null && drive.distanceLocalizer.enabled) {
                Pose2d oldPose = new Pose2d(
                        drive.pose.position.x - drive.distanceLocalizer.correction.x,
                        drive.pose.position.y - drive.distanceLocalizer.correction.y,
                        drive.pose.heading.log()
                );
                if (drive.distanceLocalizer.correcting) {
                    packet.fieldOverlay().setStroke("#00FF00");
                } else {
                    packet.fieldOverlay().setStroke("#FF0000");
                }
                packet.fieldOverlay().strokeLine(
                        oldPose.position.x,
                        oldPose.position.y,
                        drive.pose.position.x,
                        drive.pose.position.y
                );
                Drawing.drawRobot(packet.fieldOverlay(), oldPose);
            }

            MecanumDrive.sendTelemetryPacket(packet);
        }

        // Reset the wrist position to avoid "breaking" the servo:
        stopCrash();
    }

    public boolean prepareRobot() {
        lastTargetRotVel = 0;


        if(!initializeHardware(null)) {
            return false;
        }

        startHeading = drive.pose.heading.log();
        targetRot = drive.pose.heading.log();
        previousTime = currentTime();

        driveTo = new AutoDriveTo(drive);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        return true;
    }

    boolean back1WasPressed = false;

    public void controlDrivebaseWithGamepads(boolean curveStick, boolean fieldCentric, double deltaTime, TelemetryPacket packet) {
        // Update the current pose:
        PoseVelocity2d currentPoseVelocity = drive.updatePoseEstimate();

        if(gamepad1.x){
            if(!x1Pressed){
                driveTo.init(HUMAN_ZONE_DRIVE_TO, HUMAN_ZONE_DRIVE_TO_HEADING, currentPoseVelocity, telemetry);
                // We should not move the arm, wrist, or intake for Drive-To.
//                armPosition = ARM_COLLECT;
//                wrist.setPosition(WRIST_FOLDED_OUT);
//                intakeEnabled = true;
                pathing = true;
                System.out.println("init");
            }
            if (pathing) {
                pathing = !driveTo.linearDriveTo(currentPoseVelocity, deltaTime, packet, packet.fieldOverlay());
                System.out.println("run");
            }
        } else if(gamepad1.y){
            if(!y1Pressed){
                driveTo.init(SPECIMEN_DRIVE_TO, SPECIMEN_DRIVE_TO_HEADING, currentPoseVelocity, telemetry);
                // We should not move the arm, wrist, or intake for Drive-To.
//                armPosition = ARM_VERTICAL;
//                wrist.setPosition(WRIST_FOLDED_IN);
//                intakeEnabled = false;

                liftPosition = LIFT_SCORE_HIGH_SPECIMEN;
                wristPosition = WRIST_IN;
                slidePosition = SLIDE_HOME_POSITION;

                pathing = true;
            }
            if (pathing) {
                pathing = !driveTo.linearDriveTo(currentPoseVelocity, deltaTime, packet, packet.fieldOverlay());
            }
        } else {
            pathing = false;
        }

        x1Pressed = gamepad1.x;
        y1Pressed = gamepad1.y;
        back1Pressed = gamepad1.back;

        // Toggle the hold-heading
        if (!back1WasPressed && gamepad1.back) {
            holdHeading = !holdHeading;
        }
        back1WasPressed = gamepad1.back;

        // Only on GamePad1, the right and left triggers are speed multipliers
        speedMultiplier = 1 / Math.sqrt(2);
        if (gamepad1.left_trigger > 0.1) {
            speedMultiplier /= Math.sqrt(2);
        }
        if (gamepad1.right_trigger > 0.1 && getLiftPosition() < LIFT_COLLECT+LIFT_TICKS_EPSILON) {
            speedMultiplier *= Math.sqrt(2);
        }

        // When slides are out, slow down robot
        if (getSlidePosition() > (SLIDE_HOME_POSITION + SLIDE_TICKS_EPSILON)) {
            speedMultiplier /= Math.sqrt(2);
        }

        double theta, x, y, rot, rotatedX, rotatedY;
        if (fieldCentric) {
            theta = drive.pose.heading.log() - startHeading;
        } else {
            theta = 0;
        }

        // Curve the stick if needed
        if (curveStick) {
            x = curveStick(gamepad1.left_stick_x);
            y = curveStick(gamepad1.left_stick_y);
            rot = curveStick(gamepad1.right_stick_x);
        } else {
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rot = gamepad1.right_stick_x;
        }

        // Rotate the movement direction counter to the bot's rotation
        rotatedX = x * Math.cos(theta) - y * Math.sin(theta);
        rotatedY = x * Math.sin(theta) + y * Math.cos(theta);
        if(!pathing) {
            if (holdHeading) {
                driveWithHeldHeading(
                        rotatedX * speedMultiplier,
                        rotatedY * speedMultiplier,
                        rot * speedMultiplier,
                        deltaTime);
            } else {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        -rotatedY * speedMultiplier,
                                        -rotatedX * speedMultiplier
                                ),
                                -rot * speedMultiplier
                        )
                );
            }
        }

        // Press the D-Pad down button ONCE (do not hold)
        if (gamepad1.dpad_down && !buttonDpadDown1) {
            startLiftSpecimenY = drive.pose.position.y;
            runSpecimenGrab = true;
        }
        buttonDpadDown1 = gamepad1.dpad_down;
        if (runSpecimenGrab) {
            runSpecimenGrab = driverAssistRetrieveSpecimen();
        }
        // Update the current pose:
        poseVelocity = drive.updatePoseEstimate();
    }

    PoseVelocity2d poseVelocity;

    double lastTargetRotVel;
    double targetRotVel;
    double targetRot;

    public double maxAngVel = MecanumDrive.PARAMS.maxAngVel;
    public double maxAngAccel = MecanumDrive.PARAMS.maxAngAccel;

    public void driveWithHeldHeading(double userX, double userY, double userRot, double deltaTime) {
        if (userRot == 0) {
            if (userX == 0 && userY == 0) {
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightBack.setPower(0);
                drive.rightFront.setPower(0);
                targetRotVel = 0;
                targetRot = drive.pose.heading.log();
                return;
            } else {
                targetRotVel = 0;
                targetRot = drive.pose.heading.log();
            }
        }

        userRot *= -maxAngVel;
        // Calculate the rotational component of the wheel velocities
        if (userRot > 0) {
            if (userRot > targetRotVel) {
                targetRotVel = Math.min(targetRotVel + maxAngAccel * deltaTime, userRot);
            } else {
                targetRotVel = Math.max(targetRotVel - maxAngAccel * deltaTime, userRot);
            }
        } else {
            if (userRot < targetRotVel) {
                targetRotVel = Math.max(targetRotVel - maxAngAccel * deltaTime, userRot);
            } else {
                targetRotVel = Math.min(targetRotVel + maxAngAccel * deltaTime, userRot);
            }
        }

        targetRot = targetRot + targetRotVel * deltaTime;

        while (targetRot < -Math.PI)
            targetRot += 2 * Math.PI;
        while (targetRot > Math.PI)
            targetRot -= 2 * Math.PI;

        double targetRotAccel = (targetRotVel - lastTargetRotVel) / deltaTime;

        lastTargetRotVel = targetRotVel;

        double[] angular = new double[]{targetRot, targetRotVel, targetRotAccel};

        Pose2dDual<Time> txWorldTarget = new Pose2dDual<>(
                new Vector2dDual<>(new DualNum<>(new double[]{0, 0, 0}),
                        new DualNum<>(new double[]{0, 0, 0})),
                Rotation2dDual.exp(new DualNum<>(angular)));

        PoseVelocity2dDual<Time> command = new HolonomicController(
                0, 0, MecanumDrive.PARAMS.headingGain,
                0, 0, MecanumDrive.PARAMS.headingVelGain
        ).compute(txWorldTarget, drive.pose, poseVelocity);

        // Enlighten Wily Works as to where we should be:
        WilyWorks.runTo(txWorldTarget.value(), txWorldTarget.velocity().value());

        HolonomicKinematics.WheelVelocities<Time> rotWheelVels = drive.kinematics.inverse(command);

        double voltage = drive.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(0,
                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

        // Calculate the driving/strafing component of the wheel velocities
        PoseVelocity2d powers = new PoseVelocity2d(
                new Vector2d(
                        -userY * speedMultiplier,
                        -userX * speedMultiplier
                ),
                0
        );

        HolonomicKinematics.WheelVelocities<Time> driveWheelVels = new HolonomicKinematics(kinematicType, 1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double driveMaxPowerMag = 1;
        for (DualNum<Time> power : driveWheelVels.all()) {
            driveMaxPowerMag = Math.max(driveMaxPowerMag, power.value());
        }

        // Add the rotational and the driving/strafing wheel velocities and divide by voltages to become powers
        /*final MotorFeedforward feedforward = new MotorFeedforward(0,
                0,
                0);*/
        double leftFrontPower = (driveWheelVels.leftFront.get(0) / driveMaxPowerMag) +
                feedforward.compute(rotWheelVels.leftFront) / voltage;
        double leftBackPower = (driveWheelVels.leftBack.get(0) / driveMaxPowerMag) +
                feedforward.compute(rotWheelVels.leftBack) / voltage;
        double rightBackPower = (driveWheelVels.rightBack.get(0) / driveMaxPowerMag) +
                feedforward.compute(rotWheelVels.rightBack) / voltage;
        double rightFrontPower = (driveWheelVels.rightFront.get(0) / driveMaxPowerMag) +
                feedforward.compute(rotWheelVels.rightFront) / voltage;

        double denominator = Math.max(
                Math.max(
                        Math.max(
                                Math.abs(leftFrontPower),
                                Math.abs(leftBackPower)
                        ),
                        Math.max(
                                Math.abs(rightBackPower),
                                Math.abs(rightFrontPower)
                        )
                ),
                1
        );

        drive.leftFront.setPower(leftFrontPower / denominator);
        drive.leftBack.setPower(leftBackPower / denominator);
        drive.rightBack.setPower(rightBackPower / denominator);
        drive.rightFront.setPower(rightFrontPower / denominator);
    }

    public void controlMechanismsWithGamepads(double deltaTime) {
        /* Here we handle the three buttons that have direct control of the intake speed.
        These control the continuous rotation servo that pulls elements into the robot,
        If the user presses A, it sets the intake power to the final variable that
        holds the speed we want to collect at.
        If the user presses X, it sets the servo to Off.
        And if the user presses B it reveres the servo to spit out the element.*/

        /* TECH TIP: If Else loops:
        We're using an else if loop on "gamepad1.x" and "gamepad1.b" just in case
        multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
        at the same time. "a" will win over and the intake will turn on. If we just had
        three if statements, then it will set the intake servo's power to multiple speeds in
        one cycle. Which can cause strange behavior. */

        // Low basket
//        if (color == Color.RED) {
//
//        }
        if (gamepad2.x) {
            liftPosition = LIFT_SCORE_LOW_BASKET;
            wristPosition = WRIST_SCORE;
            slidePosition = SLIDE_SCORE_IN_BASKET;
        }

        // High Basket
        if (gamepad2.y) {
            liftPosition = LIFT_SCORE_HIGH_BASKET;
            wristPosition = WRIST_SCORE;
            slidePosition = SLIDE_SCORE_IN_BASKET;
        }

        // Intake on and off
        if (gamepad2.a && !buttonAPressed2) {
            intakeEnabled = !intakeEnabled;
        }
        buttonAPressed2 = gamepad2.a;

        // Collecting Sample
        if (gamepad2.right_bumper) {
            liftPosition = LIFT_COLLECT;
            wristPosition = WRIST_OUT;
            slidePosition = SLIDE_COLLECT;
            intakeEnabled = true;
        }

        // Clear floor barrier for intake
        if (gamepad2.left_bumper) {
            slidePosition = SLIDE_COLLECT;
            wristPosition = WRIST_IN;
            liftPosition = LIFT_CLEAR_BARRIER;
        }

        // Retract linear slides
        if (gamepad2.dpad_left) {
            liftPosition = LIFT_HOME_POSITION;
            wristPosition = WRIST_IN;
            slidePosition = SLIDE_HOME_POSITION;
            intakeEnabled = false;
        }

        // Correct height for specimen (High)
        if (gamepad2.dpad_right) {
            liftPosition = LIFT_SCORE_HIGH_SPECIMEN;
            wristPosition = WRIST_IN;
            slidePosition = SLIDE_HOME_POSITION;
        }

        /* Here we create a "fudge factor" for the Lift position.
        This allows you to adjust (or "fudge") the Lift position slightly with the gamepad triggers.
        We want the left trigger to move the Lift up, and right trigger to move the Lift down.
        So we add the right trigger's variable to the inverse of the left trigger. If you pull
        both triggers an equal amount, they cancel and leave the Lift at zero. But if one is larger
        than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
        The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

        liftPositionFudgeFactor = FUDGE_FACTOR_LIFT * (gamepad2.right_trigger - gamepad2.left_trigger);

        // Set the slide velocity to magnitude of the 'right stick y' multiplied by the speed multiplier (SLIDE_VELOCITY)
        double slideVelocity = -SLIDE_VELOCITY_MAX * gamepad2.right_stick_y;
        telemetry.addData("Slide target Velocity: ", slideVelocity);


        // Set slide position based on magnitude of speed (Position = Velocity * Time)
        slidePosition += slideVelocity * deltaTime;

        // Makes sure if driver tries to go past the max or min, set the position to the max or min slide values
        if (slidePosition > SLIDE_MAX) {
            slidePosition = SLIDE_MAX;
        }
        if (slidePosition < SLIDE_MIN) {
            slidePosition = SLIDE_MIN;
        }

        // If lift is travelling through 'no mans land', pull in arm and wrist, then perform the lift action.
        // Else, perform all the actions
        double liftPositionWithFudge = liftPosition + liftPositionFudgeFactor;

        // In case position is over max or min, set position to the max or min
        if (liftPositionWithFudge > LIFT_MAX) {
            liftPositionWithFudge = LIFT_MAX;
        }
        if (liftPositionWithFudge < LIFT_MIN) {
            liftPositionWithFudge = LIFT_MIN;
        }

        if (isCrossingNoSlideZone(liftPositionWithFudge)) {
            if (getSlidePosition() > SLIDE_HOME_POSITION + SLIDE_TICKS_EPSILON) {
                // moves the slides in if lift going up and not in home pos
                moveWrist(WRIST_IN);
                moveSlide(SLIDE_HOME_POSITION);

            } else {
                // if slide in home position move lift up
                moveWrist(WRIST_IN);
                moveLift(liftPositionWithFudge);
            }
        } else {
            moveWrist(wristPosition);
            moveSlide(slidePosition);
            moveLift(liftPositionWithFudge);
        }

        // Reverse intake when b-button is held down
        boolean reversed = gamepad2.b;
        // When 'b' is HELD down, it will deposit
        if (reversed && wrist.getPosition()!=WRIST_IN) {
            intakeControl(INTAKE_DEPOSIT);
        } else if (intakeEnabled) {
            // When 'a' is clicked, it is TOGGLED, so it will keep collecting until another input is clicked
            intakeControl(INTAKE_COLLECT);
        } else {
            intakeControl(INTAKE_OFF);
        }
    }

    // Angle of depression from parallel to the ground when the 4-bar is completely collapsed
    final public double INITIAL_4_BAR_ANGLE = Math.PI / 6;

    // Length of first segment of 4-bar (inches)


    // Raise the arm and move backwards simultaneously
    public boolean driverAssistRetrieveSpecimen() {
        double yDisplace = Math.abs(startLiftSpecimenY - drive.pose.position.y);
        if (yDisplace >= FIRST_SEGMENT_4_BAR_LENGTH - X_NON_OVERHANG)
            return false; // too far, no point lifting anymore. we're done

        liftPosition = ((Math.toDegrees(-Math.acos((X_NON_OVERHANG + yDisplace) / FIRST_SEGMENT_4_BAR_LENGTH)))
                - STARTING_ANGLE) * LIFT_TICKS_PER_DEGREE;
        return true; // not done yet.
    }

    // Applies a curve to the joystick input to give finer control at lower speeds
    public double curveStick(double rawSpeed) {
        return Math.copySign(Math.pow(rawSpeed, 2), rawSpeed);
    }

    public void telemeterData() {
        telemetry.addData("Lift Motor 1 direction", liftMotor1.getDirection());
        telemetry.addData("Lift Motor 2 direction", liftMotor2.getDirection());
        telemetry.addLine("Running TeleOp!");
        telemetry.addData("Kinematic Type", kinematicType);
        telemetry.addData("Stick Curve On", curve);
        telemetry.addData("Field-Centric", fieldCentered);
        telemetry.addData("Hold Heading", holdHeading);
        telemetry.addData("Distance Sensors", drive.distanceLocalizer.enabled);
        telemetry.addData("Speed Multiplier", speedMultiplier);

        telemetry.addData("Lift Motor 1 ticks", liftMotor1.getCurrentPosition());
        telemetry.addData("Lift Motor 2 ticks", liftMotor2.getCurrentPosition());

        telemetry.addData("Slide motor ticks", slideMotor.getCurrentPosition());
        telemetry.addData("Slide motor velocity", slideMotor.getVelocity());
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.update();
    }
}