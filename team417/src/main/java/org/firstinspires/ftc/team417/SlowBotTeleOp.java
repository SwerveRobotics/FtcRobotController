package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.skidaddle.AutoDriveTo;
import org.firstinspires.ftc.team417.skidaddle.DPoint;

@TeleOp(name = "TeleOp", group = "SlowBot")
@Config
public class SlowBotTeleOp extends BaseOpModeSlowBot {
    private double speedMultiplier = 1;
    public double targetHeading;
    boolean curve = true;
    boolean fieldCentered = false;

    public double startHeading;

    /* A number in degrees that the triggers can adjust the arm position by */
    // TODO: needs tuning
    public final double MAX_SLIDE_VELOCITY = 2000 ; // Ticks per second
    public final double FUDGE_FACTOR_LIFT = 200; // Ticks

    /* Variables that are used to set the arm to a specific position */
    double liftPositionFudgeFactor;

    // This variable remembers the time in the previous loop to use for slide velocity
    double previousTime = 0.0;

    boolean intakeEnabled = false;
    boolean buttonAPressed = false;

    // These are variables that will be used for individual control actions
    double slidePosition = 0;
    double wristPosition = WRIST_IN;
    double liftPosition = LIFT_HOME_POSITION;

    AutoDriveTo driveTo;

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        prepareRobot();

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            // Disable field-centric always!
            // toggleFieldCentricity();

            controlDrivebaseWithGamepads(curve, fieldCentered);

            controlMechanismsWithGamepads();

            if (drive.colorProcessor != null) {
                drive.colorProcessor.update();
            }

            telemeterData();

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);

        }

        // Reset the wrist position to avoid "breaking" the servo:
        stopCrash();
    }

    public void prepareRobot() {
        prepareRobot(new Pose2d(0, 0, Math.PI / 2));
    }

    public void prepareRobot(Pose2d startingPose) {
        targetHeading = startingPose.heading.log();

        drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, startingPose);
        initializeHardware();

        startHeading = startingPose.heading.log();
        previousTime = currentTime();

        driveTo = new AutoDriveTo(drive);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    public final double HEADING_HOLD_EPSILON = 0.01;
    public static double HEADING_HOLD_KP = 0; // Disable correction by setting kP to 0

    public void controlDrivebaseWithGamepads(boolean curveStick, boolean fieldCentric) {
        // Only on GamePad1, the right and left triggers are speed multipliers
        speedMultiplier = 0.5;
        if (gamepad1.left_trigger > 0.1) {
            speedMultiplier *= 0.5;
        }
        if (gamepad1.right_trigger > 0.1) {
            speedMultiplier *= 2;
        }

        // Use the left and right bumpers as shortcuts to turn 90 degrees
        if (gamepad1.left_bumper) {
            targetHeading -= Math.PI / 2;
        }
        if (gamepad1.right_bumper) {
            targetHeading += Math.PI / 2;
        }

        // Normal
        targetHeading = ((targetHeading + Math.PI) % (2 * Math.PI)) - Math.PI;
        if (targetHeading < -Math.PI) {
            targetHeading += 2 * Math.PI;
        }

        // When slides are out, slow down robot
        if (getSlidePosition() > (SLIDE_HOME_POSITION + TICKS_EPSILON)){
            speedMultiplier *= 0.5;
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

        // Set the drive motor powers according to the gamepad input:
        if (Math.abs(rot) > HEADING_HOLD_EPSILON) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -rotatedY * speedMultiplier,
                            -rotatedX * speedMultiplier
                    ),
                    -rot * speedMultiplier
            ));
            targetHeading = drive.pose.heading.log();
        } else {
            double correction = shortestAngleDistance(drive.pose.heading.log(), targetHeading) * HEADING_HOLD_KP;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -rotatedY * speedMultiplier,
                            -rotatedX * speedMultiplier
                    ),
                    correction
            ));
        }

        // Press the D-Pad down button ONCE (do not hold)
        if (gamepad1.dpad_down) {
            driverAssistRetrieveSpecimen();
        }

        // Update the current pose:
        drive.updatePoseEstimate();
    }

    // Method to calculate signed shortest distance between two angles
    public static double shortestAngleDistance(double theta1, double theta2) {
        // Calculate the raw difference
        double deltaTheta = theta2 - theta1;

        // Wrap into the range [-pi, pi]
        deltaTheta = (deltaTheta + Math.PI) % (2 * Math.PI) - Math.PI;

        // Handle edge case due to negative modulo in Java
        if (deltaTheta < -Math.PI) {
            deltaTheta += 2 * Math.PI;
        }

        return deltaTheta;
    }

    public void controlMechanismsWithGamepads() {

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
            if (gamepad2.x) {
                liftPosition = LIFT_SCORE_LOW_BASKET;
                wristPosition = WRIST_OUT;
                slidePosition = SLIDE_SCORE_IN_BASKET;
            }

            // High Basket
             if (gamepad2.y) {
                 liftPosition = LIFT_SCORE_HIGH_BASKET;
                 wristPosition = WRIST_OUT;
                 slidePosition = SLIDE_SCORE_IN_BASKET;
             }

             // Intake on and off
            if(gamepad2.a && !buttonAPressed){
                intakeEnabled = !intakeEnabled;
            }
            buttonAPressed = gamepad2.a;

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

            // Set the slide velocity to magnitude of the 'right stick y' multiplied by the speed multiplier (MAX_SLIDE_VELOCITY)
            double slideVelocity = -MAX_SLIDE_VELOCITY * gamepad2.right_stick_y;

            // deltaTime will be the actual current time minus the currentTime of the last loop
            double deltaTime = currentTime() - previousTime;
            previousTime = currentTime();

            // Set slide position based on magnitude of speed (Position = Velocity * Time)
            slidePosition += slideVelocity * deltaTime;

            // Makes sure if driver tries to go past the max or min, set the position to the max or min slide values
            if(slidePosition > SLIDE_MAX){
                slidePosition = SLIDE_MAX;
            }
            if(slidePosition < SLIDE_MIN){
                slidePosition = SLIDE_MIN;
            }

            // If lift is travelling through 'no mans land', pull in arm and wrist, then perform the lift action.
            // Else, perform all the actions
            double liftPositionWithFudge = liftPosition + liftPositionFudgeFactor;

            // In case position is over max or min, set position to the max or min
            if(liftPositionWithFudge > LIFT_MAX){
                liftPositionWithFudge = LIFT_MAX;
            }
            if(liftPositionWithFudge < LIFT_MIN){
                liftPositionWithFudge = LIFT_MIN;
            }
            // TEMPORARY
            PIDFCoefficients pidf = liftMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidf.f = f_coefficient;
            liftMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            PIDFCoefficients pidf2 = liftMotor2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidf2.f = f_coefficient;
            liftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf2);
            telemetry.addLine(String.format("F value: %.1f", f_coefficient ));

            if(isCrossingNoSlideZone(liftPositionWithFudge )) {
                if(getSlidePosition() > SLIDE_HOME_POSITION + TICKS_EPSILON) {
                    // moves the slides in if lift going up and not in home pos
                    moveWrist(WRIST_IN);
                    moveSlide(SLIDE_HOME_POSITION);

                } else {
                    // if slide in home position move lift up
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
            if (reversed) {
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
    final public double FIRST_SEGMENT_4_BAR_LENGTH = 17;

    // Raise the arm and move backwards simultaneously
    public void driverAssistRetrieveSpecimen() {
        Pose2d initialPose = new Pose2d(
                drive.pose.position.x,
                drive.pose.position.y,
                drive.pose.heading.log()
        );

        // Move the lift to take the specimen off
        moveLift(LIFT_GET_SPECIMEN);

        while (liftMotor1.isBusy()) {
            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            PoseVelocity2d currentPoseVelocity = drive.updatePoseEstimate();

            // deltaTime will be the actual current time minus the currentTime of the last loop
            double deltaTime = currentTime() - previousTime;
            previousTime = currentTime();

            // Determine, based on the angle, the what place the driveTo should drive to
            double angle = INITIAL_4_BAR_ANGLE -
                    liftMotor1.getCurrentPosition() * ARM_TICKS_PER_DEGREE;
            double targetDistance = FIRST_SEGMENT_4_BAR_LENGTH * Math.cos(angle);
            DPoint targetPoint = new DPoint(
                    initialPose.position.x,
                    initialPose.position.y - targetDistance
            );
            double targetHeading = initialPose.heading.log();

            // Actually drive there
            driveTo.init(targetPoint, targetHeading, currentPoseVelocity, telemetry);
            driveTo.linearDriveTo(currentPoseVelocity, deltaTime, packet, packet.fieldOverlay());
        }
    }

    // Applies a curve to the joystick input to give finer control at lower speeds
    public double curveStick(double rawSpeed) {
        return Math.copySign(Math.pow(rawSpeed, 2), rawSpeed);
    }

    boolean startWasPressed = false;
    public void toggleFieldCentricity() {
        if (!startWasPressed && gamepad1.start) {
            fieldCentered = !fieldCentered;
        }
        startWasPressed = gamepad1.start;
    }

    public void telemeterData() {
        telemetry.addLine("Running TeleOp!");
        telemetry.addData("Kinematic Type", kinematicType);
        telemetry.addData("Stick Curve On", curve);
        telemetry.addData("Field-Centric", fieldCentered);
        telemetry.addData("Speed Multiplier", speedMultiplier);

        telemetry.addData("Lift Motor 1 ticks: ", liftMotor1.getCurrentPosition());
        telemetry.addData("Lift Motor 2 ticks: ", liftMotor2.getCurrentPosition());

        telemetry.addData("Slide motor ticks: ", slideMotor.getCurrentPosition());

        telemetry.update();
    }
}