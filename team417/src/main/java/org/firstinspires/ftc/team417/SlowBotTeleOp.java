package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

@TeleOp(name = "TeleOp", group = "SlowBot")
@Config
public class SlowBotTeleOp extends BaseOpModeSlowBot {
    private double speedMultiplier = 1;
    boolean curve = true;
    boolean fieldCentered = false;

    public double startHeading;

    /* A number in degrees that the triggers can adjust the arm position by */
    // TODO: needs tuning
    final double MAX_SLIDE_VELOCITY = 200; // Ticks per second
    final double FUDGE_FACTOR_LIFT = 200; // Ticks

    /* Variables that are used to set the arm to a specific position */
    double liftPositionFudgeFactor;

    // This variable remembers the tixme in the previous loop to use for slide velocity
    double previousTime = 0.0;

    boolean intakeEnabled = false;
    boolean buttonAPressed = false;

    // These are variables that will be used for individual control actions
    double slidePosition = SLIDE_HOME_POSITION;
    double wristPosition = WRIST_IN;
    double liftPosition = LIFT_HOME_POSITION;

    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        prepareRobot();

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            toggleFieldCentricity();

            controlDrivebaseWithGamepads(curve, fieldCentered);

            controlMechanismsWithGamepads();

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
    }

    public void prepareRobot() {
        prepareRobot(new Pose2d(0, 0, Math.PI / 2));
    }

    public void prepareRobot(Pose2d startingPose) {
        drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, startingPose);
        initializeHardware();

        startHeading = startingPose.heading.log();
        previousTime = currentTime();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    public void controlDrivebaseWithGamepads(boolean curveStick, boolean fieldCentric) {
        // Only on GamePad1, the right and left bumpers are speed multipliers
        speedMultiplier = 0.5;
        if (gamepad1.left_bumper) {
            speedMultiplier *= 0.5;
        }
        if (gamepad1.right_bumper) {
            speedMultiplier *= 2;
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
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -rotatedY * speedMultiplier,
                        -rotatedX * speedMultiplier
                ),
                -rot * speedMultiplier
        ));

        // Update the current pose:
        drive.updatePoseEstimate();
    }

    // In case action needs to be changed, call this function to override the previous action and run the newest action
    public void runAction(RobotAction action) {
        // Abort any previous action that still might be running:
        drive.abortActions();
        // Run the action:
        drive.runParallel(action);
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

            // Collecting Sample
            if (gamepad2.right_bumper) {
                liftPosition = LIFT_COLLECT;
                wristPosition = WRIST_IN;
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
            double slideVelocity = MAX_SLIDE_VELOCITY * gamepad2.right_stick_y;

            // DeltaTime will be the actual current time minus the currentTime of the last loop
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
            if(isCrossingNoSlideZone(liftPositionWithFudge)) {
                if(getSlidePosition() <= SLIDE_HOME_POSITION + TICKS_EPSILON) {
                    moveWrist(WRIST_IN);
                    moveSlide(SLIDE_HOME_POSITION);
                } else {
                    moveWrist(WRIST_IN);
                    moveLift(liftPositionWithFudge);
                }
            } else {
                moveWrist(wristPosition);
                moveSlide(slidePosition);
                moveLift(liftPositionWithFudge);
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



        // These telemetry.addLine() calls will inform the user of what each button does
        telemetry.addLine("Low Basket Score: Y-Button");
        telemetry.addLine("Intake Deposit: B-Button");
        telemetry.addLine("On Intake: A-Button");
        telemetry.addLine("Off Intake: X-Button: ");
        telemetry.addLine();

        telemetry.addLine("Low rung hang orientation: Up D-Pad");
        telemetry.addLine("High Chamber Orientation: Right D-Pad");
        telemetry.addLine("Fold wrist & folds arm, intake off: Left D-Pad:");
        telemetry.addLine("Ascent robot: Down D-Pad");
        telemetry.addLine();

        telemetry.addLine("Clear floor for intake: left-bumper");
        telemetry.addLine("Sample collection: right-bumper");
        telemetry.addLine("Negative fudge position: left-trigger");
        telemetry.addLine("Positive fudge position: right-trigger");

        telemetry.update();
    }
}