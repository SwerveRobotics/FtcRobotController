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

@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class SlowBotTeleOp extends BaseOpModeSlowBot {
    private double speedMultiplier = 1;
    boolean curve = true;
    boolean fieldCentered = false;

    public double startHeading;

    /* A number in degrees that the triggers can adjust the arm position by */
    // TODO: needs tuning
    final double FUDGE_FACTOR = 200;

    /**
     * @noinspection ConstantValue
     */
    /* Variables that are used to set the arm to a specific position */
    double liftPositionFudgeFactor;
    double slidePositionFudgeFactor;

    boolean intakeEnabled = false;

    // These are variables that will be used for individual control actions
    double slidePosition = SLIDE_HOME_POSITION;
    double wristPosition = WRIST_IN;
    double liftPosition = LIFT_REST_POSITION;


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

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    public void controlDrivebaseWithGamepads(boolean curveStick, boolean fieldCentric) {
        // If the left bumper is pressed, slow down, and if the right bumper is pressed, speed up.
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

        // Press the D-Pad left button ONCE (do not hold)
        //TODO: FIX LATER


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

             // Collecting Sample
            if (gamepad2.right_bumper) {
                liftPosition = LIFT_COLLECT;
                wristPosition = WRIST_IN;
                slidePosition = SLIDE_COLLECT;
                intakeEnabled = true;
            }

            // Retract linear slides
            if (gamepad2.dpad_left) {
                liftPosition = LIFT_REST_POSITION;
                wristPosition = WRIST_IN;
                slidePosition = SLIDE_HOME_POSITION;
            }

            // Correct height for specimen (High)
            if (gamepad2.dpad_right) {
                liftPosition = LIFT_SCORE_HIGH_SPECIMEN;
                wristPosition = WRIST_IN;
                slidePosition = SLIDE_HOME_POSITION;
            }

            // Clear floor barrier for intake
            if (gamepad2.left_bumper) {
                slidePosition = SLIDE_COLLECT;
                wristPosition = WRIST_IN;
                liftPosition = LIFT_COLLECT;
            }

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

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            liftPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            /* Here we implement a set of if else loops to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            moveLift(liftPosition + liftPositionFudgeFactor);
            moveSlide(slidePosition + slidePositionFudgeFactor);
//           armMotor.setVelocity(ARM_VELOCITY);
//           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // If lift is travelling through 'no mans land', pull in arm and wrist, then perform the lift action.
            // Else, perform all the actions
            if(isCrossingNoSlideZone(liftPosition)) {
                if(getSlidePosition() <= SLIDE_HOME_POSITION + EPSILON) {
                    moveWrist(WRIST_IN);
                    moveSlide(SLIDE_HOME_POSITION);
                } else {
                    moveWrist(WRIST_IN);
                    moveLift(liftPosition);
                }
            } else {
                moveWrist(wristPosition);
                moveSlide(slidePosition);
                moveLift(liftPosition);
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