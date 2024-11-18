package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class FastBotTeleOp extends BaseOpMode {
    private double speedMultiplier = 0.5;
    boolean curve = true;
    boolean fieldCentered = false;

    public double startHeading;

    public boolean hasMechanisms = MecanumDrive.driveParameters == DriveParameters.COMPETITION_ROBOT
            || MecanumDrive.driveParameters == DriveParameters.FASTBOT_MECANUM
            || MecanumDrive.driveParameters == DriveParameters.DEVBOT_MECANUM;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    private boolean xPressed;

    /** @noinspection ConstantValue*/
    /* Variables that are used to set the arm to a specific position */
    double armPositionFudgeFactor;
    boolean intakeEnabled = false;

    Vector2d driveToPos = new Vector2d(48, 48);
    @Override
    public void runOpMode() {
        // Initialize the hardware and make the robot ready
        prepareRobot();

        AutoDriveTo driveTo = new AutoDriveTo(drive);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        // Only move wrist after start
        wrist.setPosition(WRIST_FOLDED_IN);

        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

        driveTo.motionProfileWithVector(driveToPos, 0, true, packet);

        while (opModeIsActive()) {
            toggleFieldCentricity();

            controlDrivebaseWithGamepads(curve, fieldCentered);

            controlMechanismsWithGamepads();

            telemeterData();

            // 'packet' is the object used to send data to FTC Dashboard:
            packet = MecanumDrive.getTelemetryPacket();

            if (gamepad1.x && !xPressed) {

                driveTo.motionProfileWithVector(driveToPos, 0, false, packet);
            }
            xPressed = gamepad1.x;

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);
        }
    }

    public void prepareRobot() {
        prepareRobot(new Pose2d(-0, -48, Math.PI / 2));
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
        if (gamepad1.dpad_up) {
            driveAssistHanging();
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

    public void controlMechanismsWithGamepads() {
        if (hasMechanisms && !stopAllMechanisms) {
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


            // In the loop if 'a' is clicked intakeEnabled is set to true which will be stored to memory
            if (gamepad2.right_bumper) {
                intakeEnabled = true;
            }
            // In the loop if 'x' is clicked intakeEnabled is set to false which will be stored to memory
            else if (gamepad2.x) {
                intakeEnabled = false;
            }

            boolean reversed = gamepad2.b;

            // When 'b' is HELD down, it will deposit
            if (reversed) {
                intake1.setPower(INTAKE_DEPOSIT);
            }
            // When 'a' is clicked, it is TOGGLED, so it will keep collecting until another input is clicked
            else if(intakeEnabled) {
                intake1.setPower(INTAKE_COLLECT);
            }
            // When 'x' is clicked, it is TOGGLED, so it will turn off the intake until another input
            else {
                intake1.setPower(INTAKE_OFF);
            }



            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            /* Here we implement a set of if else loops to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if (gamepad2.right_bumper) {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake1.setPower(INTAKE_COLLECT);
            } else if (gamepad2.left_bumper) {
                        /* This is about 20° up from the collecting position to clear the barrier
                        Note here that we don't set the wrist position or the intake power when we
                        select this "mode", this means that the intake and wrist will continue what
                        they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            } else if (gamepad2.y) {
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                wrist.setPosition(WRIST_FOLDED_OUT);
            } else if (gamepad2.dpad_left) {
                        /* This turns off the intake, folds in the wrist, and moves the arm
                        back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake1.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_right) {
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_up) {
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake1.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_down) {
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake1.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor1.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            armMotor1.setVelocity(ARM_VELOCITY);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (stopAllMechanisms) {
            ((PwmControl) wrist).setPwmDisable();
            intake1.setPower(INTAKE_OFF);
            ((PwmControl) intake1).setPwmDisable();
        }
    }

    // Applies a curve to the joystick input to give finer control at lower speeds
    public double curveStick(double rawSpeed) {
        return Math.copySign(Math.pow(rawSpeed, 2), rawSpeed);
    }

    boolean stopAllMechanisms = false;

    public void driveAssistHanging() {
        // action of dpad up button on gamepad 2
        armMotor1.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));
        intake1.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor1.setVelocity(ARM_VELOCITY);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for the arm to reach the position
        while (armMotor1.isBusy()) {
            if (!opModeIsActive())
                break;
        }

        // action of dpad right button on gamepad 2
        armMotor1.setTargetPosition((int) (ARM_SCORE_SPECIMEN));
        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor1.setVelocity(ARM_VELOCITY);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for the arm to reach the position
        while (armMotor1.isBusy()) {
            if (!opModeIsActive())
                break;
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // move backward while pressing (dpad_down logic)
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-0.25, 0),  // move back at 0.25 power
                0
        ));

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // action of dpad down gamepad 2
        armMotor1.setTargetPosition((int) (ARM_WINCH_ROBOT));
        intake1.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        // move backward while pressing dpad_down logic
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-0.25, 0),  // move back at 0.25 power
                0
        ));

        armMotor1.setVelocity(ARM_VELOCITY);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for the arm to reach the position
        while (armMotor1.isBusy()) {
            if (!opModeIsActive())
                break;
        }

        // once that's over then action of dpad left gamepad 2
        // noinspection ConstantValue
        armMotor1.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
        intake1.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor1.setVelocity(ARM_VELOCITY);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for the arm to reach the position
        while (armMotor1.isBusy()) {
            if (!opModeIsActive())
                break;
        }

        // stop wheels
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0),
                0
        ));

        stopAllMechanisms = true;
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

        telemetry.addData("Pose (x, y, h):",
                String.format("(%.2f\", %.2f\", %.2f°)",
                        drive.pose.position.x,
                        drive.pose.position.y,
                        Math.toDegrees(drive.pose.heading.log())));

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (armMotor1.isOverCurrent()) {
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", armMotor1.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor1.getCurrentPosition());

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