package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends BaseOpMode {
    private double speedMultiplier = 1;
    boolean curve = false;
    DcMotor armMotor;
    CRServo intake;
    Servo wrist;
    MecanumDrive drive;

    public boolean hasMechanisms = MecanumDrive.driveParameters == DriveParameters.COMPETITION_ROBOT
            || MecanumDrive.driveParameters == DriveParameters.FASTBOT_MECANUM;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 1;
    final double WRIST_FOLDED_OUT = 0.61;

    double wristPosition = 0;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        intake = hardwareMap.get(CRServo.class, "BootWheelServo");
        wrist = hardwareMap.get(Servo.class, "RotationServo");

        if (hasMechanisms) {
            armMotor = hardwareMap.get(DcMotor.class, "arm");
            intake = hardwareMap.get(CRServo.class, "intake");
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                speedMultiplier = 0.25;
            } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
                speedMultiplier = 0.5;
            } else {
                speedMultiplier = 1;
            }

            setCurve();

            // Set the drive motor powers according to the gamepad input:
            if (curve) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                // TODO: This is supposed to be negative, but the motor configs are messed up
                                curveStick(gamepad1.left_stick_y) * speedMultiplier,
                                // TODO: This is supposed to be negative, but the motor configs are messed up
                                curveStick(gamepad1.left_stick_x) * speedMultiplier
                        ),
                        -curveStick(gamepad1.right_stick_x) * speedMultiplier
                ));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * speedMultiplier,
                                -gamepad1.left_stick_x * speedMultiplier
                        ),
                        -gamepad1.right_stick_x * speedMultiplier
                ));
            }


            // Update the current pose:
            drive.updatePoseEstimate();

            controlMechanisms();

            telemetry.addLine("Running TeleOp!");
            telemetry.addData("Kinematic Type", kinematicType);
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.addData("Stick Curve", curve);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());

            // These telemetry.addline() calls will inform the user of what each button does

            telemetry.addLine("Low Basket Score: Y-Button");
            telemetry.addLine("Intake Deposit: B-Button");
            telemetry.addLine("On Intake: A-Button");
            telemetry.addLine("Off Intake: X-Button: ");
            telemetry.addLine();

            telemetry.addLine("Low rung hang orientation: Up D-Pad");
            telemetry.addLine("High Chamber Orientation: Right D-Pad");
            telemetry.addLine("Fold wrist & folds arm, intake off: Left D-Pad:");
            telemetry.addLine("Accent robot: Down D-Pad");
            telemetry.addLine();

            telemetry.addLine("Clear floor for intake: left-bumper");
            telemetry.addLine("Sample collection: right-bumper");
            telemetry.addLine("Negative fudge position: left-trigger");
            telemetry.addLine("Positive fudge position: right-trigger");

            telemetry.update();

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

    public void controlMechanisms() {
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

        // GamePad2 Arm Control
        if (gamepad2.a) {
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad2.x) {
            intake.setPower(INTAKE_OFF);
        } else if (gamepad2.b) {
            intake.setPower(INTAKE_DEPOSIT);
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
            setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad2.left_bumper) {
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
            armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad2.y) {
            /* This is the correct height to score the sample in the LOW BASKET */
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        } else if (gamepad2.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            intake.setPower(INTAKE_OFF);
            setPosition(WRIST_FOLDED_IN);
        } else if (gamepad2.dpad_right) {
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;
            setPosition(WRIST_FOLDED_IN);
        } else if (gamepad2.dpad_up) {
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;
            intake.setPower(INTAKE_OFF);
            setPosition(WRIST_FOLDED_IN);
        } else if (gamepad2.dpad_down) {
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            setPosition(WRIST_FOLDED_IN);
        }

        // Here we set the servo position.
        wrist.setPosition(wristPosition);

            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPosition(double position) {
        wristPosition = position;
    }

    boolean aWasPressed = false;
    public void setCurve() {
        if (!aWasPressed && gamepad1.a) {
            curve = !curve;
        }
        aWasPressed = gamepad1.a;
    }

    // Applies a curve to the joystick input to give finer control at lower speeds
    public double curveStick(double rawSpeed) {
        return Math.copySign(Math.pow(rawSpeed, 2), rawSpeed);
    }
}