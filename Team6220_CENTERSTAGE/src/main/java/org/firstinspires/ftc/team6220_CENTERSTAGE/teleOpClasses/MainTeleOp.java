package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.ExtendedDriveFeatures;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team6220_CENTERSTAGE.PersistentValues;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Utilities;

/*
This is our main teleop class for competition driving.
 */
@TeleOp(name=">>> Main TeleOp", group="amogus1")
public class MainTeleOp extends LinearOpMode {

    // define states for turning, using slides, outtaking

    // stop posting about ENUMS im TIRED of SEEING IT
    // my friends on discord send me ENUMS
    // in person its hecking ENUMS
    // i was in this server right?
    // and ALLLL the channels were just "STATE MACHINE"
    // ENUMS
    // GRAAAAHH

    TurnStates curTurningState = TurnStates.TURNING_MANUAL;
    enum TurnStates {
        TURNING_MANUAL,
        TURNING_90,
        TURNING_FIELD_CENTRIC
    }

    SlideStates curSlideState = SlideStates.SLIDES_MANUAL;
    enum SlideStates {
        SLIDES_MANUAL,
        SLIDES_TO_POSITION, // for slide preset buttons
    }

    // drive powers, read from input and then manipulated every loop
    double drivePowerX = 0.0;
    double drivePowerY = 0.0;
    double turnPower = 0.0;
    double intakePower = 0.0;
    double slidesPower = 0.0;
    double slidesTargetPos = 0.0;

    // 1.0 represents 100% speed; applied too drive and turn
    double slowMultiplier = 0.0;

    // holds heading from imu read which is done in roadrunner's mecanum drive class for us
    double currentHeading = 0.0;
    double targetHeading = 0.0;

    // tracks whether the dumper is extended or not (best way I could think of to do this)
    boolean dumperExtended;
    boolean justResetFieldCentric = false;

    // represents the driving direction vector that is given to roadrunner
    DriveVector driveVector = new DriveVector(0, 0);

    // manages the current inbar position (intake bar)
    double inbarPos = Constants.INBAR_MAX_POSITION;

    // useful groups of keycodes
    final GamepadKeys.Button[] BUMPER_KEYCODES = {
            GamepadKeys.Button.LEFT_BUMPER,
            GamepadKeys.Button.RIGHT_BUMPER
    };

    final GamepadKeys.Button[] DPAD_KEYCODES = {
            GamepadKeys.Button.DPAD_UP,
            GamepadKeys.Button.DPAD_LEFT,
            GamepadKeys.Button.DPAD_DOWN,
            GamepadKeys.Button.DPAD_RIGHT
    };

    MecanumDrive drive;
    ExtendedDriveFeatures exDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        exDrive = new ExtendedDriveFeatures(drive);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        // don't reset so that the zero stays accurate between programs
        /*
        // Reset encoders of slide motor
        if (!drive.isDevBot) { // is competition bot
            drive.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
         */


        // stops the imu from unwanted continuing of previous tracking
        drive.imu.resetYaw();

        boolean runningFieldCentric = true;

        double headingOffset = 0.0;
        // if there is a non-zero saved heading
        if (PersistentValues.HEADING_OFFSET != 0.0) {
            telemetry.addLine(String.format("Detected a saved heading offset of %.2f°", PersistentValues.HEADING_OFFSET));
            telemetry.addLine();
            telemetry.addLine("[A] : Use it");
        } else {
            telemetry.addLine("No saved heading offset detected.");
            telemetry.addLine();
        }
        telemetry.addLine("[RUN] or [B] : Skip menu, make new forward");
        telemetry.addLine();
        telemetry.addLine("[X]+[Y] : Run without field centric mode (only for emergencies!)");
        telemetry.update();

        // take input for using offset or not
        boolean waitingForResponse = true;
        while (!isStopRequested() && waitingForResponse && !opModeIsActive()) {
            if ((gp1.getButton(GamepadKeys.Button.A))
                    && PersistentValues.HEADING_OFFSET != 0.0) {
                waitingForResponse = false;
                headingOffset = PersistentValues.HEADING_OFFSET;
                telemetry.addLine(String.format("Using heading offset of %.2f", headingOffset));

            } else if (gp1.getButton(GamepadKeys.Button.B)) {
                waitingForResponse = false;
                telemetry.addLine("(Not using heading offset)");

            } else if (gp1.getButton(GamepadKeys.Button.X) && gp1.getButton(GamepadKeys.Button.Y)) {
                waitingForResponse = false;
                runningFieldCentric = false;
                telemetry.addLine("Disabled field centric mode!");
            }
        }

        telemetry.addLine();
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        // move inbar down to safe position that won't rub the belts
        drive.intakeServo.setPosition(Constants.INBAR_MAX_POSITION);

        while (opModeIsActive()) {
            // update gamepads and read inputs
            gp1.readButtons();
            gp2.readButtons();


            // manage turn input and state:

            turnPower = gp1.getRightX();

            // get heading from imu in degrees
            currentHeading = Utilities.limitAngle( drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + headingOffset );

            // update turn state
            if (Math.abs(turnPower) > Constants.TURN_STICK_DEADZONE) {
                targetHeading = 0.0;
                curTurningState = TurnStates.TURNING_MANUAL;

            } else if (Utilities.justPressedAny(gp1, BUMPER_KEYCODES) > -1) {
                // sets target to 90 added to current heading
                // if already turning 90, add to target instead of current
                // also limits target angle between -180 and 180
                targetHeading = Utilities.limitAngle(
                    (curTurningState == TurnStates.TURNING_90 ? targetHeading : currentHeading) +
                    (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? 90 : -90)
                );
                curTurningState = TurnStates.TURNING_90;

            } else if (Utilities.justPressedAny(gp1, DPAD_KEYCODES) > -1) {
                // find which dpad button was pressed, use its index * 90 degrees as target
                // up: 0*90=0, left: 1*90=90, down: 2*90=180, right: 3*90=270 (-> -90)
                targetHeading = Utilities.limitAngle(Utilities.justPressedAny(gp1, DPAD_KEYCODES) * 90.0);
                curTurningState = TurnStates.TURNING_FIELD_CENTRIC;

            } else if (Math.abs(Utilities.shortestDifference(currentHeading, targetHeading)) < Constants.TELEOP_MIN_HEADING_ACCURACY) {
                curTurningState = TurnStates.TURNING_MANUAL;
            }

            // use turn state
            switch (curTurningState) {
                case TURNING_MANUAL:
                    turnPower *= Constants.TURN_POWER_MULTIPLIER;
                    break;
                case TURNING_90:
                    // falls through to field centric
                case TURNING_FIELD_CENTRIC:
                    // https://www.desmos.com/calculator/zdsmmtbnwf (updated)
                    // flips difference so that it mimics turn stick directions
                    turnPower = Utilities.clamp(-Utilities.shortestDifference(currentHeading, targetHeading) / 90.0);
            }


            // calculate fancy drive vector:

            // raw base drive vector (is robot centric)
            // note: y (forward) is already flipped in the gamepadEx .get<stick>Y()
            // implementation, so it doesn't need to be flipped here like drivePowerX is
            drivePowerX = gp1.getLeftX();// * Constants.DRIVE_POWER_X_MULTIPLIER;
            drivePowerY = gp1.getLeftY();// * Constants.DRIVE_POWER_Y_MULTIPLIER;
            driveVector.setXY(drivePowerY, -drivePowerX);

            // field centric is enabled by default and has to be disabled at the beginning of teleop
            if (runningFieldCentric) {
                // convert to local robot centric vector equivalent of field centric vector
                // (do field centric driving)
                driveVector.rotate(-currentHeading);

                // if holding the A button gp1 lock drive vector to heading direction
                if (gp1.getButton(GamepadKeys.Button.A)) {
                    // clears any strafing and leaves only forward/back x driving
                    driveVector.y = 0;
                }
            }


            // finish processing drive inputs:

            // clamp powers between -1.0 and 1.0 just in case
            turnPower = Utilities.clamp(turnPower);
            driveVector.limit(); // magnitude <= 1

            driveVector.times(Constants.DRIVE_FORWARD_MULTIPLIER, Constants.DRIVE_STRAFE_MULTIPLIER);

            // apply calculated slow multiplier (ranges from 1 to full multiplier < 1)
            slowMultiplier = Utilities.getSlowMultiplier( gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), Constants.SLOWMODE_MULTIPLIER );

            turnPower *= slowMultiplier;
            driveVector.times(slowMultiplier);


            // apply drive instructions
            // note: turnPower is inverted to align with negative = clockwise
            drive.setDrivePowers(new PoseVelocity2d(driveVector.toVector2d(), -turnPower));

            // update roadrunner's estimate of the robot's position
            drive.updatePoseEstimate();


            // necessary feature
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON) || gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                telemetry.speak(Math.random() > 0.1 ? "HONK!" : "quack.");
            }


            // reset field centric forward hotkey
            if (gp1.getButton(GamepadKeys.Button.B) && gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) && !justResetFieldCentric) {
                headingOffset = -currentHeading;
                telemetry.speak("Resetting field centric forward!");
                justResetFieldCentric = true;
            } else if (!gp1.getButton(GamepadKeys.Button.B) && !gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                justResetFieldCentric = false;
            }


            // if it's the competition bot do slides, outtake, etc.
            if (!drive.isDevBot) {

                // run intake:

                // get intake power
                intakePower =
                        Math.max(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
                        * (gp1.getButton(GamepadKeys.Button.X) || gp2.getButton(GamepadKeys.Button.X) ? -1 : 1)
                        * Constants.INTAKE_POWER_MULTIPLIER;

                // apply intake instructions
                drive.intakeMotor.setPower(-intakePower); // will self stop with 0 power


                // run inbar (aka intakeServo, intake bar):

                // highest or lowest inbar positions
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    inbarPos = Constants.INBAR_MAX_POSITION;
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    inbarPos = Constants.INBAR_MIN_POSITION;

                // manual up and down (limited to max and min pos)
                } else if (gp2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    inbarPos = Math.min(Constants.INBAR_MAX_POSITION, inbarPos + Constants.INBAR_MANUAL_RATE);
                } else if (gp2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    inbarPos = Math.max(Constants.INBAR_MIN_POSITION, inbarPos - Constants.INBAR_MANUAL_RATE);
                }

                // apply inbar position
                drive.intakeServo.setPosition(inbarPos);

              
                // run slides:        

                // slide presets
                // preset down
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    switch (curSlideState) {
                        case SLIDES_MANUAL:
                            // take the current position
                            slidesTargetPos = drive.slideMotor.getCurrentPosition();
                            // fall through to set target to next down
                        case SLIDES_TO_POSITION:
                            curSlideState = SlideStates.SLIDES_TO_POSITION;
                            // next down from target (if we've already set target, it goes one more next!)
                            slidesTargetPos = Utilities.positionDown(slidesTargetPos, Constants.SLIDE_TELEOP_POSITIONS, Constants.SLIDE_NEAR_PRESET_RANGE);
                    }
                // preset up
                } else if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    switch (curSlideState) {
                        case SLIDES_MANUAL:
                            // take the current position
                            slidesTargetPos = drive.slideMotor.getCurrentPosition();
                            // fall through to set target to next up
                        case SLIDES_TO_POSITION:
                            curSlideState = SlideStates.SLIDES_TO_POSITION;
                            // next down from target (if we've already set target, it goes one more next!)
                            slidesTargetPos = Utilities.positionUp(slidesTargetPos, Constants.SLIDE_TELEOP_POSITIONS, Constants.SLIDE_NEAR_PRESET_RANGE);
                    }
                // switch to manual if slides stick is given input or slides override is active
                } else if (Math.abs(gp2.getLeftY()) > Constants.SLIDES_STICK_DEADZONE || gp2.getButton(GamepadKeys.Button.Y)) {
                    curSlideState = SlideStates.SLIDES_MANUAL;
                }

                switch (curSlideState) {
                    // if the slides are being moved by stick input
                    case SLIDES_MANUAL:

                        slidesPower = gp2.getLeftY() * Constants.SLIDE_MANUAL_MULTIPLIER;

                        // if doing manual override let any input through
                        if (gp2.getButton(GamepadKeys.Button.Y)) {
                            exDrive.moveSlides(slidesPower);
                        // if stopping manual override (releasing the button), set the new zero
                        } else if (gp2.wasJustReleased(GamepadKeys.Button.Y)) {
                            drive.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            drive.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        // otherwise do normal slide input with lower and upper limits

                        // if going up and not within tolerance of or above max position
                        // or if going down and not within tolerance of or below min position...
                        } else if ((slidesPower > 0 && drive.slideMotor.getCurrentPosition() < Constants.SLIDE_MAX_POSITION - Constants.SLIDE_LIMIT_TOLERANCE)
                                || (slidesPower < 0 && drive.slideMotor.getCurrentPosition() > Constants.SLIDE_LIMIT_TOLERANCE)) {
                            // ...then apply slides power
                            exDrive.moveSlides(slidesPower);
                        } else {
                            exDrive.moveSlides(0);
                        }
                        break;

                    // if the slides are being moved by 
                    case SLIDES_TO_POSITION:
                        // moves slides to target, if it returns false it has reached target
                        if (!exDrive.moveSlidesToPosition((int)slidesTargetPos)) {
                            // so stop moving to position
                            curSlideState = SlideStates.SLIDES_MANUAL;
                        }
                        break;
                }


                // run outtake:
               
                // toggle outtake position
                if(gp2.wasJustReleased(GamepadKeys.Button.B)) {
                    if (dumperExtended) {
                        drive.dumperServo.setPosition(Constants.DUMPER_POSITIONS[0]);
                        dumperExtended = false;
                    } else {
                        drive.dumperServo.setPosition(Constants.DUMPER_POSITIONS[1]);
                        dumperExtended = true;
                    }
                }

                // drive belt when gp2 [A] is pressed
                if (gp2.isDown(GamepadKeys.Button.A)) {
                    drive.outtakeConveyor.setPower(-Constants.OUTTAKE_CONVEYOR_POWER);
                // otherwise if intake is running
                } else if (Math.abs(intakePower) > 0) {
                    // follow intake power
                    drive.outtakeConveyor.setPower(-intakePower);
                // but if nothing's happening power should be zero
                } else {
                    drive.outtakeConveyor.setPower(0);
                }
              

                // opens gate when gp2 [A] is pressed
                if (gp2.getButton(GamepadKeys.Button.A)) {
                    drive.outtakeGate.setPosition(Constants.OUTTAKE_GATE_POSITIONS[0]);
                } else {
                    drive.outtakeGate.setPosition(Constants.OUTTAKE_GATE_POSITIONS[1]);
                }

                // drive Drone Launcher
                if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                    drive.droneLauncherServo.setPosition(Constants.DRONE_LAUNCHER_SERVO_LAUNCHING_POS);
                } else if (gp1.wasJustReleased(GamepadKeys.Button.Y)){
                    drive.droneLauncherServo.setPosition(Constants.DRONE_LAUNCHER_SERVO_PRIMED_POS);
                }
            }


            // telemetry

            telemetry.addData("driving field centric", runningFieldCentric);
            telemetry.addData("imu reading", currentHeading);
            telemetry.addLine();

            if (!drive.isDevBot) {
                telemetry.addData("slides pos", drive.slideMotor.getCurrentPosition());
                telemetry.addData("slide state", curSlideState);
                telemetry.addLine();
            }


            // Code added to draw the pose:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, drive.pose);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);

            telemetry.update();
        }

        // save the current heading in case we ended accidentally
        // and can't reset robot position/heading
        PersistentValues.HEADING_OFFSET = currentHeading;
    }

}
