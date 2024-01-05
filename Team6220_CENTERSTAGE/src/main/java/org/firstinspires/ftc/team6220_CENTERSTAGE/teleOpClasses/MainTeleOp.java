package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.ExtendedDriveFeatures;
import org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu.MenuInput;
import org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu.TextMenu;
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
@TeleOp(name="Main TeleOp", group="amogus1")
public class MainTeleOp extends LinearOpMode {

    // define states for turning, using slides, outtaking

    // stop posting about ENUMS im TIRED of SEEING IT
    // my friends on discord send me ENUMS
    // in person its hecking ENUMS
    // i was in this server right?
    // and ALLLL the channels were just "STATE MACHINE"
    // ENUMS
    // GRAAAAHH

    private int slidePreset = 0;
    private int intakePreset = Constants.INTAKE_POSITIONS.length - 1;
    private boolean resetProced = false;

    TurnStates curTurningState = TurnStates.TURNING_MANUAL;
    enum TurnStates {
        TURNING_MANUAL,
        TURNING_90,
        TURNING_FIELD_CENTRIC
    }

    // drive powers, read from input and then manipulated every loop
    double drivePowerX = 0.0;
    double drivePowerY = 0.0;
    double turnPower = 0.0;
    double intakePower = 0.0;

    // 1.0 represents 100% speed; applied too drive and turn
    double slowMultiplier = 0.0;

    // holds heading from imu read which is done in roadrunner's mecanum drive class for us
    double currentHeading = 0.0;
    double targetHeading = 0.0;

    // represents the driving direction vector that is given to roadrunner
    DriveVector driveVector = new DriveVector(0, 0);

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

        boolean hasDroneLaunched = false;

        // Reset encoders of slide motor
        if (!drive.isDevBot) { // is competition bot
            drive.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // quick menu for using persistent heading or not
        TextMenu menu = new TextMenu();
        MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);

        drive.imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            // update gamepads and read inputs
            gp1.readButtons();
            gp2.readButtons();

            turnPower = gp1.getRightX();

            // get heading from imu in degrees
            currentHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);// + PersistentValues.HEADING_OFFSET;


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


            // apply calculated slow multiplier (ranges from 1 to full multiplier)
            slowMultiplier = Utilities.getSlowMultiplier( gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), Constants.SLOWMODE_MULTIPLIER );
            //drivePowerX *= slowMultiplier;
            //drivePowerY *= slowMultiplier;
            turnPower *= slowMultiplier;

            // clamp powers between -1.0 and 1.0 just in case
            drivePowerX = Utilities.clamp(drivePowerX);
            drivePowerY = Utilities.clamp(drivePowerY);
            turnPower = Utilities.clamp(turnPower);


            // calculate fancy drive vector:

            // raw base drive vector (is robot centric)
            // note: y (forward) is already flipped in the gamepadEx .get<stick>Y()
            // implementation, so it doesn't need to be flipped here like drivePowerX is
            drivePowerX = gp1.getLeftX();// * Constants.DRIVE_POWER_X_MULTIPLIER;
            drivePowerY = gp1.getLeftY();// * Constants.DRIVE_POWER_Y_MULTIPLIER;
            driveVector.setXY(drivePowerY, -drivePowerX);
            double mag1 = driveVector.magnitude();

            // convert to local robot centric vector equivalent of field centric vector
            // (do field centric driving)
            driveVector.rotate(-currentHeading);

            // if holding the A button gp1 lock drive vector to heading direction
            if (gp1.getButton(GamepadKeys.Button.A)) {
                // clears any strafing and leaves only forward/back x driving
                driveVector.y = 0;
            }

            // apply drive instructions
            // note: turnPower is inverted to align with negative = clockwise
            double mag2 = driveVector.magnitude();
            drive.setDrivePowers(new PoseVelocity2d(driveVector.toVector2d().times(slowMultiplier), -turnPower));

            // update roadrunner's estimate of the robot's position
            drive.updatePoseEstimate();

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                telemetry.speak("HONK!");
            }

            // if it's the competition bot do slides, outtake, etc.
            if (!drive.isDevBot) {

                // get intake power
                // max function means positive (intake in) will overpower negative (intake out)
                // note that trigger input (intake in) will range from 0 to 1 (and then be scaled)
                intakePower = Math.max(
                        gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        gp1.getButton(GamepadKeys.Button.X) ? -1 : 0
                ) * Constants.INTAKE_POWER_MULTIPLIER;

                // apply intake instructions
                drive.intakeMotor.setPower(intakePower); // will self stop with 0 power

                /*

                // drive Drone Launcher
                if (gp2.wasJustPressed(GamepadKeys.Button.Y) && !hasDroneLaunched) {
                    drive.droneServo.setPosition(Constants.DRONE_SERVO_LAUNCHING_POS);
                } else if (gp2.wasJustReleased(GamepadKeys.Button.Y)){
                    drive.droneServo.setPosition(Constants.DRONE_SERVO_PRIMED_POS);
                    hasDroneLaunched = true;
                }

                // Slides stuff
                if (gp2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    exDrive.moveSlides(-0.5);
                } else if (gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    exDrive.moveSlides(0.5);
                } else {
                    exDrive.moveSlides(0);
                }

                // move intake bar down to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                    if (intakePreset > 0) {
                        intakePreset--;
                    }

                // move intake bar up to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                    if (intakePreset < Constants.INTAKE_POSITIONS.length - 1) {
                        intakePreset++;
                    }

                // move intake bar down to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                    intakePreset = 0;

                // move intake bar up to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                    intakePreset = Constants.INTAKE_POSITIONS.length - 1;

                // Moves the suspension arm (will be added in 3-5 business days) into position using drone button only after drone has launched
                if (gp2.wasJustPressed(GamepadKeys.Button.Y) && hasDroneLaunched) {
                    telemetry.addLine("CONFIRM HANG INITIALIZATION ACTUATION DO THE THING");
                    if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                        telemetry.addLine("SCHMOVING AND GROOVING (HOPE THAT WAS ON PURPOSE (SPRINGLOCKS ENGAGED))");
                        drive.moveSuspensionArmPreset(300); // moves arm up (ready to engage yoinky sploinky)
                        // add line for suspension actuator lock thing (idk what it will look like yet)
                        drive.moveSuspensionArmPreset(100); // move it back down, bringing robot up after it locked in
                    }
                }


                // drive the servo to position set by dpad using above code
                drive.intakeServo.setPosition(Constants.INTAKE_POSITIONS[intakePreset]);

                // go go gadget intake motor
                drive.intakeMotor.setPower(-gp2.getLeftY());

                // Sets everything back to constant positions if slides are at the bottom
                if ((slidePreset == 0) && resetProced) {
                    drive.dumperServo.setPosition(Constants.DUMPER_INITIALIZATION_POS);
                    drive.pixelLatchBack.setPosition(Constants.PIXEL_LATCH_POSITIONS[0]);
                    drive.pixelLatchFront.setPosition(Constants.PIXEL_LATCH_POSITIONS[1]);
                    resetProced = true;
                }

                // Manual control for the dumper
                drive.dumperServo.setPosition(1 - (0.35 - Math.min(0.0, gp2.getRightY()) * 0.35));

                // Determines whether to open or close the front pixel latch
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                    drive.pixelLatchFront.setPosition(Constants.PIXEL_LATCH_POSITIONS[1]);
                } else if (gp2.wasJustReleased(GamepadKeys.Button.A)) {
                    drive.pixelLatchFront.setPosition(Constants.PIXEL_LATCH_POSITIONS[0]);
                }

                // Determines whether to open or close the back pixel latch
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    drive.pixelLatchBack.setPosition(Constants.PIXEL_LATCH_POSITIONS[1]);
                } else if (gp2.wasJustReleased(GamepadKeys.Button.B)) {
                    drive.pixelLatchBack.setPosition(Constants.PIXEL_LATCH_POSITIONS[0]);
                }
                */
            }


            // telemetry
            /* telemetry.addData("current turning state", curTurningState);
            telemetry.addData("target heading", targetHeading);
            telemetry.addData("turn power", turnPower);
            telemetry.addData("drive power X", drivePowerX);
            telemetry.addData("drive power Y", drivePowerY);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("intake preset", intakePreset); */

            if (!drive.isDevBot) {
                //telemetry.addData("slide target", slidePreset);
                //telemetry.addData("slide encoder pos", drive.slideMotor.getCurrentPosition());
            }
            telemetry.addData("mag1", mag1);
            telemetry.addData("mag2", mag2);
            telemetry.addData("mag diff", mag1 - mag2);
            telemetry.addData("driveVectorX", driveVector.x);
            telemetry.addData("driveVectorY", driveVector.y);
            telemetry.addData("imu reading", currentHeading);
            //telemetry.addData("pose x", drive.pose.position.x);
            //telemetry.addData("pose y", drive.pose.position.y);
            //telemetry.addData("pose heading", drive.pose.heading);

            telemetry.update();
        }
    }

}
