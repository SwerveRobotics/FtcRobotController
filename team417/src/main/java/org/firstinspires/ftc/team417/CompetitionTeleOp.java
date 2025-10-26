package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class CompetitionTeleOp extends BaseOpMode {

    double FASTDRIVE_SPEED = 1.0;
    double SLOWDRIVE_SPEED = 0.5;

    ElapsedTime rightBumperTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);

        // Initialize motors, servos, LEDs
        initHardware();

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Running TeleOp!");

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            halfLinearHalfCubic(-gamepad1.left_stick_y * doSLOWMODE()),
                            halfLinearHalfCubic(-gamepad1.left_stick_x * doSLOWMODE())

                    ),
                    halfLinearHalfCubic(-gamepad1.right_stick_x * doSLOWMODE())


            ));

            // Update the current pose:

            drive.updatePoseEstimate();

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // send telemetry to FTC dashboard to graph
            packet.put("FlyWheel Speed:", launcher.getVelocity());
            packet.put("Right bumper press:", gamepad2.right_bumper ? 0 : 1000);
            packet.put("Feeder wheels:", rightFeeder.getPower() * 100);


            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);


            if (gamepad2.y) { //high speed
                launcher.setVelocity(LAUNCHER_HIGH_TARGET_VELOCITY);
                launchState = LaunchState.HIGH;

            } else if (gamepad2.a) { //slow speed
                launcher.setVelocity(LAUNCHER_LOW_TARGET_VELOCITY);
                launchState = LaunchState.LOW;

            } else if (gamepad2.x) { // sort speed
                launcher.setVelocity(LAUNCHER_SORTER_TARGET_VELOCITY);
                launchState = LaunchState.SORT;

            } else if (gamepad2.b) { //reverse
                launcher.setVelocity(LAUNCHER_REV_TARGET_VELOCITY);
                leftFeeder.setPower(REV_SPEED);
                rightFeeder.setPower(REV_SPEED);

            } else if (gamepad2.left_bumper) { // stop flywheel
                launcher.setVelocity(STOP_SPEED);
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                launchState = LaunchState.IDLE;
                CURRENT_LAUNCHSTATE = "IDLE";
            }


            /*
             * Now we call our "Launch" function.
             */
            if (rightBumperTimer.seconds() > 0.25) {
                launch(gamepad2.rightBumperWasPressed());
                rightBumperTimer.reset();
            }

            /*
             * Show the state and motor powers
             */
            telemetry.addData("State", launchState);
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("motorSpeed", launcher.getVelocity());
            telemetry.addData("FEED_TIME_SECONDS", FEED_TIME_SECONDS);
            telemetry.addData("feederSpeed", leftFeeder.getPower());

            telemetry.update();
        }
    }

    public double doSLOWMODE() {
        if (gamepad1.right_trigger != 0) {
            return -gamepad1.right_trigger + 1.1;
        } else {
            return 1;
        }
    }

    public static double halfLinearHalfCubic(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }
}
