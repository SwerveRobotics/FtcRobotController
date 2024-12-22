package org.firstinspires.ftc.team417.skidaddle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team417.SlowBotTeleOp;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

@Config
@TeleOp(name = "PathUnitTest")
public class PathUnitTest extends SlowBotTeleOp {

    public double startHeading;

    private final int ARM_COLLECT = 4830;
    private final int ARM_CLEAR_BARRIER = 4660;
    private final int ARM_COLLAPSED_INTO_ROBOT = 20;
    private final int ARM_SCORE_SPECIMEN = 3030;

    @Override
    public void runOpMode() {
        boolean pathing = false;
        double currentTime = currentTime();
        double lastTime = currentTime;
        double deltaTime;

        prepareRobot(new Pose2d(63, -63, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PoseVelocity2d currentPoseVel;

        AutoDriveTo driveTo = new AutoDriveTo(drive);

        boolean yPressed = false;
        boolean upPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            currentTime = currentTime();
            deltaTime = currentTime - lastTime;

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            currentPoseVel = drive.updatePoseEstimate();

            if (gamepad1.y && !pathing) {
                pathing = true;
                driveTo.init(new DPoint(0, -48), Math.PI/2, currentPoseVel, telemetry);
            }

            if (pathing) {
                pathing = !driveTo.linearDriveTo(currentPoseVel, deltaTime, packet, canvas);
            } else {
                // Set the drive motor powers according to the gamepad input:
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                        -gamepad1.left_stick_y / 2.0,
                        -gamepad1.left_stick_x / 2.0),
                        -gamepad1.right_stick_x / 2.0));

                controlMechanismsWithGamepads();
            }

            yPressed = gamepad1.y;
            upPressed = gamepad1.dpad_up;

            WilyWorks.updateSimulation(deltaTime);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            packet.put("x", drive.pose.position.x);
            packet.put("y", drive.pose.position.y);
            packet.put("heading", drive.pose.heading);

            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            lastTime = currentTime;
        }
    }

    public void prepareRobot(Pose2d startingPose) {
        drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, startingPose);
        initializeHardware();

        startHeading = startingPose.heading.log();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    boolean intakeEnabled = false;
    double armPositionFudgeFactor = 0;
    double armPosition = 0;

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


        // In the loop if 'a' is clicked intakeEnabled is set to true which will be stored to memory
        if (gamepad1.a) {
            intakeEnabled = true;
        }
        // In the loop if 'x' is clicked intakeEnabled is set to false which will be stored to memory
        else if (gamepad1.x) {
            intakeEnabled = false;
        }

        boolean reversed = gamepad1.b;

        // When 'b' is HELD down, it will deposit
        if (reversed) {
            intake1.setPower(INTAKE_DEPOSIT);
        }
        // When 'a' is clicked, it is TOGGLED, so it will keep collecting until another input is clicked
        else if (intakeEnabled) {
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

        armPositionFudgeFactor = FUDGE_FACTOR_LIFT * (gamepad1.right_trigger + (-gamepad1.left_trigger));

        /* Here we implement a set of if else loops to set our arm to different scoring positions.
        We check to see if a specific button is pressed, and then move the arm (and sometimes
        intake and wrist) to match. For example, if we click the right bumper we want the robot
        to start collecting. So it moves the armPosition to the ARM_COLLECT position,
        it folds out the wrist to make sure it is in the correct orientation to intake, and it
        turns the intake on to the COLLECT mode.*/

        if (gamepad1.right_bumper) {
            /* This is the intaking/collecting arm position */
            armPosition = ARM_COLLECT;
            wrist.setPosition(WRIST_OUT);
            intake1.setPower(INTAKE_COLLECT);
            intakeEnabled = true;
        } else if (gamepad1.left_bumper) {
            /* This is about 20° up from the collecting position to clear the barrier
            Note here that we don't set the wrist position or the intake power when we
            select this "mode", this means that the intake and wrist will continue what
            they were doing before we clicked left bumper. */
            armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad1.dpad_left) {
            /* This turns off the intake, folds in the wrist, and moves the arm
            back to folded inside the robot. This is also the starting configuration */
            intake1.setPower(INTAKE_OFF);
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            wrist.setPosition(WRIST_IN);
        } else if (gamepad1.dpad_right) {
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_IN);
        }

        /* Here we set the target position of our arm to match the variable that was selected
        by the driver.
        We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        liftMotor1.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

        liftMotor1.setVelocity(2120);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("armPosition", liftMotor1.getCurrentPosition());
    }
}
