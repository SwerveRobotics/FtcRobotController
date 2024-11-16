package org.firstinspires.ftc.team6220;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team6220.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team6220.roadrunner.RobotAction;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {

    class SlideAction extends RobotAction {
        final double EPSILON = 3; // Ticks

        int targetMotorPosition;
        double targetMotorPower;

        public SlideAction(int targetMotorPosition, double targetMotorPower) {
            this.targetMotorPosition = targetMotorPosition;
            this.targetMotorPower = targetMotorPower;
        }

        @Override
        public boolean run(double elapsedTime) {
            setSlideMotorPosition(targetMotorPosition);
            setSlideMotorPower(targetMotorPower);
            double slideHeight = Math.abs(getSlideMotorPosition() - targetMotorPosition);

            // Return 'true' to call again when not at target position yet:
            return (slideHeight > EPSILON);
        }
    }

    // Arm positions, in ticks:
    final int ARM_HOME = 0;
    final int ARM_COLLECT = 2000;
    final int ARM_SCORE_BASKET = 1500;

    final double INTAKE_COLLECT = 0.5;
    final double INTAKE_DEPOSIT = -1.0;
    final double INTAKE_OFF = 0.0;

    // Mechanism state:
    DcMotorEx slidesMotor;
    CRServo dumperServo;
    SlideAndDumperSimulator slideAndDumperSim = new SlideAndDumperSimulator();

    // Helper function for settings the arm position, in ticks:
    void setSlideMotorPosition(int targetMotorPosition) {
        if ((targetMotorPosition < ARM_HOME) || (targetMotorPosition > ARM_COLLECT)) {
            throw new IllegalArgumentException("Invalid setArmPosition() request.");
        }
        slidesMotor.setTargetPosition(targetMotorPosition);
        slideAndDumperSim.setSlidePosition(targetMotorPosition);
    }
    // Helper function for querying the arm position, in ticks. Uses the simulator when running
    // under Wily Works.
    int getSlideMotorPosition() {
        if (WilyWorks.isSimulating)
            return slideAndDumperSim.getSlidePosition();
        else
            return slidesMotor.getCurrentPosition();
    }
    // Helper function to set the power on the intake.
    void setSlideMotorPower(double power) { // Positive is intake, negative is out-take, zero is stop
        if ((power > INTAKE_COLLECT) || (power < INTAKE_DEPOSIT)) {
            throw new IllegalArgumentException("Invalid setIntakePower() request.");
        }
        slidesMotor.setPower(power);
        slideAndDumperSim.setDumperPower(power);
    }
    // Class for simulating the arm and drawing it to the field view of FTC Dashboard.
    static class SlideAndDumperSimulator {
        final double VELOCITY = 3000; // The arm rotates at about 2000 ticks per second
        final double SLIDE_BASE_HEIGHT = 13.77;
        double currentSlidePosition = 0; // Current arm motor position, in ticks
        double targetSlidePosition = 0; // Target arm motor position, in ticks
        double dumperPower = 0; // Current intake servo power, positive is intake, negative is deposit
        double previousTime; // Seconds

        // Return the time, in seconds:
        double time() {
            return nanoTime() * 1e-9;
        }

        // These simulate the 'real' helper functions:
        void setSlidePosition(int targetInTicks) {
            this.targetSlidePosition = targetInTicks;
            previousTime = time();
        }
        int getSlidePosition() {
            return (int) currentSlidePosition;
        }
        void setDumperPower(double power) {
            dumperPower = power;
        }

        // This method updates our simulation and draws the arm's state on FTC dashboard.
        void update(Canvas canvas, Pose2d pose) {
            // Figure out the change in time from the last update call, in seconds:
            double currentTime = time();
            double dt = currentTime - previousTime;
            previousTime = currentTime;

            // Advance the arm's position according to our simple simulation:
            double remainingTicks = targetSlidePosition - currentSlidePosition;
            double magnitudeToAdvance = Math.min(VELOCITY * dt, Math.abs(remainingTicks));
            currentSlidePosition += Math.copySign(magnitudeToAdvance, remainingTicks);

            // Draw a side view of the arm in the middle of the FTC Dashboard field, on top of
            // the submersible floor. We use the same coordinates for this as when we're
            // calculating the coordinates for the robot, so here the submersible base where
            // we'll be drawing goes from (-12, 22) to (12, -22), in inches:
            canvas.setFill("#808080"); // Set the fill color to grey
            canvas.fillRect(-12, 22, 24, -44); // Erase the submersible base

            canvas.setStroke("#000000"); // Draw the robot base in black
            canvas.strokeLine(0, 0, 18, 0); // The line representing the robot base

            canvas.setFill("#000000"); // Draw the wheels in black
            canvas.fillCircle(3, 0, 2); // Wheel #1
            canvas.fillCircle(15, 0, 2); // Wheel #2

            double slideHeight = SLIDE_BASE_HEIGHT;
            // Draw a line representing the arm. It starts at the origin and has an angle that
            // is calculated as a fraction of the end angles:
            // Add more lines to change slide height
            canvas.strokeLine(18, 0, 18, slideHeight);
        }
    }
}
