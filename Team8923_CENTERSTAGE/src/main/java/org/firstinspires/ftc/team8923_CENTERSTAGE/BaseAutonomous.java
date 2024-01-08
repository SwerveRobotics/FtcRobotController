package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;


    BlueOpenCvColorDetection myBlueColorDetection = new BlueOpenCvColorDetection(this);
    RedOpenCvColorDetection myRedColorDetection = new RedOpenCvColorDetection(this);

    public void initBlueAuto() {
        telemetry.addData("Init State", "Init S  tarted");
        telemetry.update();
        // initialize the webcam and openCV pipeline
        myBlueColorDetection.init();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        initHardware();

        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }
    public void initRedAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        // initialize the webcam and openCV pipeline
        myRedColorDetection.init();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        initHardware();

        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    public void driveInches(double x, double y) {
        double xTicks = x * TICKS_PER_INCH;
        double yTicks = y * TICKS_PER_INCH;

        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += motorFL.getCurrentPosition();
        targetFR += motorFR.getCurrentPosition();
        targetBL += motorBL.getCurrentPosition();
        targetBR += motorBR.getCurrentPosition();

        motorFL.setTargetPosition((int) targetFL);
        motorFR.setTargetPosition((int) targetFR);
        motorBL.setTargetPosition((int) targetBL);
        motorBR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBL.setPower(0.5);
        motorBR.setPower(0.5);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public  void pivot(double targetHeading) {
        targetHeading *= -1;
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double angleError = targetHeading - currentAngle + startAngle;
        double motorPower;

        // while robot hasn't reached target heading
        while (Math.abs(angleError) >= BaseOpMode.ROBOT_HEADING_TOLERANCE_DEGREES && opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            angleError = targetHeading - currentAngle + startAngle;

            // prevents angle from going above 180 degrees and below -180 degrees
            // makes sure robot takes most optimal path to get to the target heading
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // proportional motor power based on angle error
            motorPower = angleError * -BaseOpMode.TURNING_KP;
            motorPower = Math.max(Math.min(Math.abs(motorPower), BaseOpMode.MAXIMUM_TURN_POWER_AUTONOMOUS), BaseOpMode.MINIMUM_TURN_POWER) * Math.signum(motorPower);

            // gives a power to each motor to make the robot pivot
            motorFL.setPower(motorPower);
            motorFR.setPower(-motorPower);
            motorBL.setPower(motorPower);
            motorBR.setPower(-motorPower);
        }

        telemetry.addData("current angle", currentAngle);
        telemetry.update();

        stopDriving();
    }

    /*public void runIntake(double power, int rotations) {
        motorIntakeWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double targetMotorIntakeWheels = rotations * TICKS_PER_REVOLUTION;
        targetMotorIntakeWheels += motorIntakeWheels.getCurrentPosition();
        motorIntakeWheels.setTargetPosition((int) targetMotorIntakeWheels);
        runtime.reset();
        motorIntakeWheels.setPower(power);
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
        }
        motorIntakeWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/

    public void runIntake(double power, int time) {
        ElapsedTime t = new ElapsedTime();
        t.startTime();
        while (t.milliseconds() < time) {
            motorIntakeWheels.setPower(-power);
        }
        motorIntakeWheels.setPower(0.0);
    }

    private void stopDriving() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
