package org.firstinspires.ftc.team417;

import static java.lang.System.nanoTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team417.roadrunner.KinematicType;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;
@Disabled
abstract public class BaseOpModeSlowBot extends LinearOpMode {
    static MecanumDrive drive;

    // TODO: implement this
    final static double INTAKE_DEPOSIT = 0.0;
    final static double INTAKE_COLLECT = 0.0;
    final static double INTAKE_OFF = 0.0;

    final static double LIFT_MAX = 240;
    final static double LIFT_SCORE_HIGH_BASKET = 240;
    final static double LIFT_SCORE_HIGH_SPECIMEN = 0.0;
    final static double LIFT_SCORE_LOW_BASKET = 0.0;
    final static double LIFT_COLLECT = 0.0;
    final static double LIFT_MIN = 0.0;
    final static double LIFT_CLEAR_BARRIER = 0.0;
    final static double LIFT_HOME_POSITION = 0;
    final static double LIFT_NO_SLIDE_ZONE_MIN = 0;
    final static double LIFT_NO_SLIDE_ZONE_MAX = 0;

    public static double f_coefficient = 0.5;

    final static double SLIDE_MAX = 1.0;
    final static double SLIDE_COLLECT = 0.0;
    final static double SLIDE_SCORE_IN_BASKET = 0;
    final static double SLIDE_MIN = 0.0;
    final static double SLIDE_HOME_POSITION = 0;

    final static double WRIST_MAX = 0.0;
    final static double WRIST_OUT = 0.0;
    final static double WRIST_IN = 0.0;
    final static double WRIST_MIN = 0.0;

    public final static double XDRIVE_Y_SCORE_POSE = 33;

    // This provides an error tolerance for lift and slide
    final static double TICKS_EPSILON = 3.00;

    // RC 17.50
    // DEV 17.75
    final static double ROBOT_LENGTH = 17.50;
    // RC 16.50
    // DEV 18.50
    final static double ROBOT_WIDTH = 16.50;

    //motors
    static CRServo intake1;
    static CRServo intake2;
    static Servo wrist;
    static DcMotorEx liftMotor1;
    static DcMotorEx liftMotor2;
    static DcMotorEx slideMotor;

    class ControlAction extends RobotAction {
        double targetSlidePosition;
        double targetWristPosition;
        double targetLiftPosition;

        public ControlAction(double targetSlidePosition, double targetWristPosition, double targetLiftPosition) {
            this.targetSlidePosition = targetSlidePosition;
            this.targetLiftPosition = targetLiftPosition;
            this.targetWristPosition = targetWristPosition;
        }

        @Override
        public boolean run(double elapsedTime) {
            // Once lift is ABOVE the no slide zone, move the slide & wrist out at the same time
            if(isCrossingNoSlideZone(targetLiftPosition)) {
                if(getSlidePosition() <= SLIDE_HOME_POSITION + TICKS_EPSILON) {
                    moveWrist(WRIST_IN);
                    moveSlide(SLIDE_HOME_POSITION);
                } else {
                    moveWrist(WRIST_IN);
                    moveLift(targetLiftPosition);
                }
            } else {
                moveWrist(targetWristPosition);
                moveSlide(targetSlidePosition);
                moveLift(targetLiftPosition);
            }

            // Buffers for target lift & slide position
            double liftError = Math.abs(getLiftPosition() - targetLiftPosition);
            double slideError = Math.abs(getSlidePosition() - targetSlidePosition);

            // TODO: Check if the EPSILON is different for either of the errors
            // Checks if the slide or the lift is in the correct spot
            if (liftError < TICKS_EPSILON || slideError < TICKS_EPSILON) {
                return false;
            }
            return false;
        }
    }

    public boolean isCrossingNoSlideZone(double targetLiftPosition){
        return ((targetLiftPosition > LIFT_NO_SLIDE_ZONE_MAX && getLiftPosition() < LIFT_NO_SLIDE_ZONE_MAX) ||
                (targetLiftPosition < LIFT_NO_SLIDE_ZONE_MIN && getLiftPosition() > LIFT_NO_SLIDE_ZONE_MIN));
    }

    // This helper method controls the linear slides and tells motor to go to desired position in ticks
    public void moveSlide(double positionInTicks){
        if (slideMotor != null) {
            if(positionInTicks >= SLIDE_MIN && positionInTicks <= SLIDE_MAX){
                slideMotor.setTargetPosition((int) positionInTicks);
            }
        }
    }

    // This method controls the 4bar to desired height
    static double f_value = 0.5;
    public void moveLift(double heightInTicks) {
        telemetry.addLine(String.format("Height in ticks: %.1f", heightInTicks));
        if (heightInTicks >= LIFT_MIN && heightInTicks <= LIFT_MAX) {
            double velocity;
            if(getLiftPosition() > heightInTicks){
                // lift going down
                velocity = 200;
            } else {
                velocity = 2120;
            }

            if (liftMotor1 != null) {
                liftMotor1.setPower(1.0);
                liftMotor1.setTargetPosition((int) heightInTicks);
                liftMotor1.setVelocity(velocity);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (liftMotor2 != null) {
                liftMotor2.setPower(1.0);
                liftMotor2.setTargetPosition((int) heightInTicks);
                liftMotor2.setVelocity(velocity);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }
    }

    // This method moves the wrist up and down based on desired position
    public void moveWrist(double wristPosition){
        if (wrist != null) {
            if(wristPosition >= WRIST_MIN && wristPosition <= WRIST_MAX){
                wrist.setPosition(wristPosition);
            }
        }
    }

    public double getLiftPosition() {
        // ensure the lift motors are initialized
        if (liftMotor1 == null || liftMotor2 == null) {
            return 0.0; // Return 0 if motors are not initialized
        }
        // get the current position of both motors
        int position1 = liftMotor1.getCurrentPosition();
        int position2 = liftMotor2.getCurrentPosition(); // Negate the motor being in reverse
        // return the average of the two positions
        return (position1 + position2) / 2.0;
    }

    public double getSlidePosition() {
        // ensure the slide motor is initialized
        if (slideMotor == null) {
            return 0.0; // return 0 if the motor is not initialized
        }
        // get and return the current encoder position of the slide motor
        return slideMotor.getCurrentPosition();
    }

    // Controls the intake of the wrist
    public void intakeControl(double spinControl){
        if (intake1 != null) {
            intake1.setPower(spinControl);
        }
        if (intake2 != null) {
            intake2.setPower(spinControl);
        }
    }

    // The time since the robot started in seconds
    public double currentTime() {
        return nanoTime() * 1e-9;
    }

    public void initializeHardware() {
        // Only initialize arm if it's not already initialized.
        // This is CRUCIAL for transitioning between Auto and TeleOp.
        if (true) { // liftMotor1 == null && liftMotor2 == null && slideMotor == null) {
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "lift1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
            slideMotor = hardwareMap.get(DcMotorEx.class, "slides");

            /* This sets the maximum current that the control hub will apply to the arm before throwing a flag */
            liftMotor1.setCurrentAlert(5, CurrentUnit.AMPS);
            liftMotor2.setCurrentAlert(5, CurrentUnit.AMPS);
            slideMotor.setCurrentAlert(5, CurrentUnit.AMPS);

            /* Before starting the armMotor1. We'll make sure the TargetPosition is set to 0.
            Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
            If you do not have the encoder plugged into this motor, it will not run in this code. */
            liftMotor1.setTargetPosition(0);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

            liftMotor2.setTargetPosition(0);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off  */
        intake1.setPower(INTAKE_OFF);
        intake2.setPower(INTAKE_OFF);
    }

    public static final KinematicType kinematicType = KinematicType.X;
}
