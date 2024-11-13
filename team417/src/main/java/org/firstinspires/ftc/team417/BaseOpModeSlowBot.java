package org.firstinspires.ftc.team417;

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

abstract public class BaseOpModeSlowBot extends LinearOpMode {

    //TODO: tune for correct value
    final static double SLIDE_HOME_POSITION = 0;
    //TODO: tune for correct value
    final static double LIFT_REST_POSITION = 0;

    //TODO: find the range where the slizes CANNOT be out
    final static double NO_SLIDE_ZONE_MIN = 0;
    final static double NO_SLIDE_ZONE_MAX = 0;

    //motors
    static DcMotorEx armMotor1;
    static DcMotorEx armMotor2;
    static DcMotorEx slideMotor;

    //servos
    CRServo intake1;
    CRServo intake2;
    Servo wrist;

    class ControlAction extends RobotAction {

        double targetSlidePosition;
        double targetWristPosition;
        double targetLiftPosition;

        boolean isRetracting;

        final static double EPSILON = 3.00;

        public ControlAction(double targetSlidePosition, double targetWristPosition, double targetLiftPosition) {
            this.targetSlidePosition = targetSlidePosition;
            this.targetLiftPosition = targetLiftPosition;
            this.targetWristPosition = targetWristPosition;
        }

        public void initializeHardware() {
            switch (MecanumDrive.driveParameters) {
                case COMPETITION_ROBOT:
                    initCompBot();
                case FASTBOT_MECANUM:
                    initFastBot();
            }
        }

        public void initFastBot() {
            // Only initialize arm if it's not already initialized.
            // This is CRUCIAL for transitioning between Auto and TeleOp.
            if (armMotor1 == null) {
                armMotor1 = hardwareMap.get(DcMotorEx.class, "arm");
                /* This sets the maximum current that the control hub will apply to the arm before throwing a flag */
                armMotor1.setCurrentAlert(5, CurrentUnit.AMPS);
            /* Before starting the armMotor1. We'll make sure the TargetPosition is set to 0.
            Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
            If you do not have the encoder plugged into this motor, it will not run in this code. */
                armMotor1.setTargetPosition(0);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            intake1 = hardwareMap.get(CRServo.class, "intake");
            wrist = hardwareMap.get(Servo.class, "wrist");

            /* Make sure that the intake is off, and the wrist is folded in. */
            intake1.setPower(INTAKE_OFF);
            // wrist.setPosition(WRIST_FOLDED_IN); We do that after start, since we can't move wrist
            // in the gap before TeleOp.
        }

        public void initCompBot() {
            //motors

            // Only initialize arm if it's not already initialized.
            // This is CRUCIAL for transitioning between Auto and TeleOp.
            if (armMotor1 == null && armMotor2 == null && slideMotor == null) {
                armMotor1 = hardwareMap.get(DcMotorEx.class, "arm1");
                armMotor2 = hardwareMap.get(DcMotorEx.class, "arm2");
                slideMotor = hardwareMap.get(DcMotorEx.class, "slides");

                /* This sets the maximum current that the control hub will apply to the arm before throwing a flag */
                armMotor1.setCurrentAlert(5, CurrentUnit.AMPS);
                armMotor2.setCurrentAlert(5, CurrentUnit.AMPS);
                slideMotor.setCurrentAlert(5, CurrentUnit.AMPS);

            /* Before starting the armMotor1. We'll make sure the TargetPosition is set to 0.
            Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
            If you do not have the encoder plugged into this motor, it will not run in this code. */
                armMotor1.setTargetPosition(0);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                armMotor2.setTargetPosition(0);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

                armMotor1.setTargetPosition(0);
                armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            intake1 = hardwareMap.get(CRServo.class, "intake1");
            intake2 = hardwareMap.get(CRServo.class, "intake2");
            wrist = hardwareMap.get(Servo.class, "wrist");

            //servos

            /* Make sure that the intake is off, and the wrist is folded in. */
            intake1.setPower(INTAKE_OFF);
            intake2.setPower(INTAKE_OFF);
        }

        public void setArmTargetPos(int pos) {
            if (armMotor1 != null) {
                armMotor1.setTargetPosition(pos);
            }

            if (armMotor2 != null) {
                armMotor2.setTargetPosition(pos);
            }
        }

        public void setSlidesTargetPos(int pos) {
            if (slideMotor != null) {
                slideMotor.setTargetPosition(pos);
            }
        }

        public void INTAKE_OUTTAKE_SPEED(double power) {
            if (intake1 != null) {
                intake1.setPower(power);
            }

            if (intake2 != null) {
                intake2.setPower(power);
            }
        }

        public void setWristPos(double pos) {
            if (wrist != null) {
                wrist.setPosition(pos);
            }
        }

        @Override
        public boolean run(double elapsedTime) {

            boolean isCrossingNoSlideZone
                    = ((targetLiftPosition > NO_SLIDE_ZONE_MAX && getLiftPosition() < NO_SLIDE_ZONE_MAX) ||
                    (targetLiftPosition < NO_SLIDE_ZONE_MIN && getLiftPosition() > NO_SLIDE_ZONE_MIN));
            boolean isSlideIn = (getSlidePosition() <= SLIDE_HOME_POSITION);

            if (elapsedTime == 0) {
                // This block makes sure the slide goes in before lift goes up
                if ((isCrossingNoSlideZone) && (!isSlideIn)) {
                    isRetracting = true;
                }
            }
            // First retracts the slides in
            if (isRetracting) {
                moveSlide(SLIDE_HOME_POSITION);

                // Checks if the slide is at Home position
                if (getSlidePosition() <= SLIDE_HOME_POSITION + EPSILON) {
                    return false; // Call us again, we're not there yet
                }
                isRetracting = false; // No need to retract anymore
            }
            // Then move lift to target position
            moveLift(targetLiftPosition);

            // Once lift is ABOVE the no slide zone, move the slide & wrist out at the same time
            if (!isCrossingNoSlideZone) {
                moveSlide(targetSlidePosition);
                moveWrist(targetWristPosition);
            }

            // Buffers for target lift & slide position
            double liftError = Math.abs(getLiftPosition() - targetLiftPosition);
            double slideError = Math.abs(getSlidePosition() - targetSlidePosition);

            // TODO: Check if the EPSILON is different for either of the errors
            // Checks if the slide or the lift is in the correct spot
            if (liftError < EPSILON || slideError < EPSILON) {
                return false;
            }
            return false;
        }
    }


    // This method checks if the linear slide is extended or not. Returns true if it is extended, false if not
    // TODO: implement this
    public boolean isSlideExtended(){
        return false;
    }

    // This helper method controls the linear slides and tells motor to go to desired position in ticks
    // TODO: implement this
    public void moveSlide(double positionInTicks){}

    // This method controls the 4bar to desired height
    // TODO: implement this
    public void moveLift(double heightInTicks){}

    public double getLiftPosition(){
        return 0.0;
    }
    public double getSlidePosition() {
        return 0.0;
    }

    // This method moves the wrist up and down based on desired position
    // TODO: implement this
    public void moveWrist(double wristPosition){}

    // Controls the intake of the wrist
    // TODO: implement this
    public void intakeControl(double spinControl){}

    // TODO: implement this
    final static double INTAKE_DEPOSIT = 0.0;
    final static double INTAKE_COLLECT = 0.0;
    final static double INTAKE_OFF = 0.0;
    final static double LIFT_COLLECT = 0.0;
    final static double SLIDE_COLLECT = 0.0;
    final static double WRIST_OUT = 0.0;
    final static double WRIST_IN = 0.0;
    final static double LIFT_CLEAR_BARRIER = 0.0;
    // Slow bot variables
    double liftPosition = LIFT_REST_POSITION;
    double slidePosition = SLIDE_HOME_POSITION;
    final static double linearSlideHome = 0.0;
    final static double SLIDE_SCORE_IN_BASKET = 0;
    final static double LIFT_SCORE_HIGH_SPECIMEN = 0.0;
    final static double LIFT_SCORE_LOW_BASKET = 0.0;
    final static double LIFT_SCORE_HIGH_BASKET = 0.0;

    // RC 17.50
    // DEV 17.75
    final static double ROBOT_LENGTH = 17.50;
    // RC 16.50
    // DEV 18.50
    final static double ROBOT_WIDTH = 16.50;

    public void initializeHardware() {}

    public static final KinematicType kinematicType = KinematicType.X;

}
