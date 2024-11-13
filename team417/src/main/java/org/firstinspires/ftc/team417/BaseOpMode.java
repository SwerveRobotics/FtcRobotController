package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.team417.roadrunner.KinematicType;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
@Config
abstract public class BaseOpMode extends LinearOpMode {
    public double startHeading;

    final double ARM_VELOCITY = 2100; // The ticks-per-second constant that Go-Bilda gave us

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final static double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    static double ARM_COLLAPSED_INTO_ROBOT = 0;
    static double ARM_COLLECT = 254 * ARM_TICKS_PER_DEGREE;
    static double ARM_CLEAR_BARRIER = 234 * ARM_TICKS_PER_DEGREE;
    static double ARM_AUTO_REST_POSITION = 170 * ARM_TICKS_PER_DEGREE;
    static double ARM_SCORE_SAMPLE_IN_LOW = 155 * ARM_TICKS_PER_DEGREE;
    static double ARM_SCORE_SPECIMEN = 150 * ARM_TICKS_PER_DEGREE;
    static double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    static double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final static double INTAKE_COLLECT = -1.0;
    final static double INTAKE_OFF = 0.0;
    final static double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final static double WRIST_FOLDED_IN = 0.676;
    final static double WRIST_FOLDED_OUT = 0.335;

    //position used to score specimens in auto
    public final static double Y_SCORE_POSE = 41.5;
    public final static double XDRIVE_Y_SCORE_POSE = 33;
    /* A number in degrees that the triggers can adjust the arm position by */
    final static double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /** @noinspection ConstantValue*/
    /* Variables that are used to set the arm to a specific position */
    static double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    // Sharing these objects between CompetitionTeleOp and CompetitionAuto for arm controls
    
    //motors
    static DcMotorEx armMotor1;
    static DcMotorEx armMotor2;
    static DcMotorEx slideMotor;
    
    //servos
    CRServo intake1;
    CRServo intake2;
    Servo wrist;

    public boolean hasMechanisms = MecanumDrive.driveParameters == DriveParameters.COMPETITION_ROBOT
            || MecanumDrive.driveParameters == DriveParameters.FASTBOT_MECANUM;

    public static final KinematicType kinematicType = KinematicType.MECANUM; // will have to change for league 2, once all robot measurements are updated

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

    // RC 17.50
    // DEV 17.75
    // X 17.00
    final static double ROBOT_LENGTH = 17.00;
    // RC 16.50
    // DEV 18.50
    // X 17.00
    final static double ROBOT_WIDTH = 17.00;
    class MoveArm extends RobotAction {

        double targetPosition;
        double wristPosition;

        MoveArm(double targetPosition, double wristPosition) {
            this.targetPosition = targetPosition;
            this.wristPosition = wristPosition;
        }
        //ARM_SCORE_SAMPLE_IN_LOW

        final static double EPSILON = 3.00;

        @Override
        public boolean run(double elapsedTime) {
            armPosition = armMotor1.getCurrentPosition();

            double error = Math.abs(armMotor1.getCurrentPosition() - targetPosition);

            telemetry.addLine("Moving Arm!");
            telemetry.addData("Target Position", armMotor1.getTargetPosition());
            telemetry.addData("Current Position", armMotor1.getCurrentPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Within epsilon", error < EPSILON);

            // Prevents hanging from running on wily works
            if (WilyWorks.isSimulating) {
                return false;
            }

            if (error < EPSILON) {
                // Arm is within range so arm stops
                return false;
            }
            // Arm isn't within range so we keep calling
            armMotor1.setTargetPosition((int) (targetPosition));
            armMotor1.setVelocity(ARM_VELOCITY);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristPosition);
            return true;
        }
    }

    class RunIntake extends RobotAction{
        double power;
        RunIntake(double power) {
            this.power = power;
        }
        @Override
        public boolean run (double elapsedTime) {

            intake1.setPower(power);
            return false;
        }
    }
    class WaitAction extends RobotAction {
        RobotAction actionToWaitOn;
        WaitAction(RobotAction actionToWaitOn) {
            this.actionToWaitOn = actionToWaitOn;
        }
        @Override
        public boolean run(double elapsedTime) {
                return actionToWaitOn.isRunning();
        }

        }
    class ScoreSample extends RobotAction {
        @Override
        public boolean run(double elapsedTime) {
            // Keep the intake deposit on until the 2 seconds are over
            if (elapsedTime <= 2) {
                intake1.setPower(INTAKE_DEPOSIT);
                return true;
            }
            // Turn off deposit after 2 seconds and then end action
            intake1.setPower(INTAKE_OFF);
            return false;
        }
    }
    class intakeSample extends RobotAction {
        @Override
        public boolean run(double elapsedTime) {
            if(elapsedTime <=2) {
                intake1.setPower(INTAKE_COLLECT);
                return true;
            }
            return false;
        }
    }
}

