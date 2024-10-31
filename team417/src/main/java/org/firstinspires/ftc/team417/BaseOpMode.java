package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    final static double ARM_COLLAPSED_INTO_ROBOT = 0;
    final static double ARM_COLLECT = 253.5 * ARM_TICKS_PER_DEGREE;
    final static double ARM_CLEAR_BARRIER = 234 * ARM_TICKS_PER_DEGREE;
    final static double ARM_AUTO_REST_POSITION = 170 * ARM_TICKS_PER_DEGREE;
    final static double ARM_SCORE_SAMPLE_IN_LOW = 155 * ARM_TICKS_PER_DEGREE;
    final static double ARM_SCORE_SPECIMEN = 150 * ARM_TICKS_PER_DEGREE;
    final static double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final static double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final static double INTAKE_COLLECT = -1.0;
    final static double INTAKE_OFF = 0.0;
    final static double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final static double WRIST_FOLDED_IN = 0.676;
    final static double WRIST_FOLDED_OUT = 0.335;

    //position used to score specimens in auto
    public final static double Y_SCORE_POSE = 42.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final static double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /** @noinspection ConstantValue*/
    /* Variables that are used to set the arm to a specific position */
    static double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    // Sharing these objects between CompetitionTeleOp and CompetitionAuto for arm controls
    DcMotorEx armMotor;
    CRServo intake;
    Servo wrist;

    public boolean hasMechanisms = MecanumDrive.driveParameters == DriveParameters.COMPETITION_ROBOT
            || MecanumDrive.driveParameters == DriveParameters.FASTBOT_MECANUM;

    public static final KinematicType kinematicType = KinematicType.MECANUM;

    public void initializeHardware() {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        armMotor.setCurrentAlert(5, CurrentUnit.AMPS);

        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
    }
    // RC 17.50
    // DEV 17.75
    final static double ROBOT_LENGTH = 17.50;
    // RC 16.50
    // DEV 18.50
    final static double ROBOT_WIDTH = 16.50;
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
            armPosition = armMotor.getCurrentPosition();

            double error = Math.abs(armMotor.getCurrentPosition() - targetPosition);

            telemetry.addLine("Moving Arm!");
            telemetry.addData("Target Position", armMotor.getTargetPosition());
            telemetry.addData("Current Position", armMotor.getCurrentPosition());
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
            armMotor.setTargetPosition((int) (targetPosition));
            armMotor.setVelocity(ARM_VELOCITY);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(wristPosition);
            return true;
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


}

