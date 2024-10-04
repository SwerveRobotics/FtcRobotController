package org.firstinspires.ftc.teamMentor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamMentor.roadrunner.Drawing;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name="TeleOp", group="Competition")
@Config
public class CompetitionTeleOp extends BaseOpMode {

    /* Declare OpMode members. */
    public DcMotorEx  armMotor    = null; //the arm motor
    public CRServo    intake      = null; //the active intake servo
    public Servo      wrist       = null; //the wrist servo
    public DistanceSensor distance = null; //the distance sensor

    static final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)

    public static double ARM_COLLAPSED_INTO_ROBOT  = 3.7; // Was 0
    public static double ARM_COLLECT               = 250;
    public static double ARM_CLEAR_BARRIER         = 230;
    public static double ARM_SCORE_SPECIMEN        = 150;
    public static double ARM_SCORE_SAMPLE_IN_LOW   = 160;
    public static double ARM_ATTACH_HANGING_HOOK   = 124;
    public static double ARM_WINCH_ROBOT           = 15;

    public static double INTAKE_COLLECT    = -1.0;
    public static double INTAKE_OFF        =  0.0;
    public static double INTAKE_DEPOSIT    =  0.5;

    public static double WRIST_FOLDED_IN   = 0.96;
    public static double WRIST_FOLDED_OUT  = 0.57;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPositionFudgeFactor;

    // Expose these to FTC Dashboard:
    private double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValue) {
        // Slow down max speed by 36% and then shape the result:
        stickValue *= 0.8;
        return Math.signum(stickValue) * Math.abs(Math.pow(stickValue, 2.0));
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("TeamMentor TeleOp");
        telemetry.update();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, beginPose);

        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        distance = hardwareMap.get(DistanceSensor .class, "distanceFrontLeft");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        double wristPosition = WRIST_FOLDED_IN;

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            if(gamepad1.right_bumper){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wristPosition = WRIST_FOLDED_OUT;
                intake.setPower(INTAKE_COLLECT);
            }

            else if (gamepad1.left_bumper){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
                wristPosition = WRIST_FOLDED_OUT;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                wristPosition = WRIST_FOLDED_IN;
                intake.setPower(INTAKE_OFF);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wristPosition = WRIST_FOLDED_IN;
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                wristPosition = WRIST_FOLDED_IN;
                intake.setPower(INTAKE_OFF);
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                wristPosition = WRIST_FOLDED_IN;
                intake.setPower(INTAKE_OFF);
            }

            wrist.setPosition(wristPosition);

            armMotor.setTargetPosition((int) ((armPosition - ARM_COLLAPSED_INTO_ROBOT) * ARM_TICKS_PER_DEGREE + armPositionFudgeFactor));

            armMotor.setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (armMotor.isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addLine(String.format("arm target: %d, arm actual: %d", armMotor.getTargetPosition(), armMotor.getCurrentPosition()));
            telemetry.addData("range", String.format("%.01f\"", distance.getDistance(DistanceUnit.INCH)));
            telemetry.addLine();
            telemetry.addLine("A: Intake collect");
            telemetry.addLine("B: Intake deposit");
            telemetry.addLine("X: Intake off");
            telemetry.addLine("Y: Basket");
            telemetry.addLine();
            telemetry.addLine("LB: Clear barrier");
            telemetry.addLine("RB: Intaking position");
            telemetry.addLine();
            telemetry.addLine("Dpad right: High specimen bar");
            telemetry.addLine();
            telemetry.addLine("Dpad up: Prepare for hanging");
            telemetry.addLine("Dpad down: Lift robot");
            telemetry.addLine();
            telemetry.addLine("Dpad left: Home");
            // telemetry.addData("didTimeoutOccur", Boolean.toString(distance.didTimeoutOccur()));

            telemetry.update();

            // Set the drive motor powers according to the gamepad input:
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                    shapeStick(-gamepad1.left_stick_y),
                    shapeStick(-gamepad1.left_stick_x)),
                    shapeStick(-gamepad1.right_stick_x)));

            // Update the current pose:
            drive.updatePoseEstimate();

            // 'packet' is the object used to send data to FTC Dashboard:
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();

            // Do the work now for all active Road Runner actions, if any:
            drive.doActionsWork(packet);

            // Draw the robot and field:
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            MecanumDrive.sendTelemetryPacket(packet);
        }
    }
}
