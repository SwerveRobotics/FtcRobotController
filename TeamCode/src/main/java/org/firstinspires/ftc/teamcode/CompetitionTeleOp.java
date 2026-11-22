package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.Collections;
import java.util.List;
import java.util.Set;


@Config
@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    //variables
    ElapsedTime timer = new ElapsedTime();
    boolean rampMoving = false;
    boolean lastDpadUp = false;
    double targetX;
    double targetY;
    Pose targetPose;
    boolean fieldCentric = false;
    double forward;
    double lateral;
    double turn;
    double fieldCentricCorrectionFactor = 0;
    double amountToTurn =
            0;
    private boolean turning = false;
    private double targetHeading = 0;
    private DrivePowers fcPowers;
    private double turretRelative;
    //private Limelight3A limelight;
    //private AutoAim autoAim;
    @Override
    public void runOpMode() throws InterruptedException {
        //where everything gets initialized
        follower = Constants.create(hardwareMap);
        initializeHardware();
        //Auto Aim
        //variable that contains the value which the robot heading will turn

        //limelight.pipelineSwitch(0);
        //limelight.start();
        //autoAim = new AutoAim();
        //driver press st art on DS
        waitForStart();

        turretServo.setPosition(SERVO_POSITION_FORWARD);
        turretServo2.setPosition(SERVO_POSITION_FORWARD);


        //TODO: commented out RED_LIST
        follower.setPose(PoseStorage.endPose);
        chosenAlliance = PoseStorage.chosenAlliance;
        //flowerRamp.setPosition(RAMP_UP);
        targetPose = Pose.zero();
        //Set<List> ignoredTags = Set.of(RED_LIST);
        Set<List<Integer>> ignoredTags = Collections.emptySet();
        if (chosenAlliance == Alliance.BLUE) {
            ignoredTags = Set.of(RED_LIST);
        } else if (chosenAlliance == Alliance.RED) {
            ignoredTags = Set.of(BLUE_LIST);
        }

        while (opModeIsActive()) {
            updateLauncherPidf();

            /**
             *
             */
            //LLResult result = limelight.getLatestResult();
            LLResult result = null;

            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                for (FiducialResult tag : fiducials) {
                    int id = tag.getFiducialId();

                    // Skip any tag on your exclusion list TODO: commented out below
                    if (ignoredTags.contains(id)) {
                        continue;
                    }
                    double tx = tag.getTargetXDegrees();
                    double ty = tag.getTargetYDegrees();
                    telemetry.addData("Valid Tag " + id, "tx: %.2f, ty: %.2f", tx, ty);
                }

                double tx = result.getTx();
                double currentHeadingInDegrees = Math.toDegrees(follower.pose().heading());
                //turnTurretAbsolute(follower.pose().heading(), currentHeadingInDegrees - tx);
                telemetry.addData("Tag TX", tx);

            } else {
                // midpoint = (70.75, 70.75)
                follower.pose().x();
                follower.pose().y();
                double xToOrigin = follower.pose().x() - 70.75;
                double yToOrigin = follower.pose().y() - 70.75;
                double angleInRadian = Math.atan2(yToOrigin, xToOrigin);
                double convertRadiansIntoDegrees = Math.toDegrees(angleInRadian);
                //turnTurretAbsolute(follower.pose().heading(), convertRadiansIntoDegrees + 180);
                telemetry.addData("Robot target heading", convertRadiansIntoDegrees + 180);
            }



            //LAUNCH AND INTAKE CONTROLS

            //when left stick is pushed up set velocity
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(INTAKE_SPEED);
                transferMotor.setPower(TRANSFER_INTAKE_SPEED);
                gate.setPosition(GATE_CLOSED);

            } else if (gamepad2.left_stick_y < -0.05) {

                intakeMotor.setPower(-INTAKE_SPEED);

                transferMotor.setPower(-TRANSFER_SPEED);
                gate.setPosition(GATE_CLOSED);


//launching!
            }else if (gamepad2.y){
                intakeMotor.setPower(INTAKE_SPEED);//was slower by .5 before
                transferMotor.setPower(TRANSFER_SPEED);
                gate.setPosition(GATE_OPEN);

            } else {
                intakeMotor.setPower(0);
                transferMotor.setPower((0));
                gate.setPosition(GATE_CLOSED);
            }
            //launching controls

            if(gamepad2.rightTriggerWasPressed()){
                setTurretVelocity(VELOCITY_1, VELOCITY_2);
            }else if (gamepad2.left_trigger_pressed){
                setTurretVelocity(0, 0);
            }




            if(gamepad2.right_bumper){
                servoPosition(SERVO_POSITION_FORWARD);
            } else if (gamepad2.left_bumper) {
                servoPosition(SERVO_POSITION_BACKWARD);

            }

            //turretIsFast();
            // Hold Y to launch

            //flower intake up and down controls
            // Flower intake up and down controls

//            if (gamepad2.dpad_down) {
//                flowerRamp.getController().pwmEnable();
//                flowerRamp.setPosition(RAMP_DOWN);
//                rampMoving = false;
//            }
//
//            if (gamepad2.dpad_up && !lastDpadUp) {
//                flowerRamp.getController().pwmEnable();
//                timer.reset();//                flowerRamp.setPosition(RAMP_UP);
//                rampMoving = true;
//            }
//            if (rampMoving && timer.milliseconds() >= 700) {
//                flowerRamp.getController().pwmDisable();
//                telemetry.addLine("DISABLEDDDD");
//                rampMoving = false;
//            }

            lastDpadUp = gamepad2.dpad_up;
            if (gamepad1.left_bumper) {
                fieldCentricCorrectionFactor = -follower.pose().heading();
            }
            if (gamepad1.right_bumper) {
                follower.setPose(new Pose(8.5, 8.5, follower.pose().heading()));
            }
            //PSEUDO AUTO AIM USING ODOMETRY

            if (gamepad1.x) {
                if (chosenAlliance.equals(Alliance.BLUE)) {
                    targetX = 84;
                } else {
                    targetX = 58;
                }
                if (follower.pose().y() > 70) {
                    targetY = 86.5;
                }   else {
                    targetY = 54.5;
                }
                targetHeading = Math.toDegrees(Math.atan2(targetY-follower.pose().y(),targetX - follower.pose().x())); //get angle based on target
                //turnTurretAbsolute(follower.pose().heading(), targetHeading);
            }
            /*
            // Press A to turn 30 degrees
            if (gamepad1.a && !turning) {
                // Set the target to 30 degrees more the heading in follower rn

                targetHeading = follower.pose().heading() + Math.toRadians(30);
                turning = true;
            }

*/

/*
            if (gamepad1.right_bumper && autoA            //TODO: Auto Aim :)
im != null) {
                amountToTurn = AutoAim().getOffset();
                telemetry.addData("AutoAim", "ON");
                telemetry.addData("amountToTurn", amountToTurn);
            } else {
                amountToTurn = turn;
                telemetry.addData("AutoAim", "OFFFF");
            }

 */



            double headingError = targetHeading - follower.pose().heading();

            if (turning) {
                // Stop the  turn when the robot is around 2 degrees of the target(which is 30 degrees)
                if (Math.abs(headingError) < 2) {
                    turning = false;
                    turn = 0;
                } else {
                    // Calculate turn power based on heading error, limiting it between -0.5 and 0.5
                    turn = Math.max(-0.5, Math.min(0.5, headingError * 1.5));
                    //turn = Math.max(-0.5, Math.min(0.5, -headingError * 1.5));
                }
            }
            //TURRET HEADING
            if(gamepad2.right_stick_y < -0.05) {
                turretRelative = 0;
                //turnTurretRelative(turretRelative);
            } else if (gamepad2.right_stick_y > 0.05) {
                turretRelative = 180;
                //turnTurretRelative(turretRelative);
            }
            //updateTurret(follower.pose().x(), follower.pose().y(), Math.toRadians(follower.pose().heading()));
            //MOVE ROBOT!!!!!!!!!!!!!
            //read controller inputs
            forward = -gamepad1.left_stick_y * doSLOWMODE();
            lateral = -gamepad1.left_stick_x * doSLOWMODE();
            if (!turning) {
                turn = -gamepad1.right_stick_x * doSLOWMODE();
            }

            if (gamepad1.yWasPressed()) {
                fieldCentric = !fieldCentric;
            }
            if (!fieldCentric) {
                follower.manual(forward, lateral, turn);
            } else { //do math to make robot drive correctly
//                double t = Math.toRadians(follower.pose().heading() + fieldCentricCorrectionFactor);
//                follower.manual(forward * Math.cos(t) - lateral * Math.sin(t),
//                        forward * Math.sin(t) + lateral* Math.cos(t),
//                        turn);
                fcPowers = ManualDrive.fieldCentric(forward, lateral, turn, follower.pose().heading(), fieldCentricCorrectionFactor);
                follower.manual(fcPowers);
            }


            follower.update();
            printTelemetry();
            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
            telemetry.addData("KF: ", KF);
            telemetry.addData("Field-centric drive: ", fieldCentric);
            telemetry.addData("FC correction angle: ", fieldCentricCorrectionFactor);

            telemetry.update();

        }
    }
    public double doSLOWMODE() {
        if (gamepad1.right_trigger != 0) {
            return ((-gamepad1.right_trigger + 1) / 2) + 0.5;
        } else {
            return 1;
        }
    }
}