package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    //variables
    Alliance chosenAlliance;
    double targetX;
    double targetY;
    Pose targetPose;
    boolean fieldCentric = false;
    double forward;
    double lateral;
    double turn;
    double fieldCentricCorrectionFactor = 0;
    double amountToTurn = 0;
    private boolean turning = false;
    private double targetHeading = 0;

    private Follower follower;
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
        follower.setPose(PoseStorage.endPose);
        chosenAlliance = PoseStorage.chosenAlliance;
        flowerRamp.setPosition(RAMP_UP);
        targetPose = Pose.zero();

        while (opModeIsActive()) {
            /*LAUNCH AND INTAKE CONTROLS:
            gamepad1:
                left joystick: x and y movement
                right joystick: rotation
                left bumper: reset perceived position to 0, 0, 0
                right trigger: slow mode
                y: toggle field-centric drive
                dpad left: rotate field-centric drive perspective by 90 degrees counterclockwise
                dpad right: rotate field-centric drive perspective by 90 degrees clockwise
                x: rotate robot in direction of nearest hive
            gamepad2:
                left joystick: up for outtake, down for intake
                y: open launch gate and intake
                right bumper: turn on flywheel
                left bumper: turn off flywheel
                dpad up: flower ramp up
                dpad down: flower ramp down



             */
            //when left stick is pushed up set velocity
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_stick_y < -0.05 ) {
                intakeMotor.setPower(-INTAKE_SPEED);
            }else if (gamepad2.y){
                //launchMotor.setVelocity(LAUNCHER_SPEED);

                intakeMotor.setPower(INTAKE_SPEED*.5);
            } else {
                intakeMotor.setPower(0);
            }
            //launching controls
            launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(KP, KI, KD, KF));



            if (gamepad2.right_bumper) {
                launchMotor.setVelocity(LAUNCHER_SPEED);
                telemetry.addLine("Launcher is being intiated!");
            }else if (gamepad2.left_bumper) {
                launchMotor.setVelocity(0);
            }

            // Hold Y to launch
            if (gamepad2.y) {
                gate.setPosition(GATE_OPEN);

            } else {
                gate.setPosition(GATE_CLOSE);
                //intakeMotor.setPower(0);

            }
            //flower intake up and down controls
            if (gamepad2.dpad_down) {
                //flowerRamp.getController().pwmEnable();

                flowerRamp.setPosition(RAMP_DOWN);
            }
            if (gamepad2.dpad_up) {
                flowerRamp.setPosition(RAMP_UP);
                //flowerRamp.getController().pwmDisable();
                //flowerRamp.setPosition(RAMP_UP);
            }
            if (gamepad1.left_bumper) {
                follower.setPose(new Pose(0, 0, 0));
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
                targetHeading = Math.atan2(targetY-follower.pose().y(),targetX - follower.pose().x()); //get angle based on target
                turning = true;
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
                // Stop the  turn when the robot is around 2 degrees of the target(hich is 30 degrees)
                if (Math.abs(headingError) < Math.toRadians(2)) {
                    turning = false;
                    turn = 0;
                } else {
                    // Calculate turn power based on heading error, limiting it between -0.5 and 0.5
                    turn = Math.max(-0.5, Math.min(0.5, headingError * 1.5));
                    //turn = Math.max(-0.5, Math.min(0.5, -headingError * 1.5));
                }
            }
            //MOVE ROBOT!!!!!!!!!!!!!
            //read controller inputs
            forward = -gamepad1.left_stick_y * doSLOWMODE();
            lateral = -gamepad1.left_stick_x * doSLOWMODE();
            turn = -gamepad1.right_stick_x * doSLOWMODE();
            if(gamepad1.yWasPressed()) {
                fieldCentric = !fieldCentric;
            }
            if (!fieldCentric) {
                follower.manual(forward, lateral, turn);
            } else { //do math to make robot drive correctly
                double t = Math.toRadians(follower.pose().heading() + fieldCentricCorrectionFactor);
                follower.manual(forward * Math.cos(t) - lateral * Math.sin(t),
                        forward * Math.sin(t) + lateral* Math.cos(t),
                        turn);
            }
            if(gamepad1.dpad_left) { //rotate the robot's perceived angle
                fieldCentricCorrectionFactor += 90;
            }
            if(gamepad1.dpad_right) {
                fieldCentricCorrectionFactor -= 90;
            }


            follower.update();
            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            //telemetry.addData("Left stick Y:", gamepad2.left_stick_y);
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Intake speed", INTAKE_SPEED);
            telemetry.addData("Launcher Speed", launchMotor.getVelocity());
            telemetry.addData("Field-centric drive: ", fieldCentric);

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
    /*
    @Config
     class AutoAim {
        private final PIDControllerNEW pid = new PIDControllerNEW();
        public double getOffset() {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addData("AutoAim: ", "No Limelight result");
                pid.reset();
                return 0;
            }

            List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();

            LLResultTypes.FiducialResult target = null;

            // Only target AprilTags 39 and 40
            for (LLResultTypes.FiducialResult detection : detections) {
                int id = detection.getFiducialId();

                if (id == 39 || id == 40) {
                    target = detection;
                    break;
                }
            }

            if (target == null) {
                telemetry.addData("AutoAim", "Tags 39 OR 40 not detected");
                pid.reset();
                return 0;
            }

            // Horizontal angle from the Limelight to the target
            double tx = target.getTargetXDegrees();

            // Center the tag: desired horizontal angle is 0 degrees
            double turnPower = pid.calculate(tx, 0);

            // Limit turn power
            turnPower = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turnPower)
            );

            telemetry.addData("AutoAim Target ID", target.getFiducialId());
            telemetry.addData("Horizontal Error (degrees)", tx);
            telemetry.addData("AutoAim Turn Power", turnPower);

            return turnPower;
        }
    }
    class PIDControllerNEW {
        private double previousError = 0;
        private double integral = 0;
        private long lastTimestamp = System.nanoTime();

        public double calculate(double currentValue) {
            return calculate(currentValue, 0);
        }

        public double calculate(double currentValue, double setpoint) {
            long now = System.nanoTime();
            double dt = (now - lastTimestamp) / 1e9;
            lastTimestamp = now;

            // Avoid division by zero
            if (dt <= 0) {
                dt = 0.001;
            }

            double error = setpoint - currentValue;

            integral += error * dt;
            double derivative = (error - previousError) / dt;

            double output = (AIM_KP * error) + (AIM_KI * integral) + (AIM_KD * derivative);
            previousError = error;
            return output;
        }

        public void reset() {
            previousError = 0;
            integral = 0;
            lastTimestamp = System.nanoTime();
        }
    }

*/
}