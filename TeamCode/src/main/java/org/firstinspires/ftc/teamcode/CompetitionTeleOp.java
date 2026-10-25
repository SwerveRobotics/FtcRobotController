package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.Arrays;
import java.util.List;
@Config
@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    //variables
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
        flowerRamp.setPosition(RAMP_UP);

        while (opModeIsActive()) {
            //read controller inputs
            double forward = -gamepad1.left_stick_y * doSLOWMODE();
            double lateral = -gamepad1.left_stick_x * doSLOWMODE();
            double turn = -gamepad1.right_stick_x * doSLOWMODE();
            //drive the robot

            //when left stick is pushed up set velocity
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_stick_y < -0.05 ) {
                intakeMotor.setPower(-INTAKE_SPEED);
            }else if (gamepad2.y){
                intakeMotor.setPower(INTAKE_SPEED*.25);
            } else {
                intakeMotor.setPower(0);
            }
            //launching controls
            if (gamepad2.right_bumper) {
                launchMotor.setVelocity(LAUNCHER_SPEED);
            } else if (gamepad2.left_bumper) {
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
                if (gamepad1.left_bumper) {

                }
                follower.setPose(new Pose(0, 0, 0));
            }


            // Press A to turn 30 degrees
            if (gamepad1.a && !turning) {
                // Set the target to 30 degrees more the heading in follower rn

                targetHeading = follower.pose().heading() + Math.toRadians(30);
                turning = true;
            }



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

            follower.manual(forward, lateral, turn);
            follower.update();
            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            //telemetry.addData("Left stick Y:", gamepad2.left_stick_y);
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Intake speed", INTAKE_SPEED);
            telemetry.addData("Launcher Speed", launchMotor.getVelocity());
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