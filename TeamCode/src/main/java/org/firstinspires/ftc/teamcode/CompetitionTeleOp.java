package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    //variables
    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        //where everything gets initialized
        follower = Constants.create(hardwareMap);
        initializeHardware();
        waitForStart();
        follower.setPose(PoseStorage.endPose);

        flowerRamp.setPosition(RAMP_UP);
        while (opModeIsActive()) {
            //read controller inputs
            double forward = -gamepad1.left_stick_y*doSLOWMODE();
            double lateral = -gamepad1.left_stick_x*doSLOWMODE();
            double turn = -gamepad1.right_stick_x*doSLOWMODE();
            follower.manual(forward, lateral, turn);
            follower.update();


            //when left stick is pushed up set velocity
            //intakeMotor.setVelocity(gamepad2.left_stick_y);
            //TODO: Test the intake if its going at the right speed
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_stick_y < -0.05 || gamepad2.y){
                intakeMotor.setPower(-INTAKE_SPEED);
            } else {
                intakeMotor.setPower(0);
            }
            //launching controls
            if(gamepad2.right_bumper){
                launchMotor.setVelocity(LAUNCHER_SPEED);
            }
            else if (gamepad2.left_bumper){
                launchMotor.setVelocity(0);
            }

            // Hold Y to launch
            if (gamepad2.y) {
                gate.setPosition(GATE_OPEN);
            } else {
                gate.setPosition(GATE_CLOSE);
            }

            //flower intake up and down controls
            if(gamepad2.dpad_down) {
                flowerRamp.setPosition(RAMP_DOWN);
            }
            if (gamepad2.dpad_up) {
                flowerRamp.setPosition(RAMP_UP);
            }
            if(gamepad1.left_bumper) {
                if(gamepad1.left_bumper) {

                }
                follower.setPose(new Pose(0,0,0));
            }

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
            return ((-gamepad1.right_trigger + 1)/2) + 0.5;
        } else {
            return 1;
        }
    }

}
