package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
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


        while (opModeIsActive()) {
            //read controller inputs
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.manual(forward, lateral, turn);
            follower.update();


            //when left stick is pushed up set velocity
            //intakeMotor.setVelocity(gamepad2.left_stick_y);
            //TODO: Test the intake if its going at the right speed
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(INTAKE_SPEED);
            } else if (gamepad2.left_stick_y < -0.05){
                intakeMotor.setPower(-INTAKE_SPEED);
            } else {
                intakeMotor.setPower(0);
            }
            //when the button y is pressed then set velocity for launcher
            if(gamepad2.yWasPressed()){
                gate.setPosition(GATE_OPEN);
                launchMotor.setVelocity(LAUNCHER_SPEED);
//                intakeMotor.setVelocity(INTAKE_SPEED);
            }
            else if(gamepad2.yWasReleased()) {
                gate.setPosition(GATE_CLOSE);
            }
            //when y is let go of then turn.
            else if(gamepad2.aWasPressed()){
                launchMotor.setVelocity(0);
                gate.setPosition(GATE_CLOSE);
//                intakeMotor.setVelocity(0.0);
            }
            if(gamepad2.dpad_down) {
                flowerRamp.setPosition(RAMP_DOWN);
            }
            if (gamepad2.dpad_up) {
                flowerRamp.setPosition(RAMP_UP);
            }

            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Left stick Y:", gamepad2.left_stick_y);
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.addData("Intake speed", INTAKE_SPEED);
            telemetry.addData("Launcher Speed", launchMotor.getVelocity());
            telemetry.update();

        }


    }


}
