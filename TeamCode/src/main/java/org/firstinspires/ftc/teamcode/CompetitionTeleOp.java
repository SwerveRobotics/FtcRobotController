package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    //variables
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //where everything gets initialized
        follower = Constants.create(hardwareMap);
        waitForStart();
        initializeHardware();


        while (opModeIsActive()) {
            //read controler inputs
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.manual(forward, lateral, turn);
            follower.update();


            //when left stick is pushed up set velocity
            //intakeMotor.setVelocity(gamepad2.left_stick_y);
            if (gamepad2.left_stick_y > 0.05) {
                intakeMotor.setPower(1);
            } else if (gamepad2.left_stick_y < -0.05){
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }
            //when the button y is pressed then set velocity for launcher
            if(gamepad2.yWasPressed()){
                launchMotor.setVelocity(LAUNCHER_SPEED);
            }
            //when y is is let go of then turn
            else if(gamepad2.yWasReleased()){
                launchMotor.setVelocity(0);
            }

            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }


    }


}
