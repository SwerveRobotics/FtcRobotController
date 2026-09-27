package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="417 Teleop")
public class CompetitionTeleOp extends BaseOpMode {
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        //where everything gets initialized
        follower = Constants.create(hardwareMap);
        waitForStart();



        while (opModeIsActive()) {
            //read controler inputs
            double forward = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            //"The driver wants to move this way."
            //Pedro then figures out what the drivetrain motors need to do.
            follower.manual(forward, lateral, turn);
            //update pedro!
            follower.update();

            //add intake controls here
            //add follower update at the end

            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX

        }




        //slowmodeeeee

    }


}
