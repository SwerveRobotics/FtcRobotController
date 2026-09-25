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
        follower = Constants.create(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;


            //"The driver wants to move this way."
            //Pedro then figures out what the drivetrain motors need to do.
            follower.manual(forward, lateral, turn);
            follower.update();
            //add telemetry on driver station X, Y, Heading follower.pose.getheading .getY  .getX
        }

        while (opModeIsActive()) {
            //add intake controls here
            //add follower update
        }
        //slowmodeeeee

    }


}
