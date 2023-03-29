package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;

@Autonomous(name = "StackGrabbingTest")
public class StackGrabbingTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        robotCameraPipeline.setTargetColor(RobotCameraPipeline.Color.BLUE);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        waitForStart();
        sleep(300);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(300);
        for (int i = 0; i <= 4; i++) {
            //turn LED's off (for testing purposes)
            blinkinChassis.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            //drive slides to stack position
            driveSlides((Constants.SLIDE_STACK_FOUR)-(Constants.AUTONOMOUS_STACK_PER_CONE_OFFSET * i));
            //center on cone stack
            centerConeStack(robotCameraPipeline, Constants.CONE_WIDTH);
            sleep(300);
            //wait for grabber to close
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            sleep(600);
            //drive slides to stow position
            driveSlides(Constants.SLIDE_LOW);
            //drive backwards 5 inches to get the robot clear of cone stack
            driveAutonomous(180, 5);
            //turn towards junction
            turnToAngle(135);
            //detect the junction / cones on it
            robotCameraPipeline.setTargetColor(RobotCameraPipeline.Color.ALL);
            //drive to junction
            centerConeStack(robotCameraPipeline, 100 /*custom break distance TBD*/);
            //drive slides up
            driveSlides(Constants.SLIDE_TOP);
            //wait for slides to go all the way up
            sleep(600);
            //drive forward
            driveAutonomous(0, 2);
            //center on junction
            centerJunctionTop(grabberCameraPipeline);
            sleep(100);
            //lower slides onto junction
            driveSlides(Constants.SLIDE_HIGH-100);
            sleep(100);
            //open the grabber
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            //wait for cone to drop
            sleep(100);
            //drive slides back up
            driveSlides(Constants.SLIDE_TOP);
            sleep(200);
            //drive backwards
            driveAutonomous(180, 3);
            sleep(200);
            //turn back to 0 heading
            turnToAngle(0);
        }
    }
}