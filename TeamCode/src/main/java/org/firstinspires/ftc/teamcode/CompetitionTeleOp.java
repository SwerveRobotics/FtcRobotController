package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends BaseOpMode{
    private Follower follower;
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose resetPoint = poseFactory.of(80, 8, 90);
    private Pose autoDriveControl1 = poseFactory.of(0,90,90);
    private Pose autoDriveControl2 = poseFactory.of(20,30,90);




    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        follower = Constants.create(hardwareMap);
        waitForStart();

        // for imu heading correction on strafing
        double targetHeading = 0;
        boolean wasTurning = false;
        double kp = 0.03; //TODO tune for pid

        while (opModeIsActive()) {
            Pose currPose = poseFactory.of(follower.pose().x(), follower.pose().y(), follower.pose().heading());
            DashboardDraw.drawRobot(currPose);
            intakeMot.setVelocity(gamepad2.left_stick_y * INTAKE_SPEED_MULTIPLIER);
            leftIntakeServo.setPower(gamepad2.left_stick_y);
            rightIntakeServo.setPower(gamepad2.left_stick_y);

            if (gamepad2.aWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);

            }
            if (gamepad2.yWasPressed()) {
//                transferWheelMot.setVelocity(TRANSFER_SPEED);
                lowerFlywheelMot.setVelocity(LAUNCHER_SPEED - LAUNCHER_TOPSPIN);
                upperFlywheelMot.setVelocity(LAUNCHER_SPEED);

            }
            if(gamepad2.bWasPressed()) {
                transferWheelMot.setVelocity(WHEEL_STOP_SPEED);
                lowerFlywheelMot.setVelocity(WHEEL_STOP_SPEED);
                upperFlywheelMot.setVelocity(WHEEL_STOP_SPEED);
            }
            if(gamepad2.leftBumperWasPressed()){
                Pose dynamicControl1;
                Pose dynamicControl2;
                if(currPose.x() > 72){
                    dynamicControl1 = poseFactory.of(130,90,90);
                    dynamicControl2 = poseFactory.of(130,30,90);
                } else {
                   dynamicControl1 = autoDriveControl1;
                    dynamicControl2 = autoDriveControl2;

                }
                Path autoDriveTo  = curve(currPose, dynamicControl1,dynamicControl2, resetPoint).linear(currPose, resetPoint);
                follower.follow(autoDriveTo);
            }


            if(!follower.isBusy()) {
                double slowModeMultiplier = doSLOWMODE();

                double drive = -gamepad1.left_stick_y * slowModeMultiplier;
                double strafe = -gamepad1.left_stick_x * slowModeMultiplier;
                double turn = -gamepad1.right_stick_x * slowModeMultiplier;
                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

                // hold the heading of the robot at the turn heading driver has set it to
                // automatic steering correction for driving forward/backward and strafing
                if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                    targetHeading = currentHeading;
                    wasTurning = true;
                } else {
                    // set new target heading as soon as driver stops actively turning
                    if (wasTurning) {
                        targetHeading = currentHeading;
                        wasTurning = false;
                    }
                }

                double angleError = targetHeading - currentHeading;
                // set angle error to be in range 0 to 360 because imu reads from -180 to 180
                while (angleError > 180)  angleError -= 360;
                while (angleError < -180) angleError += 360;

                turn = angleError * kp;

                follower.manual(
                        drive,
                        strafe,
                        turn
                );
            }


            follower.update();


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


