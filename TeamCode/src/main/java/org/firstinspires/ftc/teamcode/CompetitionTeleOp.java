package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "CompetitionTeleOp")
public class CompetitionTeleOp extends BaseOpMode{
    private Follower follower;
    private PoseFactory poseFactory = PoseFactory.degrees();
    private Pose blue_basket_r = poseFactory.of(84,84,0);
    private Pose resetPoint = poseFactory.of(80, 8, 90);
    private Pose autoDriveControl1 = poseFactory.of(0,90,90);
    private Pose autoDriveControl2 = poseFactory.of(20,30,90);


    // AUTOAIM
    double aimTargetX = 72;
    double aimTargetY = 72;





    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(
                autoEndX,
                autoEndY,
                autoEndHeading
        ));

        waitForStart();

        // for imu heading correction on strafing
        double targetHeading = 0;
        boolean wasTurning = false;
        double kp = 0.03; //TODO tune for pid
        int button_count = 0;
        double servo_pos = 0.0;

        // set position to 0 in init
        gateServo.setPower(GATE_KP * (gateServoFeedback.getVoltage()-GATE_VOLTS[0]));


        while (opModeIsActive()) {
//            telemetry.addData("x", follower.pose().x());
//            telemetry.addData("y", follower.pose().y());
//            telemetry.addData("upper_flywheel: ", upperFlywheelMot.getVelocity());
//            telemetry.addData("lower_flywheel: ", lowerFlywheelMot.getVelocity());
//            telemetry.addData("heading", follower.pose().heading());

//            telemetry.addData("Transfer Motor Power", transferWheelMot.getPower());
//            telemetry.addData("Transfer Current (A)", transferWheelMot.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Transfer Velocity (ticks/sec)", transferWheelMot.getVelocity());
//            telemetry.addData("intake current ", intakeMot.getCurrent(CurrentUnit.AMPS));

            Pose currPose = follower.pose();
            DashboardDraw.drawRobot(currPose);

            intakeMot.setPower(gamepad2.left_stick_y);

            // if intake runs, put gate back in its blocking position
            if (Math.abs(gamepad2.left_stick_y) >= 0.05) {
                gateIndex = 0;
            }

            if (gamepad2.rightBumperWasPressed()) {
                transferWheelMot.setVelocity(-TRANSFER_SPEED);
            }

            if (gamepad2.a) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);
            } else if (gamepad2.aWasReleased()) {
                transferWheelMot.setVelocity(WHEEL_STOP_SPEED);
            }



            if (gamepad2.xWasPressed()) {
                transferWheelMot.setVelocity(TRANSFER_SPEED);
                gateIndex++;
                if (gateIndex >= GATE_VOLTS.length) {
                    gateIndex = 0;
                }
            }
            double targetVoltage = GATE_VOLTS[gateIndex];
            double currentVoltage = gateServoFeedback.getVoltage();
            gateError = currentVoltage - targetVoltage;

            gateServo.setPower(GATE_KP * gateError);
            // current check
//            if(currentVoltage >= 3.25 || currentVoltage <= 0.04) {
//                gateServo.setPower(0.0);
//            }

            // prevents gate from being pushed too far and doing a full revolution to get back
            if(Math.abs(gateError) < GATE_VOLT_TOLERANCE) {
                gateServo.setPower(0.0);
            }

            if (gamepad2.yWasPressed()) {
                lowerFlywheelMot.setVelocity(LOWER_LAUNCHER_SPEED);
                upperFlywheelMot.setVelocity(UPPER_LAUNCHER_SPEED);
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

            if (gamepad1.aWasPressed()) {
                follower.setPose(new Pose(12, 12, 0));
            }

            if(!follower.isBusy()) {
                double slowModeMultiplier = doSLOWMODE();

                double drive = 0;
                double strafe = 0;
                double turn = 0;

                // added a 0.05 deadzone in both directions for driver controls
                if (Math.abs(gamepad1.left_stick_y) > DRIVING_CONTROL_DEADZONE || Math.abs(gamepad1.left_stick_x) > DRIVING_CONTROL_DEADZONE || Math.abs(gamepad1.right_stick_x) > DRIVING_CONTROL_DEADZONE) {
                    drive = -gamepad1.left_stick_y * slowModeMultiplier;
                    strafe = -gamepad1.left_stick_x * slowModeMultiplier;
                }

                if (gamepad1.right_bumper) {

                    // Auto aim while button is held
                    turn = autoAim(currPose);

                } else {

                    // Normal manual turning
                    turn = -gamepad1.right_stick_x * slowModeMultiplier;
                }


//                telemetry.addData("turn: ", turn );
//                telemetry.addData("lower flywheel motor: ", lowerFlywheelMot.getVelocity());
//                telemetry.addData("upper flywheel motor: ", upperFlywheelMot.getVelocity());
//                telemetry.addData("gate servo fb: ", gateServoFeedback.getVoltage());
//                telemetry.addData("Gate Target", targetVoltage);
//                telemetry.addData("Gate Current", currentVoltage);
//                telemetry.addData("Gate Index", gateIndex);
                telemetry.addData("drive:", drive);
                telemetry.addData("strafe:", strafe);
                telemetry.addData("turn:", turn);


                // hold the heading of the robot at the turn heading driver has set it to
                // automatic steering correction for driving forward/backward and strafing
//                if (Math.abs(gamepad1.right_stick_x) > 0.05) {
//                    targetHeading = currentHeading;
//                    wasTurning = true;
//                } else {
//                    // set new target heading as soon as driver stops actively turning
//                    if (wasTurning) {
//                        targetHeading = currentHeading;
//                        wasTurning = false;
//                    }
//                }

//                double angleError = targetHeading - currentHeading;
//                // set angle error to be in range 0 to 360 because imu reads from -180 to 180
//                while (angleError > 180)  angleError -= 360;
//                while (angleError < -180) angleError += 360;

//                turn = angleError * kp;

                follower.manual(
                        drive,
                        strafe,
                        turn
                );
            }


            follower.update();
            telemetry.update();

        }
    }

    public double autoAim(Pose currPose) {
        double deltaX = aimTargetX - currPose.x();
        double deltaY = aimTargetY - currPose.y();

        // both targetHeading and headingError are in RADIANS
        double targetHeading = Math.atan2(deltaY, deltaX);
        double headingError = targetHeading - currPose.heading();

        // normalize to -pi to pi
        headingError = Math.atan2(
                Math.sin(headingError),
                Math.cos(headingError)
        );
        double turn = AUTO_AIM_KP * headingError;
        turn = Math.max(-1, Math.min(1, turn));

        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Current Heading", currPose.heading());
        telemetry.addData(" Turn Power", turn);


        return turn;

    }

    public double doSLOWMODE() {
        if (gamepad1.right_trigger != 0) {
            return ((-gamepad1.right_trigger + 1)/2) + 0.5;
        } else {
            return 1;
        }
    }

}


