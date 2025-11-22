package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.RobotAction;

/**
 * This class contains all of the base logic that is shared between all of the TeleOp and
 * Autonomous logic. All TeleOp and Autonomous classes should derive from this class.
 */
abstract public class BaseOpMode extends LinearOpMode {


    public DcMotorEx launcher = null;
    public CRServo leftFeeder = null;
    public CRServo rightFeeder = null;
    public static final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FEED_TIME_SECONDS = 0; //The feeder servos run this long when a shot is requested.

    public static double FEED_TIME_LOW = 0.15;
    public static double FEED_TIME_SORT = 0.07;


    public static double rememberVelocity = 0;

    public static double FULL_SPEED = 1.0; //We send this power to the servos when we want them to feed an artifact to the launcher
    public static double SLOW_REV_SPEED = -0.15; //speed for the constant reverse rotation
    public static double REV_SPEED = -1.0;//speed used for the reverse launch function
    public static double LAUNCHER_HIGH_MAX_VELOCITY = 1480; //high target velocity + 50 (will need adjusting)
    public static double LAUNCHER_HIGH_TARGET_VELOCITY = 1430;
    public static double LAUNCHER_HIGH_MIN_VELOCITY = 1380;

    public static double LAUNCHER_LOW_MAX_VELOCITY = 1270; //low target velocity + 50 (will need adjusting)
    public static double LAUNCHER_LOW_TARGET_VELOCITY = 1220;
    public static double LAUNCHER_LOW_MIN_VELOCITY = 1170;

    public static double LAUNCHER_SORTER_MAX_VELOCITY = 550; //sorter target velocity + 50 (will need adjusting)
    public static double LAUNCHER_SORTER_TARGET_VELOCITY = 500;
    public static double LAUNCHER_SORTER_MIN_VELOCITY = 450;

    public double ROBOT_WIDTH = 0;
    public double ROBOT_LENGTH = 0;

    public static double LAUNCHER_REV_TARGET_VELOCITY = -250;


    public LED redLed;
    public LED greenLed;

    ElapsedTime feederTimer = new ElapsedTime();

    public String CURRENT_LAUNCHSTATE = "IDLE";
    public enum LaunchState {
        IDLE,
        HIGH,
        LOW,
        SORT,
        LAUNCH,
        LAUNCHING,
    }

    public LaunchState launchState;

    public void initHardware() {
        launchState = LaunchState.IDLE;

        /*
        * Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step.
        */
        // leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        // rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // initialize flywheel motor and feeder servos
        launcher = hardwareMap.get(DcMotorEx.class, "motLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "servoBLaunchFeed");
        rightFeeder = hardwareMap.get(CRServo.class, "servoFLaunchFeed");


        // Reversed direction of launcher for DevBot because motor is on the other side (compared to FastBot)
        if (MecanumDrive.isDevBot) {
            launcher.setDirection(DcMotorEx.Direction.REVERSE);
            ROBOT_LENGTH = 18.5;
            ROBOT_WIDTH = 18;
            redLed = null;
            greenLed = null;

        }
        else if(MecanumDrive.isFastBot) {
            ROBOT_WIDTH = 16;
            ROBOT_LENGTH = 17;
            //redLed = hardwareMap.get(LED.class, "redLed");   Uncomment one we get LEDs
            //greenLed = hardwareMap.get(LED.class, "greenLed");
            //redLed.on();
            //greenLed.off();
        }


        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the flywheel to a braking behavior so it slows down faster when left trigger is pressed
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the feeder servos to an initial value to init the servo controller
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        //set the left feeder servo to rotate in reverse, so that the servos spin in the same relative direction
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);


        //  Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    class WaitAction extends RobotAction {
        RobotAction actionToWaitOn;
        WaitAction(RobotAction actionToWaitOn) {
            this.actionToWaitOn = actionToWaitOn;
        }

        @Override
        public boolean run(double elapsedTime) {
            return actionToWaitOn.isRunning();
        }
    }
    class LaunchAction extends RobotAction {
        public boolean run(double ElapsedTime) {
            if (ElapsedTime < 0.25) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);

                return true;
            }
            else if(ElapsedTime < 2) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                return true;
            }
            else {
                return false;
            }
        }


    }
    class SpinUpAction extends RobotAction {
        private double launcherVelocity;
        public SpinUpAction(double launcherVelocity) {
            this.launcherVelocity = launcherVelocity;
        }
        @Override
        public boolean run(double ElapsedTime) {
            launcher.setVelocity(launcherVelocity);
            return ElapsedTime < 2;
        }


    }

    public void launch(boolean shotRequested) {


        double launcherVelocity = launcher.getVelocity();
        switch (launchState) {

            case IDLE:
                leftFeeder.setPower(SLOW_REV_SPEED);
                rightFeeder.setPower(SLOW_REV_SPEED);
                CURRENT_LAUNCHSTATE = "IDLE";
            break;

            case SORT: //if sorting launchmode is selected and shotRequested is true, check that the flywheel is in the correct velocity range (450 - 500 rpm)
                leftFeeder.setPower(SLOW_REV_SPEED);
                rightFeeder.setPower(SLOW_REV_SPEED);
                if (shotRequested) {
                    CURRENT_LAUNCHSTATE = "SORT";
                    launcher.setVelocity(LAUNCHER_SORTER_TARGET_VELOCITY);
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    if (launcherVelocity > LAUNCHER_SORTER_MIN_VELOCITY && launcherVelocity < LAUNCHER_SORTER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;

                    }
                }
                break;

            case LOW: //if low launchmode is selected and shotRequested is true, check that the flywheel is in the correct velocity range (1075 - 1175 rpm)
                leftFeeder.setPower(SLOW_REV_SPEED);
                rightFeeder.setPower(SLOW_REV_SPEED);
                if (shotRequested) {
                    CURRENT_LAUNCHSTATE = "LOW";
                    launcher.setVelocity(LAUNCHER_LOW_TARGET_VELOCITY);
                    if (launcherVelocity > LAUNCHER_LOW_MIN_VELOCITY && launcherVelocity < LAUNCHER_LOW_MAX_VELOCITY) {
                        leftFeeder.setPower(STOP_SPEED);
                        rightFeeder.setPower(STOP_SPEED);
                        if (redLed != null && greenLed != null) {
                            redLed.off();
                            greenLed.on();
                        }
                        launchState = LaunchState.LAUNCH;
                    }
                }
            break;

            case HIGH: //if high launchmode is selected and shotRequested is true, check that the flywheel is in the correct velocity range (1900 - 2000 rpm)
                leftFeeder.setPower(SLOW_REV_SPEED);
                rightFeeder.setPower(SLOW_REV_SPEED);
                if (shotRequested) {
                    CURRENT_LAUNCHSTATE = "HIGH";
                    launcher.setVelocity(LAUNCHER_HIGH_TARGET_VELOCITY);
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    if (launcherVelocity > LAUNCHER_HIGH_MIN_VELOCITY && launcherVelocity < LAUNCHER_HIGH_MAX_VELOCITY) {
                        if (redLed != null && greenLed != null) {
                            redLed.off();
                            greenLed.on();
                        }
                        launchState = LaunchState.LAUNCH;
                    }
                }
                break;
            case LAUNCH: //when shotRequested, start the feeder servos to init launch
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING: //wait until feedTimer surpasses FEED_TIME_SECONDS, then stop the feeder servos.
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);

                }
                leftFeeder.setPower(SLOW_REV_SPEED);
                rightFeeder.setPower(SLOW_REV_SPEED);
                if (redLed != null && greenLed != null) {
                    redLed.off();
                    greenLed.on();
                }
                if (CURRENT_LAUNCHSTATE.equals("LOW") ) {
                   launchState = LaunchState.LOW;
                } else if (CURRENT_LAUNCHSTATE.equals("HIGH")) {
                    launchState = LaunchState.HIGH;

                } else if (CURRENT_LAUNCHSTATE.equals("SORT")) {
                    launchState = LaunchState.SORT;
                } else {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}

