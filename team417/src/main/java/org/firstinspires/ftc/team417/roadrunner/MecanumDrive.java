package org.firstinspires.ftc.team417.roadrunner;

import static java.lang.Math.max;
import static java.lang.System.nanoTime;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team417.BaseOpModeFastBot;
import org.firstinspires.ftc.team417.DriveParameters;
import org.firstinspires.ftc.team417.distance.DistanceLocalizer;
import org.firstinspires.ftc.team417.distance.DistanceSensorInfo;
import org.firstinspires.ftc.team417.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.team417.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.team417.roadrunner.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.team417.roadrunner.messages.PoseMessage;
import org.firstinspires.inspection.InspectionState;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;
import org.swerverobotics.ftc.GoBildaPinpointDriver;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public final KinematicType kinematicType;

    public static class Params {
        Params() {
            maxWheelVel = 50; // was 55
            minProfileAccel = -30;
            maxProfileAccel = 50;

            maxAngVel = Math.PI;
            maxAngAccel = Math.PI;

            switch (driveParameters) {
                case DEVBOT_MECANUM:
                    // Your DevBot Loony Tune configuration is here:
                    logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                    usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                    inPerTick = 1.0;
                    lateralInPerTick = 0.819; // Was 1.00
                    trackWidthTicks = 22.29; // Was 0.00

                    kS = 0.608; // Was 0
                    kV = 0.188; // Was 0
                    kA = 0;

                    axialGain = 6.00; // Was 0
                    axialVelGain = 0.70; // Was 0
                    lateralGain = 4.00; // Was 0.00
                    lateralVelGain = 3.00; // Was 0.00
                    headingGain = 9.0; // Was 0.0
                    headingVelGain = 0;

                    otos.offset.x = 6.115; // Was 0.00
                    otos.offset.y = 3.031; // Was 0.00
                    otos.offset.h = Math.toRadians(-89.71); // Was Math.toRadians(0)
                    otos.linearScalar = 1.000; // Was 0.000
                    otos.angularScalar = 1.0005;

                    pinpoint.ticksPerMm = 0;
                    pinpoint.xReversed = false;
                    pinpoint.yReversed = false;
                    pinpoint.xOffset = 0;
                    pinpoint.yOffset = 0;
                    break;

                case DEVBOT_X:
                    // Your competition robot Looney Tune configuration is here:
                    logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                    usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                    inPerTick = 1;
                    lateralInPerTick = inPerTick;
                    trackWidthTicks = 0;

                    kS = 0;
                    kV = 0;
                    kA = 0;

                    axialGain = 0.0;
                    axialVelGain = 0.0;
                    lateralGain = 0.0;
                    lateralVelGain = 0.0;
                    headingGain = 0.0;
                    headingVelGain = 0.0;

                    otos.offset.x = 0;
                    otos.offset.y = 0;
                    otos.offset.h = Math.toRadians(0);
                    otos.linearScalar = 0;
                    otos.angularScalar = 0;

                    pinpoint.ticksPerMm = 0;
                    pinpoint.xReversed = false;
                    pinpoint.yReversed = false;
                    pinpoint.xOffset = 0;
                    pinpoint.yOffset = 0;

                    break;

                case FASTBOT_MECANUM:
                    // Your competition robot Loony Tune configuration is here:
                    logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                    usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                    inPerTick = 1;
                    lateralInPerTick = 0.819;
                    trackWidthTicks = 14.63;

                    kS = 0.667; // Was 0.759
                    kV = 0.194; // Was 0.192
                    kA = 0.0200;

                    axialGain = 9.0;
                    axialVelGain = 2.0;
                    lateralGain = 2.0;
                    lateralVelGain = 0.40;
                    headingGain = 15.00;
                    headingVelGain = 0.0;

                    otos.offset.x = 6.011;
                    otos.offset.y = -2.784;
                    otos.offset.h = Math.toRadians(90.50); // Was 91.19
                    otos.linearScalar = 0.9801; // Was 0.989
                    otos.angularScalar = 0.9928;

                    pinpoint.ticksPerMm = 0;
                    pinpoint.xReversed = false;
                    pinpoint.yReversed = false;
                    pinpoint.xOffset = 0;
                    pinpoint.yOffset = 0;
                    break;

                case COMPETITION_ROBOT:
                    // Your DevBot Looney Tune configuration is here:
                    logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                    usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                    inPerTick = 1.0;
                    lateralInPerTick = 0.700; // Was 1.00
                    trackWidthTicks = 22.29; // Was 0.00

                    kS = 0.608; // Was 0
                    kV = 0.188; // Was 0
                    kA = 0;

                    axialGain = 6.00; // Was 0
                    axialVelGain = 0.70; // Was 0
                    lateralGain = 4.00; // Was 0.00
                    lateralVelGain = 3.00; // Was 0.00
                    headingGain = 9.0; // Was 0.0
                    headingVelGain = 0;

                    otos.offset.x = 6.115; // Was 0.00
                    otos.offset.y = 3.031; // Was 0.00
                    otos.offset.h = Math.toRadians(-89.71); // Was Math.toRadians(0)
                    otos.linearScalar = 1.000; // Was 0.000
                    otos.angularScalar = 1.0005;

                    pinpoint.ticksPerMm = 0;
                    pinpoint.xReversed = false;
                    pinpoint.yReversed = false;
                    pinpoint.xOffset = 0;
                    pinpoint.yOffset = 0;
                    break;

            }
        }

        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection; // IMU orientation
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection; // IMU orientation

        public double maxWheelVel; // Maximum velocity a wheel can achieve, inches/s
        public double minProfileAccel; // How fast the robot can accelerate, in inches/s/s
        public double maxProfileAccel; // How fast the robot can decelerate, in inches/s/s

        public double maxAngVel; // Maximum angular velocity, radians/s
        public double maxAngAccel; // How fast the robot can accelerate and decelerate turning, radians/s/s

        public double inPerTick; // Inches-per-tick for encoders (set to 1.0 if using Optical Tracking)
        public double lateralInPerTick; // Lateral inches-per-tick for encoders
        public double trackWidthTicks; // Diameter of the circle that a wheel travels to turn the robot 360 degrees, in ticks

        public double kS; // Feed-forward voltage to overcome static friction
        public double kV; // Feed-forward voltage factor to achieve target velocity, in tick units
        public double kA; // Feed-forward voltage factor to achieve target acceleration, in tick units

        public double axialGain; // Path controller proportional gain, scalar
        public double lateralGain; // Path controller proportional gain, scalar
        public double headingGain; // Path controller proportional gain, scalar

        public double axialVelGain; // Path controller derivative gain, scalar
        public double lateralVelGain; // Path controller derivative gain, scalar
        public double headingVelGain; // Path controller derivative gain, scalar

        public Otos otos = new Otos();
        public Pinpoint pinpoint = new Pinpoint();

        // Structure for the settings of the SparkFun Optical Tracking Odometry Sensor.
        public static class Otos {
            // Inches, inches and radians:
            public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
            public double linearScalar; // Scalar
            public double angularScalar; // Scalar
        }

        // Structure for the settings of the goBilda Pinpoint Odometry Computer.
        public static class Pinpoint {
            public double ticksPerMm; // Millimeters per tick
            public boolean xReversed; // True if the x-encoder (forward) is reversed, false if forward
            public boolean yReversed; // True if the y-encoder (strafe) is reversed, false if forward
            public double xOffset; // Mm offset from center of rotation to x-encoder (forward)
            public double yOffset; // Mm offset from center of rotation to y-encoder (strafe)
        }
    }

    public static String getBotName() {
        InspectionState inspection = new InspectionState();
        inspection.initializeLocal();
        Log.d("roadrunner", String.format("Device name:" + inspection.deviceName));
        return inspection.deviceName;
    }

    private static DriveParameters getDriveParameters() {
        switch (getBotName()) {
            case "DevBot":
                return DriveParameters.DEVBOT_MECANUM;
            case "We don't have this robot":
                return DriveParameters.DEVBOT_X;
            case "417-RC":
                return DriveParameters.FASTBOT_MECANUM;
            case "417-S-RC":
                return DriveParameters.COMPETITION_ROBOT;
        }
        return null; // Not one of 417's robots
    }

    public static DriveParameters driveParameters = getDriveParameters();

    public static Params PARAMS = new Params();

    public HolonomicKinematics kinematics; // Initialized by initializeKinematics()
    public VelConstraint defaultVelConstraint; // Initialized by initializeKinematics()
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public LazyImu lazyImu;

    public Localizer localizer;
    public Pose2d pose; // Current actual pose
    public PoseVelocity2d poseVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0); // Current velocity
    public Pose2d targetPose; // Target pose when actively traversing a trajectory
    public SparkFunOTOS otosDriver; // Can be null which means no OTOS
    public GoBildaPinpointDriver pinpointDriver; // Can be null which means no Pinpoint

    public double lastLinearGainError = 0; // Most recent gain error in inches and radians
    public double lastHeadingGainError = 0;
    public double maxLinearGainError = 0; // Max gain error to date in inches and radians
    public double maxHeadingGainError = 0;

    public DistanceLocalizer distanceLocalizer;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public Encoder leftFront, leftBack, rightBack, rightFront;
        public IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            // imu = lazyImu.get(); // NOT NEEDED with OTOS/Pinpoint
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new HolonomicKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    // Constructor for the Mecanum Drive object.
    public MecanumDrive(KinematicType kinematicType, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, Pose2d pose) {
        this.kinematicType = kinematicType;
        initializeKinematics();

        this.pose = pose;

        kinematics.kinematicType = kinematicType;

        WilyWorks.setStartPose(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        configure(hardwareMap);

        // Enable brake mode on the motors:
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);

        // Now that configuration is complete, verify the parameters:
        LoonyTune.verifyCodeMatchesTuneResults(this, telemetry, gamepad);
    }

    // This is where you configure Road Runner to work with your hardware:
    public void configure(HardwareMap hardwareMap) {
        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        switch (driveParameters) {
            case DEVBOT_MECANUM:
                //otosDriver = hardwareMap.get(SparkFunOTOS.class, "otos");

                if (BaseOpModeFastBot.USE_DISTANCE) {
                    UltrasonicDistanceSensor leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
                    UltrasonicDistanceSensor rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");
                    DistanceSensorInfo leftInfo = new DistanceSensorInfo(-4.5, 7, -Math.PI / 9);
                    DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.5, 0, Math.PI / 2);
                    distanceLocalizer = new DistanceLocalizer(leftSonic, leftInfo, rightSonic, rightInfo, this);
                }

                leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
                leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
                rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
                rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

                leftFront.setDirection(DcMotorEx.Direction.REVERSE);
                leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                break;

            case DEVBOT_X:
                // TODO: Create the optical tracking object:
                //   opticalTracking = hardwareMap.get(SparkFunOTOS.class, "optical");
                //   initializeOtosDriver();

                if (BaseOpModeFastBot.USE_DISTANCE) {
                    // TODO: Create the distance sensor tracking object:
                    //   UltrasonicDistanceSensor leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
                    //   UltrasonicDistanceSensor rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");
                    //   DistanceSensorInfo leftInfo = new DistanceSensorInfo(-6.75, 7.75, -0.25 * Math.PI);
                    //   DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.75, 7.75, 0.25 * Math.PI);
                    //   distanceLocalizer = new DistanceLocalizer(leftSonic, leftInfo, rightSonic, rightInfo, this);
                }

                leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
                leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
                rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
                rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
                leftFront.setDirection(DcMotorEx.Direction.REVERSE);
                leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                break;

            case FASTBOT_MECANUM:
                // enable this when we have the optical tracker
                otosDriver = hardwareMap.get(SparkFunOTOS.class, "otos");

                if (BaseOpModeFastBot.USE_DISTANCE) {
                    UltrasonicDistanceSensor leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
                    UltrasonicDistanceSensor rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");
                    DistanceSensorInfo leftInfo = new DistanceSensorInfo(-4.5, 7, -Math.PI / 9);
                    DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.5, 0, Math.PI / 2);
                    distanceLocalizer = new DistanceLocalizer(leftSonic, leftInfo, rightSonic, rightInfo, this);
                }

                leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
                leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
                rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
                rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

                rightFront.setDirection(DcMotorEx.Direction.REVERSE);
                rightBack.setDirection(DcMotorEx.Direction.REVERSE);
                leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                break;

            case COMPETITION_ROBOT:
                //otosDriver = hardwareMap.get(SparkFunOTOS.class, "otos");
                //initializeOtosDriver();

                if (BaseOpModeFastBot.USE_DISTANCE) {
                    // TODO: Create the distance sensor tracking object:
                    //   UltrasonicDistanceSensor leftSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "leftSonic");
                    //   UltrasonicDistanceSensor rightSonic = hardwareMap.get(UltrasonicDistanceSensor.class, "rightSonic");
                    //   DistanceSensorInfo leftInfo = new DistanceSensorInfo(-6.75, 7.75, -0.25 * Math.PI);
                    //   DistanceSensorInfo rightInfo = new DistanceSensorInfo(6.75, 7.75, 0.25 * Math.PI);
                    //   distanceLocalizer = new DistanceLocalizer(leftSonic, leftInfo, rightSonic, rightInfo, this);
                }

                leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
                leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
                rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
                rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

                leftFront.setDirection(DcMotorEx.Direction.REVERSE);
                leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                break;
        }

        // Initialize the tracking drivers, if any:
        initializeOtosDriver();
        initializePinpointDriver();

        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        localizer = new DriveLocalizer();
    }

    // Initialize the Pinpoint sensor if we have one.
    public void initializePinpointDriver() {
        if (pinpointDriver == null)
            return; // ====>

        pinpointDriver.setOffsets(PARAMS.pinpoint.xOffset, PARAMS.pinpoint.yOffset);
        pinpointDriver.setEncoderResolution(PARAMS.pinpoint.ticksPerMm);
        pinpointDriver.setEncoderDirections(
                PARAMS.pinpoint.xReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                PARAMS.pinpoint.yReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Now that the Pinpoint is configured, tell it the pose:
        setPose(pose);
    }

    // Initialize the OTOS sensor if we have one. Derived from configureOtos(). The
    // pose is the initial pose where the robot will start on the field.
    public void initializeOtosDriver() {
        if (otosDriver == null)
            return; // ====>

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        otosDriver.setLinearUnit(DistanceUnit.INCH);
        otosDriver.setAngularUnit(AngleUnit.RADIANS);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        otosDriver.setOffset(PARAMS.otos.offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otosDriver.setLinearScalar(PARAMS.otos.linearScalar);
        otosDriver.setAngularScalar(PARAMS.otos.angularScalar);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otosDriver.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otosDriver.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        setPose(pose);
    }

    // Set the drive powers for when driving manually via the controller:
    public void setDrivePowers(PoseVelocity2d powers) {
        // If running under Wily Works, request the drive powers directly:
        if (WilyWorks.setDrivePowers(powers, new PoseVelocity2d(new Vector2d(0, 0), 0)))
            return; // ====>

        HolonomicKinematics.WheelVelocities<Time> wheelVels = new HolonomicKinematics(kinematicType, 1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    // Used by setDrivePowers to calculate acceleration:
    PoseVelocity2d previousAssistVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    double previousAssistSeconds = 0; // Previous call's nanoTime() in seconds

    /**
     * Power the motors according to the specified velocities. 'stickVelocity' is for controller
     * input and 'assistVelocity' is for computed driver assistance. The former is specified in
     * voltage values normalized from -1 to 1 (just like the regular DcMotor::SetPower() API)
     * whereas the latter is in inches/s or radians/s. Both types of velocities can be specified
     * at the same time in which case the velocities are added together (to allow assist and stick
     * control to blend together, for example).
     * <br>
     * It's also possible to map the controller input to inches/s and radians/s instead of the
     * normalized -1 to 1 voltage range. You can reference MecanumDrive.PARAMS.maxWheelVel and
     * .maxAngVel to determine the range to specify. Note however that the robot can actually
     * go faster than Road Runner's PARAMS values so you would be unnecessarily slowing your
     * robot down.
     *
     * @noinspection unused
     */
    public void setDrivePowers(
            // Current pose:
            Pose2d pose,
            // Current velocity:
            PoseVelocity2d poseVelocity,
            // Desired manual power velocity, normalized voltage from -1 to 1, robot-relative
            // coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Desired computed power velocity, inches/s and radians/s, field-relative coordinates,
            // can be null:
            PoseVelocity2d assistVelocity) {
        if (stickVelocity == null)
            stickVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (assistVelocity == null)
            assistVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // If running under Wily Works, request the drive powers directly:
        if (WilyWorks.setDrivePowers(stickVelocity, assistVelocity))
            return; // ====>

        // Compute the assist acceleration as the difference between the new assist velocity
        // and the old divided by delta-t:
        double currentSeconds = nanoTime() * 1e-9;
        //noinspection ExtractMethodRecommender
        PoseVelocity2d assistAcceleration = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (previousAssistSeconds != 0) {
            double deltaT = currentSeconds - previousAssistSeconds;
            assistAcceleration = new PoseVelocity2d(new Vector2d(
                    (assistVelocity.linearVel.x - previousAssistVelocity.linearVel.x) / deltaT,
                    (assistVelocity.linearVel.y - previousAssistVelocity.linearVel.y) / deltaT),
                    (assistVelocity.angVel - previousAssistVelocity.angVel) / deltaT);
        }
        previousAssistSeconds = currentSeconds;

        // Remember the current velocity for next time:
        previousAssistVelocity = new PoseVelocity2d(new Vector2d(
                assistVelocity.linearVel.x, assistVelocity.linearVel.y), assistVelocity.angVel);

        // Compute the wheel powers for the stick contribution:
        HolonomicKinematics.WheelVelocities<Time> manualVels = new HolonomicKinematics(kinematicType, 1).inverse(
                PoseVelocity2dDual.constant(stickVelocity, 1));

        double leftFrontPower = manualVels.leftFront.get(0);
        double leftBackPower = manualVels.leftBack.get(0);
        double rightBackPower = manualVels.rightBack.get(0);
        double rightFrontPower = manualVels.rightFront.get(0);

        // Compute the wheel powers for the assist:
        double[] x = {pose.position.x, assistVelocity.linearVel.x, assistAcceleration.linearVel.x};
        double[] y = {pose.position.y, assistVelocity.linearVel.y, assistAcceleration.linearVel.y};
        double[] angular = {pose.heading.log(), assistVelocity.angVel, assistAcceleration.angVel};

        Pose2dDual<Time> computedDualPose = new Pose2dDual<>(
                new Vector2dDual<>(new DualNum<>(x), new DualNum<>(y)),
                Rotation2dDual.exp(new DualNum<>(angular)));

        // Compute the feedforward for the assist while disabling the PID portion of the PIDF:
        PoseVelocity2dDual<Time> command = new HolonomicController(0, 0, 0)
                .compute(computedDualPose, pose, poseVelocity);

        HolonomicKinematics.WheelVelocities<Time> assistVels = kinematics.inverse(command);

        double voltage = getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(
                PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

        // Check for zero velocities and accelerations to avoid adding 'kS'. Arguably these
        // should be epsilon compares but equality is fine for checking when the assist is
        // disabled:
        if ((assistVels.leftFront.get(0) != 0) || (assistVels.leftFront.get(1) != 0))
            leftFrontPower += feedforward.compute(assistVels.leftFront) / voltage;

        if ((assistVels.leftBack.get(0) != 0) || (assistVels.leftBack.get(1) != 0))
            leftBackPower += feedforward.compute(assistVels.leftBack) / voltage;

        if ((assistVels.rightBack.get(0) != 0) || (assistVels.rightBack.get(1) != 0))
            rightBackPower += feedforward.compute(assistVels.rightBack) / voltage;

        if ((assistVels.rightFront.get(0) != 0) || (assistVels.rightFront.get(1) != 0))
            rightFrontPower += feedforward.compute(assistVels.rightFront) / voltage;

        // Normalize if any powers are more than 1:
        double maxPower = max(max(max(max(1, leftFrontPower), leftBackPower), rightBackPower), rightFrontPower);

        // Set the power to the motors:
        leftFront.setPower(leftFrontPower / maxPower);
        leftBack.setPower(leftBackPower / maxPower);
        rightBack.setPower(rightBackPower / maxPower);
        rightFront.setPower(rightFrontPower / maxPower);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                resetLoopTimeStatistic();
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // Enlighten Wily Works as to where we should be:
            WilyWorks.runTo(txWorldTarget.value(), txWorldTarget.velocity().value());

            // Remember the target pose:
            targetPose = txWorldTarget.value();

            HolonomicKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double voltage = getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            // Update some statistics:
            updateLoopTimeStatistic(p);
            Pose2d error = txWorldTarget.value().minusExp(pose);
            lastLinearGainError = error.position.norm();
            lastHeadingGainError = error.heading.toDouble();
            maxLinearGainError = Math.max(maxLinearGainError, lastLinearGainError);
            maxHeadingGainError = Math.max(maxHeadingGainError, lastHeadingGainError);

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        public final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
                resetLoopTimeStatistic();
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // Enlighten Wily Works as to where we should be:
            WilyWorks.runTo(txWorldTarget.value(), txWorldTarget.velocity().value());

            // Remember the target pose:
            targetPose = txWorldTarget.value();

            HolonomicKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double voltage = getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            // Update some statistics:
            updateLoopTimeStatistic(p);
            Pose2d error = txWorldTarget.value().minusExp(pose);
            lastLinearGainError = error.position.norm();
            lastHeadingGainError = error.heading.toDouble();
            maxLinearGainError = Math.max(maxLinearGainError, lastLinearGainError);
            maxHeadingGainError = Math.max(maxHeadingGainError, lastHeadingGainError);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            //noinspection SpellCheckingInspection
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    // Update the loop time, in milliseconds, and show it on FTC Dashboard:
    double lastLoopTime; // Seconds

    void updateLoopTimeStatistic(TelemetryPacket p) {
        double currentTime = nanoTime() * 1e-9; // Seconds
        double loopTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;
        p.put("Loop time", loopTime * 1000.0); // Milliseconds
    }

    void resetLoopTimeStatistic() {
        lastLoopTime = nanoTime() * 1e-9;
    }

    public PoseVelocity2d updatePoseEstimate() {
        if (pinpointDriver != null) {
            // Query the driver for position and velocity:
            pinpointDriver.update();
            Pose2D pose2D = pinpointDriver.getPosition();
            Pose2D poseVelocity2D = pinpointDriver.getVelocity();

            // Convert to Road Runner format, remembering that the Pinpoint tracking sensor
            // reports velocity as field-relative but Road Runner wants it robot-relative:
            pose = new Pose2d(
                    pose2D.getX(DistanceUnit.INCH),
                    pose2D.getY(DistanceUnit.INCH),
                    pose2D.getHeading(AngleUnit.RADIANS));
            double xVelocity = poseVelocity2D.getX(DistanceUnit.INCH);
            double yVelocity = poseVelocity2D.getY(DistanceUnit.INCH);
            poseVelocity = new PoseVelocity2d(
                    rotateVector(new Vector2d(xVelocity, yVelocity), -pose.heading.toDouble()),
                    poseVelocity2D.getHeading(AngleUnit.RADIANS));

        } else if (otosDriver != null) {
            // Get the current pose and current pose velocity from the optical tracking sensor.
            // Reads over the I2C bus are very slow so for performance we query the velocity only
            // if we'll actually need it - i.e., if using non-zero velocity gains:
            SparkFunOTOS.Pose2D position = new SparkFunOTOS.Pose2D(0, 0, 0);
            SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D(0, 0, 0);
            SparkFunOTOS.Pose2D acceleration = new SparkFunOTOS.Pose2D(0, 0, 0);

            // This single call is faster than separate calls to getPosition() and getVelocity():
            otosDriver.getPosVelAcc(position, velocity, acceleration);

            // Convert to Road Runner format, remembering that the optical tracking sensor
            // reports velocity as field-relative but Road Runner wants it robot-relative:
            pose = new Pose2d(position.x, position.y, position.h);
            poseVelocity = new PoseVelocity2d(
                    rotateVector(new Vector2d(velocity.x, velocity.y), -pose.heading.toDouble()),
                    velocity.h);
        } else {
            // Use the wheel odometry to update the pose:
            Twist2dDual<Time> twist = WilyWorks.localizerUpdate();
            if (twist == null) {
                twist = localizer.update();
            }

            pose = pose.plus(twist.value());
            poseVelocity = twist.velocity().value();
        }

        if (distanceLocalizer != null) {
            pose = new Pose2d(pose.position.plus(distanceLocalizer.updateIfPossible()), pose.heading.log());
        }

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return poseVelocity;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    // Build a trajectory without any transformations:
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return actionBuilder(beginPose, new IdentityPoseMap());
    }

    // The poseMap is a handy option for transformations (such as mirroring) on all
    // coordinates in the trajectory:
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose, PoseMap poseMap) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                poseMap
        );
    }

    // Create the kinematics object and any dependents from the current settings:
    public void initializeKinematics() {
        kinematics = new HolonomicKinematics(
                kinematicType,
                PARAMS.inPerTick * PARAMS.trackWidthTicks,
                PARAMS.inPerTick / PARAMS.lateralInPerTick);
        defaultVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));
    }

    /**
     * @noinspection unused
     */ // Rotate a vector by a prescribed angle:
    static public Vector2d rotateVector(Vector2d vector, double theta) {
        return new Vector2d(
                Math.cos(theta) * vector.x - Math.sin(theta) * vector.y,
                Math.sin(theta) * vector.x + Math.cos(theta) * vector.y);
    }

    // Override the current pose for Road Runner and the optical tracking sensor:
    public void setPose(Pose2d pose) {
        // Set the Road Runner pose:
        this.pose = pose;
        this.targetPose = pose;

        // Clear the history as it's no longer relevant:
        poseHistory.clear();

        // Set the pose on the optical tracking sensor:
        if (otosDriver != null) {
            otosDriver.setPosition(new SparkFunOTOS.Pose2D(
                    pose.position.x, pose.position.y, pose.heading.toDouble()));
        }
        // Set the pose on the Pinpoint tracking sensor:
        if (pinpointDriver != null) {
            pinpointDriver.setPosition(new Pose2D(
                    DistanceUnit.INCH, pose.position.x, pose.position.y,
                    AngleUnit.RADIANS, pose.heading.toDouble()));
        }
    }

    // Get the current voltage, amortized for performance:
    double previousVoltageSeconds = 0;
    double previousVoltageRead = 0;

    public double getVoltage() {
        final double UPDATE_INTERVAL = 0.1; // Minimum duration between hardware reads, in seconds
        double currentSeconds = nanoTime() * 1e-9;
        if (currentSeconds - previousVoltageSeconds > UPDATE_INTERVAL) {
            previousVoltageSeconds = currentSeconds;
            previousVoltageRead = voltageSensor.getVoltage();
        }
        return previousVoltageRead;
    }

    // List of currently running Actions:
    LinkedList<Action> actionList = new LinkedList<>();

    /**
     * @noinspection unused
     */ // Invoke an Action to run in parallel during TeleOp:
    public void runParallel(Action action) {
        actionList.add(action);
    }

    // On every iteration of your robot loop, call 'doActionsWork' to allow Road Runner Actions
    // to be processed while running TeleOp. Specify the packet if you're drawing on the graph for
    // FTC Dashboard:
    public boolean doActionsWork(TelemetryPacket packet) {
        LinkedList<Action> deletionList = new LinkedList<>();
        for (Action action : actionList) {
            // Let the Action do any field rendering (such as to draw the path it intends to
            // traverse):
            action.preview(packet.fieldOverlay());

            // Once the Action returns false, the action is done:
            if (!action.run(packet))
                // We can't delete an item from a list while we're iterating on that list:
                deletionList.add(action);
        }
        actionList.removeAll(deletionList);
        return !actionList.isEmpty();
    }

    /**
     * @noinspection unused
     */
    // Abort all currently running actions:
    public void abortActions() {
        actionList.clear();
    }

    // Create a new telemetry packet to draw stuff on the FTC Dashboard field.
    public static TelemetryPacket getTelemetryPacket() {
        return getTelemetryPacket(true);
    }

    public static TelemetryPacket getTelemetryPacket(boolean showField) {
        TelemetryPacket packet = new TelemetryPacket();

        // Prepare the packet for drawing.
        //
        // By default, Road Runner draws the field so positive y goes left, positive x
        // goes up. Rotate the field clockwise so that positive positive y goes up, positive x
        // goes right. This rotation is 90 (rather than -90) degrees in page-frame space.
        // Then draw the grid on top and finally set the transform to rotate all subsequent
        // rendering.
        Canvas canvas = packet.fieldOverlay();
        if (showField) {
            canvas.drawImage("/dash/into-the-deep.png", 0, 0, 144, 144,
                    Math.toRadians(90), 0, 144, true);
        } else {
            canvas.setFill("#000000");
            canvas.fillRect(-72, -72, 144, 144);
        }
        canvas.drawGrid(0, 0, 144, 144, 7, 7);

        canvas.setRotation(Math.toRadians(-90));
        // Fade field and grid to white by drawing transparent white over it:
        canvas.setAlpha(0.8);
        canvas.setFill("#ffffff");
        canvas.fillRect(-72, -72, 144, 144);
        canvas.setAlpha(1.0);

        return packet;
    }

    // When done with an FTC Dashboard telemetry packet, send it!
    public static void sendTelemetryPacket(TelemetryPacket packet) {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
