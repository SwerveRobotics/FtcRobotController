package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

/**
 * This class exposes the competition version of Autonomous. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@Autonomous(name = "Reliable Auto", group = "Competition", preselectTeleOp = "CompetitionTeleOp")
public class ReliableAuto extends BaseOpMode {
    ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Send all telemetry data to both the Driver Hub and FTC Dashboard:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(kinematicType, hardwareMap, telemetry, gamepad1, beginPose);

        // Wait for Start to be pressed on the Driver Hub!
        waitForStart();

        clock.reset();

        while (opModeIsActive()) {
            telemetry.addLine("Running Reliable Auto!");
            telemetry.addData("Kinematic Type", kinematicType);
            telemetry.addData("Elapsed time", clock.milliseconds());
            telemetry.update();
            if (clock.milliseconds() < 3000) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                0.2
                        ),
                        0
                ));
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                0
                        ),
                        0
                ));
            }
        }
    }
}
