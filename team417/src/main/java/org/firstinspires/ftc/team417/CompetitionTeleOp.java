package org.firstinspires.ftc.team417;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.HolonomicDrive;
import org.firstinspires.ftc.team417.roadrunner.KinematicType;

/**
 * This class exposes the competition version of TeleOp. As a general rule, add code to the
 * BaseOpMode class rather than here so that it can be shared between both TeleOp and Autonomous.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class CompetitionTeleOp extends BaseOpMode {
    public final KinematicType kinematicType = KinematicType.MECANUM;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        HolonomicDrive drive = new HolonomicDrive(hardwareMap, telemetry, gamepad1, new Pose2d(0, 0, 0), kinematicType);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addLine("Running 417's TeleOp!");
            telemetry.update();

            TelemetryPacket packet = HolonomicDrive.getTelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            HolonomicDrive.sendTelemetryPacket(packet);
        }
    }
}
