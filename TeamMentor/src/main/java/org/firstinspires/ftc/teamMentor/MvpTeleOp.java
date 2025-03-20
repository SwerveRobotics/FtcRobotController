package org.firstinspires.ftc.teamMentor;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamMentor.roadrunner.MecanumDrive;

@TeleOp(name = "TeleOp", group = "MentorBot")
public class MvpTeleOp extends LinearOpMode {

    // Shape the stick input for more precision at slow speeds:
    static double shapeStick(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Poser poser = Poser.getPoser(hardwareMap, telemetry, gamepad1, null);
        Arm arm = new Arm(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, poser, telemetry, gamepad1);

        telemetry.addLine("Ready to start!");
        telemetry.update();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        while (opModeIsActive()) {
            Stats.beginLoop(Stats.Mode.REGULAR);
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            PoseVelocity2d velocity = new PoseVelocity2d(new Vector2d(
                    shapeStick(-gamepad1.left_stick_y),
                    shapeStick(-gamepad1.left_stick_x)),
                    shapeStick(-gamepad1.right_stick_x));

            drive.setDrivePowers(velocity);

            // Spew the stats:
            telemetry.addLine(Stats.getString());

            // We're done!
            MecanumDrive.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
