package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name="telemetry format test", group="amogus1")
public class telemetry_format_test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        telemetry.addLine("<b>text</b>");
        telemetry.addLine("This is <font color='red'>red</font> and this is <font color='blue'>blue</font>.");

        telemetry.addLine("test words! These are some WORDS..?");
        telemetry.addLine("0123456789012345678901234567890123456789");
        telemetry.addLine("Who knows if THIS will align correctly?");
        telemetry.addLine("0123456789012345678901234567890123456789");
        telemetry.addLine("fancy char: ➤[x]➤[x]1➤[2]➤[3]");
        telemetry.update();

        while (opModeIsActive()) {
        }
    }
}
