package org.firstinspires.ftc.teamcode.DriveABotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ArcadeDrive")
public class ArcadeDrive extends LinearOpMode {

  private DcMotor MotorR;
  private DcMotor MotorL;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    MotorR = hardwareMap.get(DcMotor.class, "mot_right");
    MotorL = hardwareMap.get(DcMotor.class, "mot_left");


    MotorR.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();

    if (opModeIsActive()) {

      while (opModeIsActive()) {

        MotorL.setPower(-(gamepad1.left_stick_y + gamepad1.right_stick_x));
        MotorR.setPower(-(gamepad1.left_stick_y - gamepad1.right_stick_x));
        telemetry.addData("Left Pow", MotorL.getPower());
        telemetry.addData("Right Pow", MotorR.getPower());
        telemetry.update();
      }
    }
  }
}
