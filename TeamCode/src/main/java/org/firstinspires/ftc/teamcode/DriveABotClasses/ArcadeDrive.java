package org.firstinspires.ftc.teamcode.DriveABotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@TeleOp(name = "ArcadeDrive")
public class ArcadeDrive extends LinearOpMode {

  private DcMotor MotorR;
  private DcMotor MotorL;
  ElapsedTime timer = new ElapsedTime();
  int timeCapSeconds = 120;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    MotorR = hardwareMap.get(DcMotor.class, "mot_right");
    MotorL = hardwareMap.get(DcMotor.class, "mot_left");

    MotorR.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    timer.reset();

    if (opModeIsActive()) {

      while (opModeIsActive() && timer.seconds() <= timeCapSeconds) {
        MotorL.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        MotorR.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
        telemetry.addData("Time Remaining", convertSecondsToString(timeCapSeconds - (int)timer.seconds()));
        telemetry.update();
      }
    }
  }

  private String convertSecondsToString(int seconds) {
    String str = "";
    if (seconds >= 60) {
      str += seconds / 60 + "m ";
    }
    str += seconds % 60 + "s";
    return str;
  }
}
