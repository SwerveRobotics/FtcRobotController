package org.firstinspires.ftc.teamcode.DriveABotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {

  private DcMotor MotorR;
  private DcMotor MotorL;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    MotorR = hardwareMap.get(DcMotor.class, "mot_right");
    MotorL = hardwareMap.get(DcMotor.class, "mot_left");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    MotorR.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        MotorL.setPower(-(gamepad1.left_stick_y / 1));
        MotorR.setPower(-(gamepad1.right_stick_y / 1));
        telemetry.addData("Left Pow", MotorL.getPower());
        telemetry.addData("Right Pow", MotorR.getPower());
        telemetry.update();
      }
    }
  }
}
