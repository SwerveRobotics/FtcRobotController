package org.firstinspires.ftc.teamcode.session4;

// Import necessary classes from the com.qualcomm.robotcore library:

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// An OpMode is basically a "mode of operation" for the robot.
// It is something you can run on the robot.
// The name should be descriptive, because it will appear on the driver hub.
// Declare that this class is a TeleOp class (i.e. a remote control OpMode) called "Hardware Setup":
@TeleOp(name = "Hardware Setup")
// We are creating a child class of LinearOpMode.
// This allows our code to run in sequence, or linearly.
// Demonstration of how to set up the hardware code for FTC
public class HardwareSetup extends LinearOpMode {
    // Initialize attributes for all the drive motors:
    public DcMotor bl, br, fl, fr;

    // @Override is not necessary but encouraged.
    // This is used when you write a method that is overriding a method from the parent class.
    @Override
    public void runOpMode() {
        // Use initializeMotor(String, DcMotor.Direction) to assign values to the motor attributes:
        bl = initializeMotor("BLMotor", DcMotor.Direction.FORWARD);
        br = initializeMotor("BRMotor", DcMotor.Direction.FORWARD);
        fl = initializeMotor("FLMotor", DcMotor.Direction.FORWARD);
        fr = initializeMotor("FRMotor", DcMotor.Direction.FORWARD);
        // We will use these objects to move the actual motors on the robot.

        // Notify the user that the hardware setup is complete:
        telemetry.addLine("Hardware setup complete");

        // Clear all the data from the telemetry and replace with the new data:
        telemetry.update();
    }

    // Create an initializeMotor(String, DcMotor.Direction) method to help initialize motors faster.
    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        // Get a motor from the hardwareMap according to the name:
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);

        // Reset the encoder on the motor, but also still allow it to run:
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Make the motor brake (i.e. stop rigidly) the power is set to zero:
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the motor direction to be either DcMotor.Direction.FORWARD
        //     or DcMotor.Direction.REVERSE:
        motor.setDirection(direction);
        // This depends on how your hardware is set up.

        // Return the motor to the runOpMode() method:
        return motor;
    }
}