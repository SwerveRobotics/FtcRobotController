package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleOp", group="DAB")
public class DAB_drive extends BaseOpMode {


    //drive
    boolean isJoystickLeft = false;
    boolean isJoystickRight = false;


    //attachment
    boolean isLBumperPressed = false;
    boolean isRBumperPressed = false;
    boolean isLTriggerPressed = false;
    boolean isRTriggerPressed = false;




    public boolean isDpadPressed(){
        return gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;


    }
    public double[] drive(){
        double left = 0;
        double right = 0;

        if(gamepad1.left_stick_x == 0 || gamepad1.left_stick_y == 0){
            right = gamepad1.left_stick_x * -0.5 + 0.5 * 2;
            left =  gamepad1.left_stick_x * 0.5 + 0.5 * 2;
        } else if ((gamepad1.right_stick_x == 0 || gamepad1.right_stick_y == 0)) {

        } else if (isDpadPressed()) {

        } else {

        }

    }
    @Override
    public void runOpMode(){
        waitForStart();






    }
}
