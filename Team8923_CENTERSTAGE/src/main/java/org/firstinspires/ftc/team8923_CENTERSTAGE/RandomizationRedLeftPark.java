package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationRedLeftPark")

public class RandomizationRedLeftPark extends BaseAutonomous {
    public void runOpMode() {
        initRedAuto();
        waitForStart();
        driveInches(0, 5);
        switch (myRedColorDetection.detectColor()) {
            case ONE:
                driveInches(0, 23);
                pivot(-90);
                driveInches(0, -4);
                runIntake(0.55, 1500);
                /*driveInches(0, 4);
                pivot(-180);
                driveInches(0, 26);
                pivot(-90);
                driveInches(0, 96);*/
                break;
            case TWO:
                driveInches(0, 32);
                driveInches(0, -6);
                runIntake(0.6, 1500);
                /*driveInches(0, -19.5);
                pivot(90);
                driveInches(0, 96);*/
                break;
            case THREE:
                driveInches(0, 28);
                pivot(90);
                driveInches(0, -4);
                runIntake(0.5, 1500);
                /*driveInches(0, 4);
                pivot(0);
                driveInches(0, -26);
                pivot(90);
                driveInches(0, 96);*/
                break;
        }
    }
}