package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationBlueRightPark")

public class RandomizationBlueRightPark extends BaseAutonomous {
        public void runOpMode() {
            initBlueAuto();
            waitForStart();
            switch (myBlueColorDetection.detectColor()) {
                case ONE:
                    driveInches(0, 24);
                    pivot(-90);
                    runIntake(0.5, 2000);
                    pivot(-180);
                    driveInches(0, 24);
                    pivot(-90);
                    driveInches(0, 96);
                    break;
                case TWO:
                    driveInches(0, 28);
                    driveInches(0, -3);
                    // output purple pixel
                    runIntake(0.5, 2000);
                    driveInches(0, -24);
                    pivot(-90);
                    driveInches(0, 96);
                    break;
                case THREE:
                    driveInches(0, 24);
                    pivot(-90);
                    runIntake(0.5, 2);
                    pivot(-90);
                    driveInches(0, -24);
                    pivot(-90);
                    driveInches(0, 96);
                    break;

            }


        }
}
