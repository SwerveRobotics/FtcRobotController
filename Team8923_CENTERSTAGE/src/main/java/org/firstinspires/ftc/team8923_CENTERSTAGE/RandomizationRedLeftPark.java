package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationRedLeftPark")

public class RandomizationRedLeftPark extends BaseAutonomous {
    public void runOpMode() {
        if(gamepad1.x){
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
        }

        if(gamepad1.b){
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
        }
        // start and stop pipeline
        if(gamepad1.y){
            myColorDetection.startStreaming();
        }

        if(gamepad1.a){
            myColorDetection.stopStreaming();
        }
        initAuto();

        while (opModeIsActive()) {
            telemetry.addData("color", myColorDetection.myColor);
            telemetry.update();

        }
        waitForStart();
        /*telemetry.addData("color", myColorDetection.myColor);
        telemetry.update();
        waitForStart();*/

       // myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
        switch (myColorDetection.detectColor()) {
            case ONE:
                driveInches(0, 24);
                pivot(-90);
                // pivot -90 degrees
                runIntake(0.5, 2);
                // output pixel
                pivot(-90);
                //pivot -90 degrees
                driveInches(0, 24);
                // pivot -90 degrees
                driveInches(0, 96);
                break;
            case TWO:
                driveInches(0, 24);
                runIntake(0.5, 2);
                // output pixel
                driveInches(0, -24);
                // pivot 90 degrees
                pivot(90);
                driveInches(0, 96);
                break;
            case THREE:
                driveInches(0, 24);
                // pivot 90 degrees
                pivot(90);
                // output pixel
                runIntake(0.5, 2);
                //pivot 90 degrees
                pivot(90);
                driveInches(0, 24);
                pivot(-90);
                // pivot -90 degrees
                driveInches(0, 96);
                break;
            case FOUR:
                // pivot 90 degrees
                pivot(90);
                driveInches(0, 96);
                break;
        }
    }
}