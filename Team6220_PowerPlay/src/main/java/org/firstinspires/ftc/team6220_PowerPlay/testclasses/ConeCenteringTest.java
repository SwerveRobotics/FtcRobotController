package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ConeCenteringTest")
public class ConeCenteringTest extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize();

        //driveSlidesAutonomous(Constants.SLIDE_TOP);

        RobotCameraPipeline robotCameraPipeline = new RobotCameraPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        robotCameraPipeline.setTargetColor(RobotCameraPipeline.Color.ALL);

        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        waitForStart();

        //centerConeStack(robotCameraPipeline, Constants.CONE_WIDTH);

        robotCamera.closeCameraDevice();
    }
}
