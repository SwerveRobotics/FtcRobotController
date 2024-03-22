package org.firstinspires.ftc.team417_CENTERSTAGE.apriltagsnew;

/*
    Stores all the info about camera, including offset, resolution, etc.
    Used in AprilTagPoseEstimator
*/

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.CameraInfo;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.LensIntrinsics;

public class CameraInfoDump {
    //Stores the info about the cameras in an array
    public static CameraInfo[] cameraInfoArray = new CameraInfo[] {
            new CameraInfo("CompetitionRobotSlowCamera", "webcam", new Vector2d(3, 7), 0, new Size(1920, 1080), null, 640, Math.toRadians(70.4)),
            new CameraInfo("DevBotSlowCamera", "webcamfront", new Vector2d(0, 7.8125), 0, new Size(1920, 1080), null, 640, Math.toRadians(70.4)),
            new CameraInfo("DevBotFastCamera", "webcamback", new Vector2d(0, 0) /*TODO: fix this!!!*/, 0, new Size(1280, 800), new LensIntrinsics(906.940247073, 906.940247073, 670.833056673, 355.34234068), 190, Math.toRadians(70.4))
    };

    public static CameraInfo camera(String realName) {
        for (CameraInfo cameraInfo : cameraInfoArray) {
            if (cameraInfo.realName.equals(realName)) {
                return cameraInfo;
            }
        }
        throw new Error("Camera not found");
    }
}
