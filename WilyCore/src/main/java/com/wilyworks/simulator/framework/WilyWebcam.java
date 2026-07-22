package com.wilyworks.simulator.framework;

import android.content.Context;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.SerialNumber;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.function.Consumer;

import kotlin.coroutines.Continuation;

/**
 * Wily Works named webcam implementation.
 */
public class WilyWebcam extends WilyHardwareDevice implements WebcamName {
    WilyWorks.Config.Camera wilyCamera;

    WilyWebcam(String deviceName) {
        for (WilyWorks.Config.Camera camera : WilyCore.config.cameras) {
            if (camera.name.equals(deviceName)) {
                wilyCamera = camera;
            }
        }
        if (wilyCamera == null) {
            System.out.printf("WilyWorks: Couldn't find configuration data for camera '%s'", deviceName);
        }
    }

    @Override
    public boolean isWebcam() {
        return true;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation) {

    }

    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return false;
    }

    @Override
    public CameraCharacteristics getCameraCharacteristics() {
        return null;
    }

    @Override
    public SerialNumber getSerialNumber() {
        return null;
    }

    @Nullable
    @Override
    public String getUsbDeviceNameIfAttached() {
        return null;
    }

    @Override
    public boolean isAttached() {
        return false;
    }
}
