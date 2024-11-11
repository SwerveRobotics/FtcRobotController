package org.firstinspires.ftc.team6220.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team6220.DRIFTConstants;

public class ArmElbow {
    private CRServo elbowServo;

    public ArmElbow(HardwareMap hardwareMap) {
        elbowServo = hardwareMap.get(CRServo.class, DRIFTConstants.ARM_ELBOW_SERVO_HARDWARE_IDENTIFIER);
    }

    public Action setPower(double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    elbowServo.setPower(power);
                    initialized = true;
                }

                double power = elbowServo.getPower();
                telemetryPacket.put("elbowServoPower", power);
                return power < 10_000.0;
            }
        };
    }
}
