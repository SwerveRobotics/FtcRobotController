package org.firstinspires.ftc.team6220.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team6220.DRIFTConstants;

public class ArmElbowAction {
    private Servo elbowServo;

    public ArmElbowAction(HardwareMap hardwareMap) {
        elbowServo = hardwareMap.get(Servo.class, DRIFTConstants.ARM_ELBOW_SERVO_HARDWARE_IDENTIFIER);
    }

    public Action setTargetPosition(double targetPosition) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    elbowServo.setPosition(targetPosition);
                    initialized = true;
                    System.out.println("gmaing");
                }

                double currentPosition = elbowServo.getPosition();
                telemetryPacket.put("elbowServoPosition", currentPosition);
                // return false because we only need to set the position once
                return false;
            }
        };
    }
}
