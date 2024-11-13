package org.firstinspires.ftc.team6220.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.team6220.DRIFTConstants;

public class ArmMovementTicksAction {
    private DcMotorEx armBaseMotor;

    public ArmMovementTicksAction(HardwareMap hardwareMap) {
        armBaseMotor = hardwareMap.get(DcMotorEx.class, DRIFTConstants.ARM_BASE_MOTOR_HARDWARE_IDENTIFIER);
    }

    public Action setTargetPositionTicks(int targetPositionTicks) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    armBaseMotor.setTargetPosition(targetPositionTicks);
                    initialized = true;
                }

                double currentPositionTicks = armBaseMotor.getCurrentPosition();
                telemetryPacket.put("armBaseMotorTicks", currentPositionTicks);
                // return false if wilyworks is doing stuff because yeah
                return WilyWorks.isSimulating ? false : currentPositionTicks < targetPositionTicks ;
            }
        };
    }
}
