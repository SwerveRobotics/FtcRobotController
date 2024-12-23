package org.firstinspires.ftc.team417.motorcache;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorCache implements DcMotorEx {
    protected final DcMotorEx motor;

    public DcMotorCache(DcMotorEx motor) {
        this.motor = motor;
    }

    private static final double EPSILON = 0.0001;

    private boolean isNotEpsilonEqual(double a, double b) {
        return Math.abs(a - b) > EPSILON;
    }

    // Caching for setPower
    private volatile double setPowerPowerCache = 0;
    @Override
    public void setPower(double power) {
        if (isNotEpsilonEqual(setPowerPowerCache, power)) {
            motor.setPower(power);
            setPowerPowerCache = power;
        }
    }

    // Caching for setVelocity
    private volatile double setVelocityCache = 0;
    @Override
    public void setVelocity(double angularRate) {
        if (isNotEpsilonEqual(setVelocityCache, angularRate)) {
            motor.setVelocity(angularRate);
            setVelocityCache = angularRate;
        }
    }

    // Caching for setVelocity with unit
    private volatile double setVelocityWithUnitCache = 0;
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (isNotEpsilonEqual(setVelocityWithUnitCache, angularRate)) {
            motor.setVelocity(angularRate, unit);
            setVelocityWithUnitCache = angularRate;
        }
    }

    // Caching for setMode
    private volatile RunMode setModeCache = RunMode.RUN_WITHOUT_ENCODER;
    @Override
    public void setMode(RunMode mode) {
        if (mode != setModeCache) {
            motor.setMode(mode);
            setModeCache = mode;
        }
    }

    // Caching for setZeroPowerBehavior
    private volatile ZeroPowerBehavior setZeroPowerBehaviorCache = ZeroPowerBehavior.FLOAT;
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior != setZeroPowerBehaviorCache) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            setZeroPowerBehaviorCache = zeroPowerBehavior;
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    // Caching for setTargetPositionTolerance
    private volatile int setTargetPositionToleranceCache = 0;
    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (tolerance != setTargetPositionToleranceCache) {
            motor.setTargetPositionTolerance(tolerance);
            setTargetPositionToleranceCache = tolerance;
        }
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    // Caching for setDirection
    private volatile Direction setDirectionCache = Direction.FORWARD;
    @Override
    public void setDirection(Direction direction) {
        if (direction != setDirectionCache) {
            motor.setDirection(direction);
            setDirectionCache = direction;
        }
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    // Caching for setMotorType
    private volatile MotorConfigurationType setMotorTypeCache = null;
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        if (!motorType.equals(setMotorTypeCache)) {
            motor.setMotorType(motorType);
            setMotorTypeCache = motorType;
        }
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    // Caching for setTargetPosition
    private volatile int setTargetPositionCache = 0;
    @Override
    public void setTargetPosition(int position) {
        if (position != setTargetPositionCache) {
            motor.setTargetPosition(position);
            setTargetPositionCache = position;
        }
    }

    // Caching for setCurrentAlert
    private volatile double setCurrentAlertCache = 0;
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        if (current != setCurrentAlertCache) {
            motor.setCurrentAlert(current, unit);
            setCurrentAlertCache = current;
        }
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    // Caching for setMotorEnable
    private volatile boolean setMotorEnableCache = false;
    @Override
    public void setMotorEnable() {
        if (!setMotorEnableCache) {
            motor.setMotorEnable();
            setMotorEnableCache = true;
        }
    }

    // Caching for setMotorDisable
    private volatile boolean setMotorDisableCache = false;
    @Override
    public void setMotorDisable() {
        if (!setMotorDisableCache) {
            motor.setMotorDisable();
            setMotorDisableCache = true;
        }
    }

    // Caching for setPowerFloat
    private volatile boolean setPowerFloatCache = false;
    @Override
    public void setPowerFloat() {
        if (!setPowerFloatCache) {
            motor.setPowerFloat();
            setPowerFloatCache = true;
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    // Caching for setPIDCoefficients
    private volatile RunMode setPIDCoefficientsModeCache = RunMode.RUN_WITHOUT_ENCODER;
    private volatile PIDCoefficients setPIDCoefficientsCache = null;
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        if (mode != setPIDCoefficientsModeCache || isNotEpsilonEqual(pidCoefficients.p, setPIDCoefficientsCache.p) || isNotEpsilonEqual(pidCoefficients.i, setPIDCoefficientsCache.i) || isNotEpsilonEqual(pidCoefficients.d, setPIDCoefficientsCache.d)) {
            motor.setPIDCoefficients(mode, pidCoefficients);
            setPIDCoefficientsModeCache = mode;
            setPIDCoefficientsCache = pidCoefficients;
        }
    }

    // Caching for setPIDFCoefficients
    private volatile RunMode setPIDFCoefficientsModeCache = RunMode.RUN_WITHOUT_ENCODER;
    private volatile PIDFCoefficients setPIDFCoefficientsCache = null;
    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        if (mode != setPIDFCoefficientsModeCache || isNotEpsilonEqual(pidfCoefficients.p, setPIDFCoefficientsCache.p) || isNotEpsilonEqual(pidfCoefficients.i, setPIDFCoefficientsCache.i) || isNotEpsilonEqual(pidfCoefficients.d, setPIDFCoefficientsCache.d) || isNotEpsilonEqual(pidfCoefficients.f, setPIDFCoefficientsCache.f)) {
            motor.setPIDFCoefficients(mode, pidfCoefficients);
            setPIDFCoefficientsModeCache = mode;
            setPIDFCoefficientsCache = pidfCoefficients;
        }
    }

    // Caching for setVelocityPIDFCoefficients
    private volatile double setVelocityPIDFCoefficientsPCache = 0;
    private volatile double setVelocityPIDFCoefficientsICache = 0;
    private volatile double setVelocityPIDFCoefficientsDCache = 0;
    private volatile double setVelocityPIDFCoefficientsFCache = 0;
    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (isNotEpsilonEqual(p, setVelocityPIDFCoefficientsPCache) || isNotEpsilonEqual(i, setVelocityPIDFCoefficientsICache) || isNotEpsilonEqual(d, setVelocityPIDFCoefficientsDCache) || isNotEpsilonEqual(f, setVelocityPIDFCoefficientsFCache)) {
            motor.setVelocityPIDFCoefficients(p, i, d, f);
            setVelocityPIDFCoefficientsPCache = p;
            setVelocityPIDFCoefficientsICache = i;
            setVelocityPIDFCoefficientsDCache = d;
            setVelocityPIDFCoefficientsFCache = f;
        }
    }

    // Caching for setPositionPIDFCoefficients
    private volatile double setPositionPIDFCoefficientsPCache = 0;
    @Override
    public void setPositionPIDFCoefficients(double p) {
        if (isNotEpsilonEqual(p, setPositionPIDFCoefficientsPCache)) {
            motor.setPositionPIDFCoefficients(p);
            setPositionPIDFCoefficientsPCache = p;
        }
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
