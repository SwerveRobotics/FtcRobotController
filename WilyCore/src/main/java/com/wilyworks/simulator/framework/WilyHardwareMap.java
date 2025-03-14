package com.wilyworks.simulator.framework;


import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.swerverobotics.ftc.GoBildaPinpointDriver;
import org.swerverobotics.ftc.UltrasonicDistanceSensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Consumer;

import kotlin.coroutines.Continuation;

/**
 * Wily Works simulated IMU implementation.
 */
class WilyIMU extends WilyHardwareDevice implements IMU {
    double startYaw;
    @Override
    public boolean initialize(Parameters parameters) {
        resetYaw();
        return true;
    }

    @Override
    public void resetYaw() {
        startYaw = WilyWorks.getPose().heading.log();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(
                AngleUnit.RADIANS,
                WilyWorks.getPose().heading.log() - startYaw,
                0,
                0,
                0);
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return new Orientation();
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return new Quaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return new AngularVelocity(
                angleUnit,
                (float) WilyWorks.getPoseVelocity().angVel, // ### transformedAngularVelocityVector.get(0),
                0, // ### transformedAngularVelocityVector.get(1),
                0, // ### transformedAngularVelocityVector.get(2),
                0); // ### rawAngularVelocity.acquisitionTime);
    }
}

/**
 * Wily Works voltage sensor implementation.
 */
class WilyVoltageSensor extends WilyHardwareDevice implements VoltageSensor {
    @Override
    public double getVoltage() {
        return 13.0;
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor";
    }
}

/**
 * Wily Works distance sensor implementation.
 */
class WilyDistanceSensor extends WilyHardwareDevice implements DistanceSensor {
    @Override
    public double getDistance(DistanceUnit unit) { return unit.fromMm(65535); } // Distance when not responding
}

/**
 * Wily Works normalized color sensor implementation.
 */
class WilyNormalizedColorSensor extends WilyHardwareDevice implements NormalizedColorSensor {
    @Override
    public NormalizedRGBA getNormalizedColors() { return new NormalizedRGBA(); }

    @Override
    public float getGain() { return 0; }

    @Override
    public void setGain(float newGain) { }
}

/**
 * Wily Works color sensor implementation.
 */
class WilyColorSensor extends WilyHardwareDevice implements ColorSensor {

    @Override
    public int red() { return 0; }

    @Override
    public int green() { return 0; }

    @Override
    public int blue() { return 0; }

    @Override
    public int alpha() { return 0; }

    @Override
    public int argb() { return 0; }

    @Override
    public void enableLed(boolean enable) { }

    @Override
    public void setI2cAddress(I2cAddr newAddress) { }

    @Override
    public I2cAddr getI2cAddress() { return null; }
}

/**
 * Wily Works named webcam implementation.
 */
class WilyWebcam extends WilyHardwareDevice implements WebcamName {
    WilyWorks.Config.Camera wilyCamera;

    WilyWebcam(String deviceName) {
        for (WilyWorks.Config.Camera camera: WilyCore.config.cameras) {
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

/**
 * Wily Works ServoController implementation.
 */
class WilyServoController extends WilyHardwareDevice implements ServoController {
    @Override
    public void pwmEnable() { }

    @Override
    public void pwmDisable() { }

    @Override
    public PwmStatus getPwmStatus() { return PwmStatus.DISABLED; }

    @Override
    public void setServoPosition(int servo, double position) { }

    @Override
    public double getServoPosition(int servo) { return 0; }

    @Override
    public void close() { }
}

/**
 * Wily Works Servo implementation.
 */
class WilyServo extends WilyHardwareDevice implements Servo {
    double position;

    @Override
    public ServoController getController() {
        return new WilyServoController();
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPosition(double position) { this.position = Math.max(0, Math.min(1, position)); }

    @Override
    public double getPosition() {return position; }

    @Override
    public void scaleRange(double min, double max) {

    }
}

/**
 * Wily Works CRServo implementation.
 */
class WilyCRServo extends WilyHardwareDevice implements CRServo {

    @Override
    public ServoController getController() {
        return new WilyServoController();
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }
}

/**
 * Wily Works DigitalChannel implementation.
 */
class WilyDigitalChannel extends WilyHardwareDevice implements DigitalChannel {
    boolean state;

    @Override
    public Mode getMode() { return null; }

    @Override
    public void setMode(Mode mode) {}

    @Override
    public boolean getState() { return state; }

    @Override
    public void setState(boolean state) { this.state = state; }
}

/**
 * Wily Works LED implementation.
 */
class WilyLED extends LED {
    // Assume that every digital channels is a REV LED indicator. Doesn't hurt if that's not
    // the case:
    boolean enable = true; // They're always on by default
    double x;
    double y;
    boolean isRed;
    WilyLED(String deviceName) {
        WilyWorks.Config.LEDIndicator wilyLed = null;
        for (WilyWorks.Config.LEDIndicator led: WilyCore.config.ledIndicators) {
            if (led.name.equals(deviceName)) {
                wilyLed = led;
            }
        }
        if (wilyLed != null) {
            x = wilyLed.x;
            y = wilyLed.y;
            isRed = wilyLed.isRed;
        } else {
            isRed = !(deviceName.toLowerCase().contains("green"));
        }
    }

    @Override
    public void enableLight(boolean enable) {
        this.enable = enable;
    }

    @Override
    public boolean isLightOn() {
        return enable;
    }
}

/**
 * Wily Works hardware map.
 */
public class WilyHardwareMap implements Iterable<HardwareDevice> {

    public DeviceMapping<VoltageSensor>            voltageSensor            = new DeviceMapping<>(VoltageSensor.class);
    public DeviceMapping<DcMotor>                  dcMotor                  = new DeviceMapping<>(DcMotor.class);
    public DeviceMapping<DistanceSensor>           distanceSensor           = new DeviceMapping<>(DistanceSensor.class);
    public DeviceMapping<NormalizedColorSensor>    normalizedColorSensor    = new DeviceMapping<>(NormalizedColorSensor.class);
    public DeviceMapping<ColorSensor>              colorSensor              = new DeviceMapping<>(ColorSensor.class);
    public DeviceMapping<WebcamName>               webcamName               = new DeviceMapping<>(WebcamName.class);
    public DeviceMapping<Servo>                    servo                    = new DeviceMapping<>(Servo.class);
    public DeviceMapping<CRServo>                  crservo                  = new DeviceMapping<>(CRServo.class);
    public DeviceMapping<DigitalChannel>           digitalChannel           = new DeviceMapping<>(DigitalChannel.class);
    public DeviceMapping<LED>                      led                      = new DeviceMapping<>(LED.class);
    public DeviceMapping<SparkFunOTOS>             sparkFunOTOS             = new DeviceMapping<>(SparkFunOTOS.class);
    public DeviceMapping<GoBildaPinpointDriver>    goBildaPinpointDrivers   = new DeviceMapping<>(GoBildaPinpointDriver.class);
    public DeviceMapping<UltrasonicDistanceSensor> ultrasonicDistanceSensor = new DeviceMapping<>(UltrasonicDistanceSensor.class);
    protected Map<String, List<HardwareDevice>>    allDevicesMap            = new HashMap<>();
    protected List<HardwareDevice>                 allDevicesList           = new ArrayList<>();

    public WilyHardwareMap() {
        put("voltage_sensor", VoltageSensor.class);
    }

    public final Context appContext = new Context();
    protected final Object lock = new Object();

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new LinkedList<T>();
        return result;
    }

    private synchronized <T> T wilyTryGet(Class<? extends T> classOrInterface, String deviceName){
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list != null) {
            for (HardwareDevice device : list){
                if(classOrInterface.isInstance(device)) return classOrInterface.cast(device);
            }
        }
        return null;
    }

    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        return get(classOrInterface, deviceName);
    }
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        if (classOrInterface.equals(IMU.class)) {
            return (T) new WilyIMU();
        }
        T result = wilyTryGet(classOrInterface, deviceName);
        if (result == null) {
            // Wily Works behavior is that we automatically add the device if it's not found:
            put(deviceName, classOrInterface);
            result = wilyTryGet(classOrInterface, deviceName);
        }
        return result;
    }

    @Deprecated
    public HardwareDevice get(String deviceName) {
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list != null) {
            return list.get(0);
        }

        throw new IllegalArgumentException("Use the typed version of get(), e.g. get(DcMotorEx.class, \"leftMotor\")");
    }

    // Wily Works way to add devices to the hardware map:
    public synchronized void put(String deviceName, Class klass) {
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list == null) {
            list = new ArrayList<>();
            allDevicesMap.put(deviceName, list);
        }
        HardwareDevice device;
        if (CRServo.class.isAssignableFrom(klass)) {
            device = new WilyCRServo();
            crservo.put(deviceName, (CRServo) device);
        } else if (Servo.class.isAssignableFrom(klass)) {
            device = new WilyServo();
            servo.put(deviceName, (Servo) device);
        } else if (DcMotor.class.isAssignableFrom(klass)) {
            device = new WilyDcMotorEx();
            dcMotor.put(deviceName, (DcMotor) device);
        } else if (VoltageSensor.class.isAssignableFrom(klass)) {
            device = new WilyVoltageSensor();
            voltageSensor.put(deviceName, (VoltageSensor) device);
        } else if (DistanceSensor.class.isAssignableFrom(klass)) {
            device = new WilyDistanceSensor();
            distanceSensor.put(deviceName, (DistanceSensor) device);
        } else if (NormalizedColorSensor.class.isAssignableFrom(klass)) {
            device = new WilyNormalizedColorSensor();
            normalizedColorSensor.put(deviceName, (NormalizedColorSensor) device);
        } else if (ColorSensor.class.isAssignableFrom(klass)) {
            device = new WilyColorSensor();
            colorSensor.put(deviceName, (ColorSensor) device);
        } else if (WebcamName.class.isAssignableFrom(klass)) {
            device = new WilyWebcam(deviceName);
            webcamName.put(deviceName, (WebcamName) device);
        } else if (DigitalChannel.class.isAssignableFrom(klass)) {
            device = new WilyDigitalChannel();
            digitalChannel.put(deviceName, (DigitalChannel) device);
        } else if (LED.class.isAssignableFrom(klass)) {
            device = new WilyLED(deviceName);
            led.put(deviceName, (LED) device);
        } else if (SparkFunOTOS.class.isAssignableFrom(klass)) {
            device = new SparkFunOTOS(null);
            sparkFunOTOS.put(deviceName, (SparkFunOTOS) device);
        } else if (GoBildaPinpointDriver.class.isAssignableFrom(klass)) {
            device = new GoBildaPinpointDriver(null, false);
            goBildaPinpointDrivers.put(deviceName, (GoBildaPinpointDriver) device);
        } else if (UltrasonicDistanceSensor.class.isAssignableFrom(klass)) {
            device = new WilyUltrasonicDistanceSensor(deviceName);
            ultrasonicDistanceSensor.put(deviceName, (WilyUltrasonicDistanceSensor) device);
        } else {
            throw new IllegalArgumentException("Unexpected device type for HardwareMap");
        }
        list.add(device);
        allDevicesList.add(device);
    }

    private void initializeDeviceIfNecessary(HardwareDevice device) {
    }

    private void initializeMultipleDevicesIfNecessary(Iterable<? extends HardwareDevice> devices) {
        for (HardwareDevice device: devices) {
            initializeDeviceIfNecessary(device);
        }
    }

    public SortedSet<String> getAllNames(Class<? extends HardwareDevice> classOrInterface) {
        SortedSet<String> result = new TreeSet<>();
        result.add("voltage_sensor");
        return result;
    }

    @Override
    public @NonNull Iterator<HardwareDevice> iterator() {
        return new ArrayList<HardwareDevice>(allDevicesList).iterator();
    }

    // A DeviceMapping contains a sub-collection of the devices registered in a HardwareMap
    // comprised of all devices of a particular device type.
    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE> {
        private final Map<String, DEVICE_TYPE> map = new HashMap<String, DEVICE_TYPE>();
        private final Class<DEVICE_TYPE> deviceTypeClass;

        public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        public Class<DEVICE_TYPE> getDeviceTypeClass() {
            return this.deviceTypeClass;
        }

        public DEVICE_TYPE cast(Object obj) {
            return this.deviceTypeClass.cast(obj);
        }

        public DEVICE_TYPE get(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                DEVICE_TYPE device = map.get(deviceName);
                if (device == null) {
                    String msg = String.format("Unable to find a hardware device with the name \"%s\"", deviceName);
                    throw new IllegalArgumentException(msg);
                }
                initializeDeviceIfNecessary(device);
                return device;
            }
        }

        public void put(String deviceName, DEVICE_TYPE device) {
            map.put(deviceName.trim(), device);
        }

        public void putLocal(String deviceName, DEVICE_TYPE device) {
            synchronized (lock) {
            }
        }

        public boolean contains(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                return map.containsKey(deviceName);
            }
        }

        public boolean remove(String deviceName) {
            return remove(null, deviceName);
        }

        public boolean remove(@Nullable SerialNumber serialNumber, String deviceName) {
            synchronized (lock) {
                return false;
            }
        }

        @Override public @NonNull Iterator<DEVICE_TYPE> iterator() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new ArrayList<>(map.values()).iterator();
            }
        }

        public Set<Map.Entry<String, DEVICE_TYPE>> entrySet() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new HashSet<>(map.entrySet());
            }
        }

        public int size() {
            synchronized (lock) {
                return map.size();
            }
        }
    }
}
