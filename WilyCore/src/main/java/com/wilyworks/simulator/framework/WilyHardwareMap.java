package com.wilyworks.simulator.framework;


import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    public DeviceMapping<AnalogInput>              analogInput              = new DeviceMapping<>(AnalogInput.class);
    public DeviceMapping<IMU>                      imu                      = new DeviceMapping<>(IMU.class);
    public DeviceMapping<Limelight3A>              limelight                = new DeviceMapping<>(Limelight3A.class);
    public DeviceMapping<SparkFunLEDStick>         sparkFunLEDStick         = new DeviceMapping<>(SparkFunLEDStick.class);
    protected Map<String, List<HardwareDevice>>    allDevicesMap            = new HashMap<>();
    protected List<HardwareDevice>                 allDevicesList           = new ArrayList<>();

    public final List<DeviceMapping<? extends HardwareDevice>> allDeviceMappings = new ArrayList<>();

    public WilyHardwareMap(Context appContext, OpModeManagerNotifier notifier) {
        // Road Runner expects this object to be already created because it references
        // hardwareMap.voltageSensor.iterator().next() directly:
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
        T result = wilyTryGet(classOrInterface, deviceName);
        if (result == null) {
            // Wily Works behavior is that we automatically add the device if it's not found:
            put(deviceName, classOrInterface);
            result = wilyTryGet(classOrInterface, deviceName);
        }
        return result;
    }

    public HardwareDevice get(String deviceName) {
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list != null) {
            for (HardwareDevice device : list) {
                initializeDeviceIfNecessary(device);
                return device;
            }
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
        } else if (AnalogInput.class.isAssignableFrom(klass)) {
            device = new WilyAnalogInput();
            analogInput.put(deviceName, (WilyAnalogInput) device);
        } else if (IMU.class.isAssignableFrom(klass)) {
            device = new WilyIMU();
            imu.put(deviceName, (WilyIMU) device);

        // Not actually built into HardwareMap.java:
        } else if (UltrasonicDistanceSensor.class.isAssignableFrom(klass)) {
            device = new WilyUltrasonicDistanceSensor(deviceName);
            ultrasonicDistanceSensor.put(deviceName, (WilyUltrasonicDistanceSensor) device);
        } else if (SparkFunOTOS.class.isAssignableFrom(klass)) {
            device = new SparkFunOTOS(null);
            sparkFunOTOS.put(deviceName, (SparkFunOTOS) device);
        } else if (GoBildaPinpointDriver.class.isAssignableFrom(klass)) {
            device = new GoBildaPinpointDriver(null, false);
            goBildaPinpointDrivers.put(deviceName, (GoBildaPinpointDriver) device);
        } else if (Limelight3A.class.isAssignableFrom(klass)) {
            device = new Limelight3A(null, "", null);
            limelight.put(deviceName, (Limelight3A) device);
        } else if (SparkFunLEDStick.class.isAssignableFrom(klass)) {
            device = new SparkFunLEDStick(null, false);
            sparkFunLEDStick.put(deviceName, (SparkFunLEDStick) device);
        } else {
            throw new IllegalArgumentException("Unexpected device type for HardwareMap");
        }

        // Let the game simulation change the behavior of this device:
        device = WilyCore.mechSim.hookDevice(deviceName, device);

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
        for (String userName: allDevicesMap.keySet()) {
            HardwareDevice device = allDevicesMap.get(userName).get(0);
            if (classOrInterface.isInstance(device)) {
                result.add(userName);
            }
        }
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
