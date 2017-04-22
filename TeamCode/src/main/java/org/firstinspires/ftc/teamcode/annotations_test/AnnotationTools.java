package org.firstinspires.ftc.teamcode.annotations_test;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Created by hdamron1594 on 4/22/17.
 */

public class AnnotationTools {
    public static Set<HardwareDevice> hardwareMapSet(final HardwareMap hardwareMap) {
        Set<HardwareDevice> completeSet = new HashSet<>();
        addDeviceMapping(completeSet, hardwareMap.accelerationSensor);
        addDeviceMapping(completeSet, hardwareMap.analogInput);
        addDeviceMapping(completeSet, hardwareMap.analogOutput);
        addDeviceMapping(completeSet, hardwareMap.compassSensor);
        addDeviceMapping(completeSet, hardwareMap.dcMotor);
        addDeviceMapping(completeSet, hardwareMap.dcMotorController);
        addDeviceMapping(completeSet, hardwareMap.deviceInterfaceModule);
        addDeviceMapping(completeSet, hardwareMap.digitalChannel);
        addDeviceMapping(completeSet, hardwareMap.gyroSensor);
        addDeviceMapping(completeSet, hardwareMap.i2cDevice);
        addDeviceMapping(completeSet, hardwareMap.irSeekerSensor);
        addDeviceMapping(completeSet, hardwareMap.legacyModule);
        addDeviceMapping(completeSet, hardwareMap.lightSensor);
        addDeviceMapping(completeSet, hardwareMap.opticalDistanceSensor);
        addDeviceMapping(completeSet, hardwareMap.pwmOutput);
        addDeviceMapping(completeSet, hardwareMap.servo);
        addDeviceMapping(completeSet, hardwareMap.servoController);
        addDeviceMapping(completeSet, hardwareMap.touchSensor);
        addDeviceMapping(completeSet, hardwareMap.touchSensorMultiplexer);
        addDeviceMapping(completeSet, hardwareMap.ultrasonicSensor);
        addDeviceMapping(completeSet, hardwareMap.voltageSensor);
        addDeviceMapping(completeSet, hardwareMap.colorSensor);
        addDeviceMapping(completeSet, hardwareMap.led);
        //Add more device types here if more become supported
        return completeSet;
    }

    /**
     * Modifies map in place to contain the hardware device mappings
     * @param set Set to be modified to contain the HardwareDevice values of devices
     * @param devices DeviceMapping to be applied to map
     */
    private static void addDeviceMapping(Set<HardwareDevice> set, HardwareMap.DeviceMapping<? extends HardwareDevice> devices) {
        for(Map.Entry<String, ? extends HardwareDevice> dev : devices.entrySet()) {
            set.add(dev.getValue());
        }
    }
}
