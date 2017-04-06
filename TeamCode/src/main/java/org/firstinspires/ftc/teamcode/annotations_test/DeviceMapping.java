package org.firstinspires.ftc.teamcode.annotations_test;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Created by roboticsteam on 4/5/2017.
 */

@Inherited
@Documented
@Retention(RetentionPolicy.RUNTIME)
public @interface DeviceMapping {
    String devName();
    Class<? extends HardwareDevice> type();
}