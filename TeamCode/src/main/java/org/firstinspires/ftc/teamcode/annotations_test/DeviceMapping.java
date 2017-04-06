package org.firstinspires.ftc.teamcode.annotations_test;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by roboticsteam on 4/5/2017.
 */

public @interface DeviceMapping {
    String devName();
    Class<? extends HardwareDevice> type();
}