package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by gssmrobotics on 3/24/2017.
 */

public class RangeSensor {
    protected I2cDevice sensor;
    protected I2cDeviceSynch reader;

    // Experimentally determined constants for converting optical measurements
    // to distance. See cmFromOptical() below.
    public double pParam = -1.02001;
    public double qParam = 0.00311326;
    public double rParam = -8.39366;
    public int    sParam = 10;

    public RangeSensor(I2cDevice sensorDevice, int address) {
        sensor = sensorDevice;
        reader = new I2cDeviceSynchImpl(this.sensor, I2cAddr.create8bit(address), false);
        reader.engage();
    }

    protected double cmFromOptical(int opticalReading)
    {
        if (opticalReading < sParam)
            return 0;
        else
            return pParam * Math.log(qParam * (rParam + opticalReading));
    }

    public double getDistance(DistanceUnit unit) {
        byte[] reading = reader.read(0x04, 2);
        double cmUltrasonic = reading[0] & 0xFF;
        int rawOptical = reading[1] & 0xFF;
        double cmOptical = cmFromOptical(rawOptical);
        double cm = cmOptical > 0 ? cmOptical : cmUltrasonic;
        return unit.fromUnit(DistanceUnit.CM, cm);
    }
}
