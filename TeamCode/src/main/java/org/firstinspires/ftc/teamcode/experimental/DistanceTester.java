package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Distance Tester", group="Experimental")
public class DistanceTester extends OpMode {
    protected ModernRoboticsI2cRangeSensor frontDist, leftFrontDist, leftRearDist;

    @Override
    public void init() {
        frontDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("frontDist"));
        frontDist.setI2cAddress(new I2cAddr(0x12));
        leftFrontDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("leftFrontDist"));
        leftFrontDist.setI2cAddress(new I2cAddr(0x14));
        leftRearDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("leftRearDist"));
        leftRearDist.setI2cAddress(new I2cAddr(0x16));
    }

    @Override
    public void loop() {
        for(I2cDeviceSynch dev : hardwareMap.i2cDeviceSynch) {
            telemetry.addData("con info device: " + dev.getDeviceName(), dev.getConnectionInfo());
            telemetry.addData("address device " + dev.getDeviceName(), Integer.toHexString(frontDist.getI2cAddress().get8Bit()));
            telemetry.addData("reading device " + dev.getDeviceName(), new ModernRoboticsI2cRangeSensor(dev).getDistance(DistanceUnit.CM));
        }

        telemetry.addData("Front",frontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("LeftFront",leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("LeftRear",leftRearDist.getDistance(DistanceUnit.CM));
    }
}
