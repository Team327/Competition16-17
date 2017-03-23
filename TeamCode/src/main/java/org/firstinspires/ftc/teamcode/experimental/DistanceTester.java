package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by gssmrobotics on 1/14/2017.
 */

@Autonomous(name = "Distance Tester")
public class DistanceTester extends OpMode {

    protected ModernRoboticsI2cRangeSensor frontDist, leftFrontDist, leftRearDist;

    @Override
    public void init() {

        frontDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("frontDist"));
        //leftFrontDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("leftFrontDist"));
        leftRearDist = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("leftRearDist"));
    }

    @Override
    public void loop() {
        telemetry.addData("Front",frontDist.getDistance(DistanceUnit.CM));
        //telemetry.addData("LeftFront",leftFrontDist.getDistance(DistanceUnit.CM));
        telemetry.addData("LeftRear",leftRearDist.getDistance(DistanceUnit.CM));
    }
}
