package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Distance Tester")

@Disabled
public class DistanceSensorTester extends OpMode {

    private OpticalDistanceSensor dist1 = null;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        
        dist1 = hardwareMap.opticalDistanceSensor.get("dist1");
        
        dist1.enableLed(true);
    }


    @Override
    public void loop()
    {
        telemetry.addData("Status", dist1.getLightDetected());
    }
}
