package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name = "Distance Tester")

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
