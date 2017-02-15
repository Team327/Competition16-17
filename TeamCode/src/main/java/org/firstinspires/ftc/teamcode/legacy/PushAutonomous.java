package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by gssmrobotics on 12/10/2016.
 */

@Disabled
@Autonomous(name = "Push Autonomous")

public class PushAutonomous extends OpMode{

    DcMotor left,right;
    long startTime=0;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
    }
    public void loop()
    {
        if(startTime==0)
            startTime=System.currentTimeMillis();
        else if(System.currentTimeMillis()-startTime>3000)
        {
            stop();
        }
        else
        {
            right.setPower(1);
            left.setPower(-1);
        }
    }
}
