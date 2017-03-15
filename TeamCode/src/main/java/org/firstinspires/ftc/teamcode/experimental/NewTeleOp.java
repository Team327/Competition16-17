package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by gssmrobotics on 3/15/2017.
 */
@TeleOp(name= "New TeleOp")
public class NewTeleOp extends VisionOpMode {
    Robot robot;


    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

    }


    @Override
    public void loop()
    {
        if(gamepad1.a)
        {
            robot.pushBeacon();
        }
        else
        {
            robot.retractBeacon();
        }
    }

}
