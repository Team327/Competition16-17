package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by gssmrobotics on 12/9/2016.
 */

@TeleOp(name = "Final TeleOp")
public class FinalTeleOp extends OpMode
{

    DcMotor right,left,intake,shooter;
    Servo beacon;

    public void init()
    {
        right=hardwareMap.dcMotor.get("right");
        left=hardwareMap.dcMotor.get("left");
        shooter=hardwareMap.dcMotor.get("shooter");

        beacon = hardwareMap.servo.get("beacon");

        beacon.setPosition(.3);
    }

    public void loop()
    {
        /*
         * Driver 1 Stuffs
         *
         * Sticks to drive - Tank drive
        */

        //Driving
        right.setPower(gamepad1.right_stick_y);
        left.setPower(-gamepad1.left_stick_y);




        /*
         *
         * Driver 2 stuff
         *
         * Right Trigger shoot
         * Left Trigger reverse
         *
        */

        if(!(gamepad2.right_bumper ^ gamepad2.left_bumper))//cheeky xor
            beacon.setPosition(.3);
        else if(gamepad2.right_bumper)//right hit, set to hit right button
            beacon.setPosition(.1);
        else//left pressed, set to hit left button
            beacon.setPosition(.5);

        shooter.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }



}
