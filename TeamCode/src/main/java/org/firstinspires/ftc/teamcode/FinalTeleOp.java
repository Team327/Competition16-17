package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by gssmrobotics on 12/9/2016.
 */

@TeleOp(name = "Final TeleOp")
public class FinalTeleOp extends OpMode
{
    Robot robot;

    /**
     * Tank Drive System Description
     *
     * Gamepad 1 (Driver):
     *      Left Joystick y: Left Drive
     *      Right Joystick y: Right Drive
     *
     * Gamepad 2 (Operator):
     *      Triggers:
     *          Right (Shoot direction)
     *          Left (Reverse)
     *          (Right - Left is direction)
     *      Bumpers:
     *          Both or Neither (Centered)
     *          Right (Right hitter)
     *          Left (Left hitter)
     */

    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop()
    {
        /*
         * Driver 1 Stuffs
         *
         * Sticks to drive - Tank drive
        */

        //Driving
        robot.setRightPower(gamepad1.right_stick_y);
        robot.setLeftPower(gamepad1.left_stick_y);




        /*
         *
         * Driver 2 stuff
         *
         * Right Trigger shoot
         * Left Trigger reverse
         *
         */

        if(!(gamepad2.right_bumper ^ gamepad2.left_bumper))//cheeky xor - both or neither
            robot.centerBeacon();
        else if(gamepad2.right_bumper)//right hit, set to hit right button
            robot.hitRightBeacon();
        else//left pressed, set to hit left button
            robot.hitLeftBeacon();

        if(!(gamepad2.right_trigger >0 ^ gamepad2.left_trigger>0))//cheeky xor - both or neither over 1
            robot.stopShooter();
        else if(gamepad2.right_trigger >0)//right hit, shoot
            robot.shoot();
        else//left pressed, reverse shooter
            robot.reverseShoot();
    }



}
