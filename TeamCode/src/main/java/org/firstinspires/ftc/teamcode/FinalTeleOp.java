package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by gssmrobotics on 12/9/2016.
 */

@TeleOp(name = "Final TeleOp")
public class FinalTeleOp extends OpMode
{
    DcMotor right, left, shooter;
    Servo beacon;

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
        right=hardwareMap.dcMotor.get("right");
        left=hardwareMap.dcMotor.get("left");
        shooter=hardwareMap.dcMotor.get("shooter");

        beacon = hardwareMap.servo.get("beacon");

        beacon.setPosition(0.5);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
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

        if(!(gamepad2.right_bumper ^ gamepad2.left_bumper))//cheeky xor - both or neither
            beacon.setPosition(.5);
        else if(gamepad2.right_bumper)//right hit, set to hit right button
            beacon.setPosition(.3);
        else//left pressed, set to hit left button
            beacon.setPosition(.7);

        shooter.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }



}
