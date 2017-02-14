package org.firstinspires.ftc.teamcode.tournament;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by gssmrobotics on 12/9/2016.
 */

@TeleOp(name = "Final TeleOp")
public class FinalTeleOp extends OpMode
{
    Robot robot;

    private long lastFrontChange;
    /**
     * Tank Drive System Description
     *
     * Gamepad 1 (Driver):
     *      Left Joystick y: Left Drive
     *      Right Joystick y: Right Drive
     *
     *      A:
     *          Flipper down
     *      B:
     *          Flipper up
     *
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
     *
     */

    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);
        lastFrontChange = System.currentTimeMillis();
    }

    @Override
    public void loop()
    {
        /*
         * Driver 1 Stuffs
         *
         * Sticks to drive - Tank drive
        */

        if((gamepad1.right_bumper || gamepad1.left_bumper)&& System.currentTimeMillis()-lastFrontChange >300) {
            robot.reverseFront();
            lastFrontChange = System.currentTimeMillis();
        }

        //This locks out the driver if the shooter hits any dpad button
        if (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up) {
            robot.brake();
        } else {
            //Driving
            robot.setRightPower(gamepad1.right_stick_y);
            robot.setLeftPower(gamepad1.left_stick_y);
        }
        if (gamepad1.a) {
            robot.flipIn();
        } else if (gamepad1.b) {
            robot.flipOut();
        } else {
            robot.stopFlipper();

        }




        /*
         *
         * Driver 2 stuff
         *
         * Right Trigger shoot
         * Left Trigger reverse
         *
         */

        if (gamepad2.right_bumper && gamepad2.left_bumper)
            robot.centerBeacon();
        else if (gamepad2.right_bumper)//right hit, set to hit right button
            robot.hitRightBeacon();
        else if (gamepad2.left_bumper)//left pressed, set to hit left button
            robot.hitLeftBeacon();
        else
            robot.centerBeacon();

        if (!(gamepad2.right_trigger > 0 ^ gamepad2.left_trigger > 0))//cheeky xor - both or neither over 1
        {
            robot.stopShooter();
        } else if (gamepad2.right_trigger > 0) //right hit, shoot
        {
            telemetry.addData("Status", System.currentTimeMillis() + ": SHOOTING");
            robot.shoot();
        }
        else//left pressed, reverse shooter
        {
            robot.reverseShoot();
        }

    }


}
