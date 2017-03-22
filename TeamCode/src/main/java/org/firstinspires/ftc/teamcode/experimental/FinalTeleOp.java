package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gssmrobotics on 12/9/2016.
 */

@Disabled
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
     *      Right Bumper:
     *          Invert drive
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
    public void makeInit(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.init();
    }

    @Override
    public void loop()
    {
        /*
         * Driver 1 Stuffs
         *
         * Sticks to drive - Tank drive
        */

        if((gamepad1.right_bumper )&& System.currentTimeMillis()-lastFrontChange >300) {
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

        if (gamepad2.right_trigger > 0) //right hit, shoot

        {
            robot.shoot();
        }

    }


}
