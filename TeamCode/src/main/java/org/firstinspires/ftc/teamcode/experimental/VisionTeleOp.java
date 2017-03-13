package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by roboticsteam on 2/21/2017.
 */
@TeleOp(name = "Vision TeleOp")
public class VisionTeleOp extends VisionOpMode {
    VisionRobot visionBot;
    private FinalTeleOp teleOp;
    private double Kp;
    private double Kd;
    private int driveTime;
    private int driveTimeInterval;
    private boolean prevDLeft;
    private boolean prevDRight;
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
     *          Invert Drive
     *      Left Bumper:
     *          Cancel Action
     *      Y:
     *          PD to Beacon
     *      X:
     *          Detect Beacon
     *      Dpad Up:
     *          Hit beacon
     *      Dpad Down:
     *          Backup from beacon
     *      Dpad Left:
     *          Subtract from drive time
     *      Dpad Right:
     *          Add to drive time
     *
     *
     *
     *
     * Gamepad 2 (Operator):
     *      Triggers:
     *          Right (Shoot direction)
     *          Left (Reverse)
     *          (Right - Left is direction)
     *
     *
     *
     */

    @Override
    public void init()
    {
        visionBot = new VisionRobot(hardwareMap, this);
        teleOp = new FinalTeleOp();
        teleOp.makeInit(gamepad1, gamepad2, telemetry, hardwareMap);

        Kp = 1;
        Kd = 1;

        driveTime = 1000;
        driveTimeInterval = 500;
        prevDLeft = false;
        prevDRight = false;
    }

    public boolean pressed(Gamepad g)
    {
        return (g.atRest() || g.left_stick_button || g.right_stick_button   //analog/trigger
                || g.a || g.b || g.x || g.y                                 // a b x y
                || g.dpad_left || g.dpad_right || g.dpad_up || g.dpad_down  //dpad
                || g.start || g.back || g.guide);                           //center buttons
    }

    @Override
    public void loop()
    {
        //engages loop for regular teleop functions
        teleOp.loop();

        //adds available telemetry from visionBot
        visionBot.logData();

        //says that no button is pressed
        telemetry.addData("Drive Time:", driveTime);

        //PD to Beacon
        if(gamepad1.y)
        {
            visionBot.PDtoBeacon(Kp, Kd, 5000);

        }

        //Hit beacon
        else if(gamepad1.dpad_up)
        {
            visionBot.hitBeacon(0.5, 0.5, 5000);

        }

        //Backup from Beacon
        else if(gamepad1.dpad_down)
        {
            visionBot.backupFromBeacon(0.5, 0.5, 5000);

        }

        //Detect beacon
        else if(gamepad1.x)
        {
            visionBot.detectBeacon( -0.5, 0.5, 5000);

        }

        //Timed Drive
        //Detracts from the drive time

        else if(gamepad1.dpad_left && !prevDLeft)
        {
            if(driveTime > driveTimeInterval)
            {
                driveTime -= driveTimeInterval;
            }
            prevDLeft = gamepad1.dpad_left;
        }
        else if(!gamepad1.dpad_left && prevDLeft)
        {
            prevDLeft = gamepad1.dpad_left;
        }



        //adds to the drive time

        else if(gamepad1.dpad_right && !prevDRight)
        {
            driveTime += driveTimeInterval;

            prevDRight = gamepad1.dpad_right;
        }
        else if(!gamepad1.dpad_right && prevDRight)
        {
            prevDRight = gamepad1.dpad_right;
        }


        //Drives for the driveTime
        else if(gamepad1.back)
        {
            visionBot.timeDrive(0.7,0.7, driveTime);
        }

        //Cancels action
        else if(gamepad1.left_bumper)
        {
            visionBot.cancel();
        }

        //cancels any action if no button is pressed
        //every function declares pressed true - therefore, it will not cancel
        //  if anything is pressed
        else
        {
            visionBot.continueAction();
        }

    }
}
