package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
     *      Bumpers:
     *          Invert Drive
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
    }


    @Override
    public void loop()
    {
        //engages loop for regular teleop functions
        teleOp.loop();

        //adds available telemetry from visionBot
        visionBot.logData();

        //says that no button is pressed
        boolean pressed = false;
        telemetry.addData("Drive Time:", driveTime);

        //PD to Beacon
        if(gamepad1.y)
        {
            pressed = true;
            visionBot.PDtoBeacon(Kp, Kd, 5000);

        }

        //Hit beacon
        if(gamepad1.dpad_up)
        {
            pressed = true;
            visionBot.hitBeacon(0.5, 0.5, 5000);

        }

        //Backup from Beacon
        if(gamepad1.dpad_down)
        {
            pressed = true;
            visionBot.backupFromBeacon(0.5, 0.5, 5000);

        }

        //Detect beacon
        if(gamepad1.x)
        {
            pressed = true;
            visionBot.detectBeacon( -0.5, 0.5, 5000);

        }

        //Timed Drive
        //Detracts from the drive time
        if(gamepad1.dpad_left)
        {
            if(driveTime > driveTimeInterval)
            {
                driveTime -= driveTimeInterval;
            }
        }

        //adds to the drive time
        if(gamepad1.dpad_right)
        {
            driveTime += driveTimeInterval;
        }

        //Drives for the driveTime
        if(gamepad1.back)
        {
            pressed = true;
            visionBot.timeDrive(0.7,0.7, driveTime);
        }

        //cancels any action if no button is pressed
        //every function declares pressed true - therefore, it will not cancel
        //  if anything is pressed
        if(!pressed)
        {
            visionBot.cancel();
        }

    }
}
