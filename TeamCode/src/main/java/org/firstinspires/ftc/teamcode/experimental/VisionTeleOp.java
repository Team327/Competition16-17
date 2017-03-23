package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by roboticsteam on 2/21/2017.
 */
@TeleOp(name = "Vision TeleOp")
public class VisionTeleOp extends VisionOpMode {
    VisionRobot visionBot=null;
    private NewTeleOp teleOp=null;
    private double Kp=0;
    private double Kd=0;
    private double power = 0.5;
    private int driveTicks = 0;
    private int driveTime=0;
    private double drive2dist = 30;
    private int driveTimeInterval=500;
    private int driveTicksInterval=500;
    private double drive2distInterval=2;
    private boolean prevDLeft=false;
    private boolean prevDRight=false;

    private enum varChange {DRIVE_TIME, LEFT_POWER, RIGHT_POWER}

    /**
     * Controller Map
     *
     * Gamepad 1:
     *      A:
     *          Wall follow (toggle)
     *      B:
     *          Cap Ball Servo (toggle)
     *      X:
     *
     *      Y:
     *          Invert Drive (toggle)
     *      Dpad Up:
     *
     *      Dpad Down:
     *
     *      Dpad Left:
     *
     *      Dpad Right:
     *
     *      Start:
     *
     *      Back:
     *          Cancel
     *      Left Analog Y:
     *          Left Side Drive (Wall Follow Power)
     *      Right Analog Y:
     *          Right Side Drive
     *      Left Bumper:
     *          Intake
     *      Right Bumper:
     *          Reverse Intake
     *      Left Trigger:
     *          Lower Lift
     *      Right Trigger:
     *          Raise lift
     * Gamepad 2:
     *      A:
     *          Shoot (Entire process)
     *      B:
     *          Beacon out (toggle)
     *      X:
     *          Hold to use encoder dist
     *      Y:
     *          Hold to use drive2dist
     *      Dpad Up:
     *          Hit Beacon
     *      Dpad Down:
     *          Backup from Beacon
     *      Dpad Left:
     *          Subtract from drive time
     *      Dpad Right:
     *          Add to drive time
     *      Start:
     *          PD to beacon
     *      Back:
     *          Cancel
     *      Left Analog Y:
     *
     *      Right Analog Y:
     *
     *      Left Bumper:
     *          Detect Beacon
     *      Right Bumper:
     *          Time/Dist/2Dist drive (depends on key held x->dist, y->2dist, none->time)
     *      Left Trigger:
     *
     *      Right Trigger:
     *
     *
     */

    @Override
    public void init()
    {
        telemetry.addData("Beacon_stuff", beacon.getAnalysis().getButtonString());
        visionBot = new VisionRobot(hardwareMap, this);
        ///VisionOpMode initialization
        //enable camera extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        visionBot.init();

        teleOp = new NewTeleOp();
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
        //adds available telemetry from visionBot
        visionBot.logData();

        //says that no button is pressed
        telemetry.addData("Drive Time:", driveTime);

        //Cancels action
        if(gamepad1.back || gamepad2.back)
        {
            visionBot.cancel();
        }

        //PD to Beacon
        else if(gamepad2.start)
        {
            visionBot.PDtoBeacon(Kp, Kd, 5000);
        }

        //Hit beacon
        else if(gamepad2.dpad_up)
        {
            visionBot.hitBeacon(power, power, 5000);

        }

        //Backup from Beacon
        else if(gamepad2.dpad_down)
        {
            visionBot.backupFromBeacon(power, power, 5000);
        }

        //Detect beacon
        else if(gamepad2.left_bumper)
        {
            visionBot.detectBeacon( -power, power, 5000);
        }

        //Time/dist Drive
        //Detracts from the drive time
        else if(gamepad2.dpad_left && !prevDLeft)
        {
            if(gamepad2.x && driveTicks > driveTicksInterval)  {
                //change encoder dist
                driveTicks -= driveTicksInterval;
            } else if(gamepad2.y && drive2dist > drive2distInterval) {
                //change drive2dist
                drive2dist -= drive2distInterval;
            } else if (driveTime > driveTimeInterval) {
                //neither x nor y pressed -> change drive time
                driveTime -= driveTimeInterval;
            }
            prevDLeft = gamepad2.dpad_left;
        }
        else if(!gamepad2.dpad_left && prevDLeft)
        {
            prevDLeft = gamepad2.dpad_left;
        }



        //adds to the drive parameters
        else if(gamepad2.dpad_right && !prevDRight)
        {
            if(gamepad2.x) {
                //change encoder dist
                driveTicks += driveTicksInterval;
            } else if(gamepad2.y) {
                //change drive2dist
                drive2dist += drive2distInterval;
            } else {
                driveTime += driveTimeInterval;
            }
            prevDRight = gamepad2.dpad_right;
        }
        else if(!gamepad2.dpad_right && prevDRight)
        {
            prevDRight = gamepad2.dpad_right;
        }


        //Drives for the certain amount
        else if(gamepad2.right_bumper)
        {
            if(gamepad2.x) {
                visionBot.distDriveTicks(power, power, driveTicks);
            } else if(gamepad2.y) {
                visionBot.drive2dist(power, power, drive2dist, 5000, power>0);
            } else {
                visionBot.timeDrive(power, power, driveTime);
            }
        }

        //continues previous action
        else
        {
            visionBot.continueAction();
        }

        if(!visionBot.isBusy()) {
            //engages loop for regular teleop functions
            teleOp.loop();
        }
    }
}
