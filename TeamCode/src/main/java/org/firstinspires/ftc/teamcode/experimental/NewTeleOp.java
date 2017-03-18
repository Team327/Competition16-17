package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.VisionOpMode;

/**
 * Created by gssmrobotics on 3/15/2017.
 */
@TeleOp(name= "New TeleOp")
public class NewTeleOp extends VisionOpMode {
    Robot robot;
    boolean preva;
    boolean beaconOut;

    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

        preva = false;
        beaconOut = false;

    }

    public void makeInit(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap)
    {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.init();
    }

    /**
     * Controller Map
     *
     * Gamepad 1:
     *      A:
     *
     *      B:
     *          Beacon out (toggle)
     *      X:
     *
     *      Y:
     *
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
     *
     *      Left Analog Y:
     *          Left Side Drive
     *      Right Analog Y:
     *          Right Side Drive
     *      Left Bumper:
     *
     *      Right Bumper:
     *
     *      Left Trigger:
     *
     *      Right Trigger:
     *
     * Gamepad 2:
     *      A:
     *
     *      B:
     *
     *      X:
     *
     *      Y:
     *
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
     *
     *      Left Analog Y:
     *
     *      Right Analog Y:
     *
     *      Left Bumper:
     *
     *      Right Bumper:
     *
     *      Left Trigger:
     *
     *      Right Trigger:
     *
     *
     */

    /**
     * Loops through checking all user inputs which are mapped
     */
    @Override
    public void loop()
    {
        robot.setRightPower(gamepad1.right_stick_y);
        robot.setLeftPower(gamepad1.left_stick_y);

        if(false)
        {

        }

        //Toggle Control for Drive Inversion
        else if(gamepad1.)


        //Toggle Control for Beacon Pusher
        else if(gamepad1.a && !preva)
        {

            if (beaconOut)
                robot.retractBeacon();
            else
                robot.pushBeacon();


            beaconOut = !beaconOut;
            preva = gamepad1.a;
        }
        else if(!gamepd1.a && preva)
        {
            preva = gamepad1.a;
        }





    }

}
