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
    private boolean prev2a;
    private boolean prev1a;
    private boolean beaconOut;
    private boolean regDrive;

    private enum LaunchPosition {LAUNCHED, PULLED_BACK, LOADED_BALL, BALL_READY}
    private LaunchPosition launcher;
    private double kp, ki, kd;
    private double loadTime;
    private final double loadDelay = 500;

    /* SYSTEMS

            -Shooter
                -Shooter motor - launch load
                -servo block
            -Drivetrain
                -Regular
                -Wall Follow
                -Invert
            -Beacon Pusher
                -Out
                -In
            -Lift
                -Lift Up/down
                -Servo pull
            -Intake
                -Forward/backward



     */


    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

        prev2a = false;
        prev1a = false;
        beaconOut = false;
        regDrive = true;
        kp = 1;
        ki = 1;
        kd = 1;
        loadTime = System.currentTimeMillis();
        launcher = LaunchPosition.LAUNCHED;

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
     *          Wall follow (toggle)
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
     *          Left Side Drive (Wall Follow Power)
     *      Right Analog Y:
     *          Right Side Drive
     *      Left Bumper:
     *          Intake
     *      Right Bumper:
     *
     *      Left Trigger:
     *
     *      Right Trigger:
     *          Raise lift
     * Gamepad 2:
     *      A:
     *          Shoot
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
        telemetry.addData("LOADING", launcher);


        //drive
        if(regDrive) {
            robot.setRightPower(gamepad1.right_stick_y);
            robot.setLeftPower(gamepad1.left_stick_y);
        }
        else
        {
            robot.wallFollow(kp, ki, kd, gamepad1.left_stick_y);
        }
        //toggle control for drive type
        if( gamepad1.a && !prev1a)
        {
            regDrive = !regDrive;
            prev1a = gamepad1.a;
        }
        else if (!gamepad1.a && prev1a)
        {
            prev1a = gamepad1.a;
        }



        //shooter
        if(gamepad2.a && launcher == LaunchPosition.BALL_READY)
        {
            robot.launchBall(45);                    //TODO ENSURE POSITION
            launcher = LaunchPosition.LAUNCHED;
        }
        if(launcher == LaunchPosition.LAUNCHED && !robot.shooterIsBusy())
        {
            robot.pullBack(315);                    //TODO ENSURE POSITION
            launcher = LaunchPosition.PULLED_BACK;
        }
        if(launcher == LaunchPosition.PULLED_BACK && !robot.shooterIsBusy())
        {

        }



        //Toggle Control for Drive Inversion
        if(gamepad1.y)
        {

        }

        //Intake
        if(gamepad1.left_bumper)
        {
            robot.intake(1);
        }

        //Lift
        robot.lift(gamepad1.left_trigger);
        //Toggle Control for Beacon Pusher
        if(gamepad2.a && !prev2a)
        {

            if (beaconOut)
                robot.retractBeacon();
            else
                robot.pushBeacon();


            beaconOut = !beaconOut;
            prev2a = gamepad2.a;
        }
        else if(!gamepad2.a && prev2a)
        {
            prev2a = gamepad2.a;
        }





    }

}
