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

    private boolean prev2b, prev2a; //Gamepad 2
    private boolean prev1a, prev1y, prev1b; //Gamepad 1

    //Telemetry
    private boolean beaconOut;
    private boolean wallFollow;
    private boolean invertedDrive;
    private boolean capBallServoOut;

    private double kp, ki, kd;

    private double loadTime;
    private final double loadDelay = 500;               //TODO FIND RIGHT TIME



    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

        //telemetry
        beaconOut = false;
        invertedDrive = true;
        wallFollow = false;
        capBallServoOut = false;
        loadTime = System.currentTimeMillis();

        //PID Constants
        kp = 1;
        ki = 1;                         //TODO TEST PID CONSTANTS
        kd = 1;

        //Toggle booleans
        prev1a = false;
        prev1b = false;
        prev1y = false;
        prev2b = false;
        prev2a = false;



    }

    /**
     * Allows other opmodes to extend this without actually extending it.
     *
     * @param gamepad1
     * @param gamepad2
     * @param telemetry
     * @param hardwareMap
     */
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
     *
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
        telemetry.addData("ENCODER:", robot.getShooterPos());
        telemetry.addData("BEACON PUSHER OUT:", beaconOut);
        telemetry.addData("CAP BALL SERVO OUT:", capBallServoOut);
        telemetry.addData("INVERTED DRIVE:", invertedDrive);
        telemetry.addData("WALL FOLLOW:", wallFollow);


        //DRIVE-----------------------------------------------

        /**
         * Drive regularly (tank)
         */
        if(!wallFollow) {
            robot.setRightPower(gamepad1.right_stick_y);
            robot.setLeftPower(gamepad1.left_stick_y);
        }
        /**
         * drive with set distance from wall
         */
        else
        {
            robot.wallFollow(kp, ki, kd, gamepad1.left_stick_y, telemetry);
        }

        /**
         * toggle control for wall follow (Compatible with inversion)
         * resets wall follow data when turned off
         */
        if( gamepad1.a && !prev1a)
        {
            telemetry.addData("wf breakpoint", -1); telemetry.update();
            wallFollow = !wallFollow;
            if(!wallFollow)
                    robot.resetWallFollow();
            telemetry.addData("wf breakpoint", 0);
            prev1a = gamepad1.a;
        }
        else if (!gamepad1.a && prev1a)
        {
            prev1a = gamepad1.a;
        }


        /**
         * Toggle Control for Inversion (Compatible with Wall Follow)
         */
        if(gamepad1.y && !prev1y)
        {
            robot.reverseFront();
            invertedDrive = !invertedDrive;
            prev1y = gamepad1.y;
        }
        else if(!gamepad1.y && prev1y)
        {
            prev1y = gamepad1.y;
        }

        //SHOOTER---------------------------------------------


//        if (gamepad2.a && !prev2a)
//        {
//            robot.launch();
//            prev2a = gamepad2.a;
//        } else if (!gamepad2.a && prev2a)
//        {
//            prev2a = gamepad2.a;
//        }

        robot.launch(gamepad2.a);

        //INTAKE----------------------------------------------

        /**
         * Activates Intake if left bumper
         */
        if(gamepad1.left_bumper)
        {
            robot.intake(1);
        } else if (gamepad1.right_bumper)
        {
            robot.intake(-1);
        } else {
            robot.intake(0);
        }

        //LIFT------------------------------------------------

        /**
         * lower if left is greater
         */
        if(gamepad1.left_trigger>gamepad1.right_trigger)
            robot.lower(gamepad1.left_trigger);

        /**
         * raise if right is greater
         */
        else if(gamepad1.right_trigger>gamepad1.left_trigger)
            robot.lift(gamepad1.right_trigger);

        /**
         * Toggle Control for Cap Ball Servo
         */
        if(gamepad1.b && !prev1b)
        {
                if(capBallServoOut)
                    robot.ballIn();
                else
                    robot.ballOut();
                capBallServoOut = !capBallServoOut;
        }
        else if(!gamepad1.b && prev1b)
        {
            prev1b = gamepad1.b;
        }


        //BEACON----------------------------------------------

        /**
         * Toggle Control for Beacon Pusher
         */
        if (gamepad2.b && !prev2b)
        {

            if (beaconOut)
                robot.retractBeacon();
            else
                robot.pushBeacon();


            beaconOut = !beaconOut;
            prev2b = gamepad2.b;
        } else if (!gamepad2.b && prev2b)
        {
            prev2b = gamepad2.b;
        }





    }

}
