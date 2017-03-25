package org.firstinspires.ftc.teamcode.competition;

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
    private boolean prev1a, prev1y; //Gamepad 1 prev1b

    //Telemetry
    private boolean beaconOut;
    private boolean wallFollow;
    private boolean invertedDrive;
    //private boolean capBallServoOut;

    private double dpad_motor_vel;
    private double kp, ki, kd;

    private double loadTime;
    private final double loadDelay = 500; //TODO FIND RIGHT TIME



    @Override
    public void init()
    {
        try {
            robot = new Robot(hardwareMap);
        } catch(Exception e) {
            telemetry.addData("ERROR ERROR ERROR", "Unable to connect to robot");
            robot = new SimBot(hardwareMap, this, gamepad2);
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
            //TODO remove this - it's a super security concern
        }

        //telemetry
        beaconOut = false;
        invertedDrive = true;
        wallFollow = false;
        //capBallServoOut = false;
        loadTime = System.currentTimeMillis();

        //PID Constants
        kp = 1;
        ki = 1;                         //TODO TEST PID CONSTANTS
        kd = 1;

        //Toggle booleans
        prev1a = false;
        //prev1b = false;
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
        //telemetry.addData("CAP BALL SERVO OUT:", capBallServoOut);
        telemetry.addData("INVERTED DRIVE:", robot.getDirection());
        telemetry.addData("DISTANCES: ", "FRONT=" + robot.getFrontDist() + "; LEFT_FRONT=" +
                robot.getLeftFrontDist() + "; LEFT_REAR=" + robot.getLeftRearDist());
        telemetry.addData("WALL FOLLOW:", wallFollow);
        telemetry.addData("DRIVE INPUT", " "+gamepad1.left_stick_y+":"+ gamepad1.right_stick_y);

        //DRIVE-----------------------------------------------

        /**
         * Drive regularly (tank)
         */


        double leftDrivePower = -gamepad1.left_stick_y,
                rightDrivePower = -gamepad1.right_stick_y;

        if(gamepad1.right_bumper || gamepad1.left_bumper) {
            //slow down drive if either is clicked
            leftDrivePower = leftDrivePower / 3;
            rightDrivePower = rightDrivePower / 3;
        }

        if(gamepad1.dpad_left)
        {
            robot.setLeftPower(dpad_motor_vel);
        } else if (gamepad1.dpad_right)
        {
            robot.setRightPower(dpad_motor_vel);
        }
        if(gamepad1.a)
        {
            wallFollow = true;
        }
        else
        {
            wallFollow = false;
            robot.resetWallFollow();
        }

        if(!wallFollow) {
            robot.setRightPower(rightDrivePower);
            robot.setLeftPower(leftDrivePower);
        }

        /**
         * drive with set distance from wall
         */
        else
        {
            //robot.wallFollow(kp, ki, kd, leftDrivePower, telemetry);
            robot.wallFollow(kp, ki, kd, leftDrivePower, 20, telemetry); //TODO revert to above
        }

        /**
         * toggle control for wall follow (Compatible with inversion)
         * resets wall follow data when turned off
         *//*
        if( gamepad1.a && !prev1a)
        {
            wallFollow = !wallFollow;
            if(!wallFollow)
                    robot.resetWallFollow();
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

        telemetry.addData("launch breakpoint", 0); telemetry.update();
        robot.launch(gamepad2.a, telemetry);

        //INTAKE----------------------------------------------

        /**
         * Activates Intake if left bumper
         */
        if(gamepad1.left_trigger != 0)
        {
            robot.intake(1);
        } else if (gamepad1.right_trigger != 0)
        {
            robot.intake(-1);
        } else {
            robot.intake(0);
        }

        //LIFT------------------------------------------------

        /**
         * lower if left is greater
         */
//        if(gamepad1.left_trigger>gamepad1.right_trigger)
//            robot.lower(gamepad1.left_trigger);

        /**
         * raise if right is greater
         */
//        else if(gamepad1.right_trigger>gamepad1.left_trigger)
//            robot.lift(gamepad1.right_trigger);

//        else
//        {
//            robot.liftBrake();
//        }

        /**
         * Toggle Control for Cap Ball Servo
         *//**
        if(gamepad1.b && !prev1b)
        {
                if(capBallServoOut)
                    robot.ballIn();
                else
                    robot.ballOut();
                capBallServoOut = !capBallServoOut;
                prev1b = gamepad1.b;
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
