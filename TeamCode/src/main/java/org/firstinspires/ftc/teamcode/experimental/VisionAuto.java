package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by roboticsteam on 3/8/2017.
 */
@Autonomous(name="Vision Auto")
public class VisionAuto extends LinearVisionOpMode{


    //creates a robot to use
    VisionRobot visRobot;

    //Stages of Autonomoose v2.1

    static final double Kp = 1;
    static final double Kd = -1;

    //single iteration variable
    private final boolean communism = true; //Are we the red alliance
    private final int balls2shoot = 2; //Number of balls to shoot

    public void initialize()
    {
        telemetry.addData("Status", "Initializing");

        visRobot = new VisionRobot(hardwareMap, this);

        ///VisionOpMode initialization
        //enable camera extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        visRobot.init();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //init the robot
        this.initialize();
        waitForVisionStart(); //TODO is this necessary

//        //drive to ball
//        visRobot.drive2dist(1, 1, 12, 1500); //straight (at diagonal) until near ball
//        while(visRobot.isBusy()) {
//            visRobot.continueAction();
//            telemetry.addData("auto status", "drive2dist");
//        }
//        telemetry.addData("auto status", "finished drive2dist");

        //drive forward 4 feet
        visRobot.distDriveTicks(1, 1, visRobot.dist2ticks(4));
        while (visRobot.isBusy()) {
            visRobot.continueAction();
            telemetry.addData("auto status", "drive past ball");
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done drive past ball");

        //turn 90 degrees (roughly 2 feet?)
        visRobot.distDriveTicks(-1, 1, visRobot.dist2ticks(2));
        while (visRobot.isBusy()) {
            visRobot.continueAction();
            telemetry.addData("auto status", "turn to beacon");
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done turn to beacon");

        //turn clockwise until beacon seen
        visRobot.detectBeacon(0.7, -0.4, 2000);
        while (visRobot.isBusy()) {
            visRobot.continueAction();
            telemetry.addData("auto status", "search for beacon 1");
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done search for beacon 1");

        //PD to beacon
        visRobot.detectBeacon(0.7, -0.4, 2000);
        while (visRobot.isBusy()) {
            visRobot.continueAction();
            telemetry.addData("auto status", "drive past ball");
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done driving past ball");

        //turn clockwise until beacon seen
        visRobot.detectBeacon(0.7, -0.4, 2000);
        while (visRobot.isBusy()) {
            visRobot.continueAction();
            telemetry.addData("auto status", "drive past ball");
        }
        telemetry.addData("visbot status", visRobot.getState());
        telemetry.addData("auto status", "done driving past ball");
    }
}
