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

//        visRobot.distDriveDrive; //TODO
    }
}
