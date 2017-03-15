package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Constants;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by roboticsteam on 3/8/2017.
 */
@Autonomous(name="Vision Auto")
public class VisionAuto extends LinearVisionOpMode{


    //creates a robot to use
    VisionRobot visRobot;

    //variables for necessary things

    boolean red;

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
        telemetry.addData("Did visbot.init()", "yes"); //TODO remove
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //init the robot
        this.initialize();
        waitForVisionStart(); //TODO is this necessary
        telemetry.addData("Starting the match", "true"); //TODO remove
        //drive to ball
        while(visRobot.getFrontDist() > 12 )
        {
            visRobot.setRightPower(1);
            visRobot.setLeftPower(1);
        }
        visRobot.brake();





    }




}
