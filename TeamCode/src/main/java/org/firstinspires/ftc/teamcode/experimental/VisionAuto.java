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

            //VisionOpMode initialization



        //select camera
        //  PRIMARY is non-selfie camera
        this.setCamera(Cameras.PRIMARY);

        //Set frame size
        this.setFrameSize(new Size(900, 900));

        //enable camera extensions
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

    }

    @Override
    public void runOpMode()
    {
        //init the robot
        this.initialize();

        //drive to ball
        while(visRobot.getFrontDist() > 12 )
        {
            visRobot.setRightPower(1);
            visRobot.setLeftPower(1);
        }
        visRobot.brake();





    }




}
